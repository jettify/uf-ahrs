#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use core::time::Duration;

use crate::telemetry::{Telemetry, DOWN_TO_UP};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_time::{with_timeout, Delay, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use nalgebra::Vector3;
use panic_probe as _;
use tbs_lucid_h7_bsp as bsp;

use bsp::hal::gpio::{Level, Output, Pull, Speed};
use bsp::hal::{interrupt, spi};
use uf_ahrs::{
    Ahrs, Axis, AxisRemap, GyroBiasCalibration, GyroBiasCalibrationParams,
    GyroBiasCalibrationStatus, InertialCalibration, Mahony, MahonyParams,
};

#[path = "../../common/telemetry.rs"]
mod telemetry;

bsp::bind_board_interrupts!(struct Irqs {
    primary_imu;
    primary_imu_drdy;
});

const IMU_ODR_HZ: u64 = 1_000;
const SAMPLE_PERIOD: Duration = Duration::from_micros(1_000_000 / IMU_ODR_HZ);
const LOG_HZ: u64 = 2;
const LOG_EVERY_N_SAMPLES: u64 = IMU_ODR_HZ / LOG_HZ;
// The driver uses 20-byte high-resolution FIFO packets and a one-packet watermark.
const FIFO_PACKET_BYTES: u16 = core::mem::size_of::<icm426xx::fifo::FifoPacket4>() as u16;
const MAX_DRAIN_SAMPLES: usize = 32;
const FIFO_POLL_TIMEOUT: embassy_time::Duration = embassy_time::Duration::from_millis(10);
const ERROR_BACKOFF: embassy_time::Duration = embassy_time::Duration::from_millis(1);
const CALIBRATION_DURATION: Duration = Duration::from_millis(1_250);
// ICM-42688-P gyro rate noise spectral density: 0.0028 deg/s/√Hz.
const GYRO_NOISE_DENSITY_RAD_S_SQRT_HZ: f32 = 4.8869e-5;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let board = bsp::Board::new(bsp::hal::init(bsp::config()));
    let mut led = Output::new(board.leds.led0, Level::Low, Speed::Low);

    // Betaflight CW90_DEG_FLIP gives forward-left-up. Flip Y and Z for forward-right-down.
    let Some(imu_to_board) = AxisRemap::new(Axis::YPos, Axis::XNeg, Axis::ZPos) else {
        error!("invalid primary IMU axis mapping");
        return;
    };

    let mut spi_config = spi::Config::default();
    spi_config.frequency = bsp::hal::time::mhz(10);
    let imu = board.imu_primary.new_spi(Irqs, spi_config);
    let cs = Output::new(imu.cs, Level::High, Speed::Low);
    let spi_device = match ExclusiveDevice::new_no_delay(imu.spi, cs) {
        Ok(device) => device,
        Err(infallible) => match infallible {},
    };

    let imu_config = icm426xx::Config {
        int1_mode: icm426xx::InterruptMode::Latched,
        int1_polarity: icm426xx::InterruptPolarity::ActiveHigh,
        rate: icm426xx::OutputDataRate::Hz1000,
        timestamps_are_absolute: false,
    };
    let Ok(icm) = icm426xx::ICM42688::new(spi_device)
        .initialize(Delay, imu_config)
        .await
    else {
        error!("IMU initialization failed");
        return;
    };

    let drdy = bsp::parts::setup_primary_imu_drdy(
        imu.int,
        imu.int_exti,
        Irqs,
        Pull::None,
        interrupt::Priority::P6,
    );

    let Ok(calibrator) = GyroBiasCalibrationParams::from_noise_density(
        CALIBRATION_DURATION,
        SAMPLE_PERIOD,
        GYRO_NOISE_DENSITY_RAD_S_SQRT_HZ,
    )
    .and_then(GyroBiasCalibration::new) else {
        error!("invalid gyro calibration parameters");
        return;
    };

    let mut imu = ImuReader::new(icm, drdy);
    let gyro_calibration = calibrate_gyro(&mut imu, calibrator).await;

    let mut mahony =
        Mahony::new_with_orientation(SAMPLE_PERIOD, MahonyParams::default(), DOWN_TO_UP);
    let mut samples_since_log = 0;

    loop {
        let Ok(sample) = imu.next_sample().await else {
            continue;
        };
        let Some((gx, gy, gz)) = sample.gyro else {
            continue;
        };
        let gyro = imu_to_board.remap(gyro_calibration.calibrate(Vector3::new(gx, gy, gz)));
        let Some((ax, ay, az)) = sample.accel else {
            // Preserve this sample's rotation when no gravity correction is available.
            let _orientation = mahony.update_gyro(gyro);
            continue;
        };

        let accel = imu_to_board.remap(Vector3::new(ax, ay, az));
        let orientation = mahony.update_imu(gyro, accel);

        samples_since_log += 1;
        if samples_since_log >= LOG_EVERY_N_SAMPLES {
            samples_since_log = 0;
            led.toggle();
            let telemetry = Telemetry {
                orientation,
                accel,
                gyro,
            };
            #[cfg(feature = "defmt")]
            info!("{}", defmt::Display2Format(&telemetry));
            #[cfg(not(feature = "defmt"))]
            let _ = telemetry;
        }
    }
}

async fn calibrate_gyro<SPI: SpiDevice>(
    imu: &mut ImuReader<'_, SPI>,
    mut calibrator: GyroBiasCalibration,
) -> InertialCalibration {
    info!("keep the board still for gyro bias calibration");
    loop {
        let Ok(sample) = imu.next_sample().await else {
            calibrator.reset();
            continue;
        };
        let Some((gx, gy, gz)) = sample.gyro else {
            continue;
        };
        match calibrator.update(SAMPLE_PERIOD, Vector3::new(gx, gy, gz)) {
            GyroBiasCalibrationStatus::Collecting => {}
            GyroBiasCalibrationStatus::Rejected(reason) => {
                fmt::warn!(
                    "gyro calibration rejected ({}); retrying",
                    defmt::Debug2Format(&reason)
                );
            }
            GyroBiasCalibrationStatus::InvalidSample => {
                fmt::warn!("invalid gyro calibration sample; retrying");
            }
            GyroBiasCalibrationStatus::Complete(offset) => {
                info!(
                    "gyro bias calibrated (sensor frame, rad/s): x={} y={} z={}",
                    offset.x, offset.y, offset.z
                );
                return InertialCalibration {
                    offset,
                    ..Default::default()
                };
            }
        }
    }
}

struct Discontinuity;

struct ImuReader<'d, SPI> {
    icm: icm426xx::ICM42688<SPI, icm426xx::Ready>,
    drdy: bsp::parts::ImuDrdyInput<'d>,
    has_more: bool,
    reads_since_yield: usize,
}

impl<'d, SPI: SpiDevice> ImuReader<'d, SPI> {
    fn new(
        icm: icm426xx::ICM42688<SPI, icm426xx::Ready>,
        drdy: bsp::parts::ImuDrdyInput<'d>,
    ) -> Self {
        Self {
            icm,
            drdy,
            // Drain startup data before waiting for the first notification.
            has_more: true,
            reads_since_yield: 0,
        }
    }

    async fn next_sample(&mut self) -> Result<icm426xx::Sample, Discontinuity> {
        loop {
            if !self.has_more {
                // has_more describes occupancy before the previous packet was consumed;
                // recheck FIFO count to catch samples that arrived during that read.
                match self.icm.read_fifo_count().await {
                    Ok(bytes) if bytes < FIFO_PACKET_BYTES => {
                        self.reads_since_yield = 0;
                        // Already-high INT1 wakes immediately; timeout covers missed notifications.
                        let _wake =
                            with_timeout(FIFO_POLL_TIMEOUT, self.drdy.wait_for_high()).await;
                    }
                    Ok(_) => {}
                    Err(err) => {
                        self.recover_from_error(icm426xx::Error::Bus(err)).await;
                        return Err(Discontinuity);
                    }
                }
            }

            if self.reads_since_yield >= MAX_DRAIN_SAMPLES {
                self.reads_since_yield = 0;
                yield_now().await;
            }

            self.reads_since_yield += 1;
            match self.icm.read_sample().await {
                Ok(Some((sample, has_more))) => {
                    self.has_more = has_more;
                    return Ok(sample);
                }
                Ok(None) => {
                    self.has_more = false;
                }
                Err(err) => {
                    self.recover_from_error(err).await;
                    return Err(Discontinuity);
                }
            }
        }
    }

    /// Restore packet alignment before allowing any further sample reads.
    async fn recover_from_error(&mut self, err: icm426xx::Error<SPI::Error>) {
        match err {
            icm426xx::Error::FifoOverflow => fmt::warn!("IMU FIFO overflow; resetting FIFO"),
            icm426xx::Error::Bus(_) => error!("IMU bus read failed; resetting FIFO"),
            icm426xx::Error::WhoAmIMismatch(whoami) => {
                error!("unexpected IMU identity: {:#04x}", whoami);
            }
        }
        self.has_more = false;
        self.reads_since_yield = 0;
        Timer::after(ERROR_BACKOFF).await;
        while self.icm.reset_fifo().await.is_err() {
            fmt::warn!("IMU FIFO reset failed; retrying");
            Timer::after(ERROR_BACKOFF).await;
        }
    }
}
