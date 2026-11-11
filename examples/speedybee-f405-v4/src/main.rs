#![no_std]
#![no_main]

use core::time::Duration;

use crate::telemetry::{DOWN_TO_UP, Telemetry};

use embassy_executor::Spawner;
use embassy_time::Delay;
use embassy_time::Timer;
use embedded_hal_bus::spi::ExclusiveDevice;
use nalgebra::Vector3;
use panic_halt as _;
use speedybee_f405_v4_bsp as bsp;

use bsp::hal::exti;
use bsp::hal::gpio::{Level, Output, Pull, Speed};
use bsp::hal::{interrupt, spi};
use uf_ahrs::{
    Ahrs, Axis, AxisRemap, GyroBiasCalibration, GyroBiasCalibrationParams,
    GyroBiasCalibrationStatus, InertialCalibration, Mahony, MahonyParams,
};

#[path = "../../common/telemetry.rs"]
mod telemetry;
mod usb_logger;

bsp::bind_board_interrupts!(struct Irqs {
    primary_imu;
    primary_imu_drdy;
    usb_fs;
});

const IMU_ODR_HZ: u64 = 1_000;
const SAMPLE_PERIOD: Duration = Duration::from_micros(1_000_000 / IMU_ODR_HZ);
const LOG_HZ: u64 = 2;
const LOG_EVERY_N_SAMPLES: u64 = IMU_ODR_HZ / LOG_HZ;
const CALIBRATION_DURATION: Duration = Duration::from_millis(1_250);
// ICM-42688-P gyro rate noise spectral density: 0.0028 deg/s/√Hz.
const GYRO_NOISE_DENSITY_RAD_S_SQRT_HZ: f32 = 4.8869e-5;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let board = bsp::Board::new(bsp::hal::init(bsp::config_with_usb()));
    let mut led = Output::new(board.leds.led0, Level::Low, Speed::Low);
    if let Ok(task) = usb_logger::run(board.usb, Irqs) {
        spawner.spawn(task);
    }
    Timer::after_millis(1_000).await;

    // Betaflight CW90_DEG gives forward-left-up. Flip Y and Z for forward-right-down.
    let Some(imu_to_board) = AxisRemap::new(Axis::YPos, Axis::XPos, Axis::ZNeg) else {
        log::error!("invalid primary IMU axis mapping");
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
        int1_mode: icm426xx::InterruptMode::Pulsed,
        int1_polarity: icm426xx::InterruptPolarity::ActiveHigh,
        rate: icm426xx::OutputDataRate::Hz1000,
        timestamps_are_absolute: false,
    };
    let Ok(mut icm) = icm426xx::ICM42688::new(spi_device)
        .initialize(Delay, imu_config)
        .await
    else {
        log::error!("IMU initialization failed");
        return;
    };

    let Ok(mut drdy) = bsp::parts::setup_primary_imu_drdy(
        imu.int,
        imu.int_exti,
        Irqs,
        Pull::None,
        exti::TriggerEdge::Rising,
        interrupt::Priority::P6,
    ) else {
        log::error!("IMU data-ready latch already claimed");
        return;
    };

    let Ok(mut calibrator) = GyroBiasCalibrationParams::from_noise_density(
        CALIBRATION_DURATION,
        SAMPLE_PERIOD,
        GYRO_NOISE_DENSITY_RAD_S_SQRT_HZ,
    )
    .and_then(GyroBiasCalibration::new) else {
        log::error!("invalid gyro calibration parameters");
        return;
    };
    let mut gyro_calibration: Option<InertialCalibration> = None;
    let mut mahony =
        Mahony::new_with_orientation(SAMPLE_PERIOD, MahonyParams::default(), DOWN_TO_UP);
    let mut samples_since_log = 0;
    log::info!("keep the board still for gyro bias calibration");

    loop {
        let _ = drdy.wait().await;

        loop {
            match icm.read_sample().await {
                Ok(Some((sample, has_more))) => {
                    if let Some((gx, gy, gz)) = sample.gyro {
                        let raw_gyro = Vector3::new(gx, gy, gz);
                        match (&gyro_calibration, sample.accel) {
                            (Some(calibration), Some((ax, ay, az))) => {
                                let gyro = imu_to_board.remap(calibration.calibrate(raw_gyro));
                                let accel = imu_to_board.remap(Vector3::new(ax, ay, az));
                                let orientation = mahony.update_imu(gyro, accel);

                                samples_since_log += 1;
                                if samples_since_log == LOG_EVERY_N_SAMPLES {
                                    samples_since_log = 0;
                                    led.toggle();
                                    let telemetry = Telemetry {
                                        orientation,
                                        accel,
                                        gyro,
                                    };
                                    log::info!("{telemetry}");
                                }
                            }
                            (None, _) => {
                                gyro_calibration = step_gyro_calibration(&mut calibrator, raw_gyro);
                            }
                            (Some(_), None) => {}
                        }
                    }

                    if !has_more {
                        break;
                    }
                }
                Ok(None) => break,
                Err(icm426xx::Error::FifoOverflow) => {
                    log::warn!("IMU FIFO overflow; resetting FIFO");
                    if gyro_calibration.is_none() {
                        calibrator.reset();
                    }
                    if icm.reset_fifo().await.is_err() {
                        log::error!("IMU FIFO reset failed");
                    }
                    break;
                }
                Err(icm426xx::Error::Bus(_)) => {
                    log::error!("IMU bus read failed");
                    if gyro_calibration.is_none() {
                        calibrator.reset();
                    }
                    break;
                }
                Err(icm426xx::Error::WhoAmIMismatch(whoami)) => {
                    log::error!("unexpected IMU identity: {whoami:#04x}");
                    return;
                }
            }
        }
    }
}

fn step_gyro_calibration(
    calibrator: &mut GyroBiasCalibration,
    gyro: Vector3<f32>,
) -> Option<InertialCalibration> {
    match calibrator.update(SAMPLE_PERIOD, gyro) {
        GyroBiasCalibrationStatus::Collecting => None,
        GyroBiasCalibrationStatus::Rejected(reason) => {
            log::warn!("gyro calibration rejected ({reason:?}); retrying");
            None
        }
        GyroBiasCalibrationStatus::InvalidSample => {
            log::warn!("invalid gyro calibration sample; retrying");
            None
        }
        GyroBiasCalibrationStatus::Complete(offset) => {
            log::info!(
                "gyro bias calibrated (sensor frame, rad/s): x={} y={} z={}",
                offset.x,
                offset.y,
                offset.z
            );
            Some(InertialCalibration {
                offset,
                ..Default::default()
            })
        }
    }
}
