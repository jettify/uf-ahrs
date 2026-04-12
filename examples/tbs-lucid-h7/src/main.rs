#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use core::time::Duration;

use embassy_executor::Spawner;
use embassy_stm32::exti;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::time::mhz;
use embassy_stm32::Config as StmConfig;
use embassy_stm32::{bind_interrupts, dma, peripherals, spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use nalgebra::{UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Mahony, MahonyParams};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_STREAM4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
});

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}

fn log_sample(sample: &icm426xx::Sample) {
    if let Some((ax, ay, az)) = sample.accel {
        info!("accel m/s^2: x={} y={} z={}", ax, ay, az);
    }

    if let Some((gx, gy, gz)) = sample.gyro {
        info!("gyro rad/s: x={} y={} z={}", gx, gy, gz);
    }

    let corrected_temp_c = corrected_fifo_temp_celsius(sample.temperature_celsius);
    info!("temp C: {}", corrected_temp_c);
}

fn log_orientation(orientation: UnitQuaternion<f32>) {
    let q = orientation.quaternion();
    let (roll, pitch, yaw) = orientation.euler_angles();
    info!(
        "q w={} x={} y={} z={} rpy(rad)=({}, {}, {})",
        q.w, q.i, q.j, q.k, roll, pitch, yaw
    );
}

const SPI_FREQ_MHZ: u32 = 10;
const IMU_ODR_HZ: f32 = 8000.0;
const LOG_EVERY_N_SAMPLES: u32 = 200;

fn corrected_fifo_temp_celsius(driver_temp_celsius: f32) -> f32 {
    // Temporary workaround for icm426xx 0.4.0:
    // FIFO Packet-4 temperature is handled as 16-bit by the driver, but the
    // conversion currently uses the FIFO 8-bit scale (2.07 LSB/C), inflating
    // temperature delta-from-25C by ~64x. Undo that inflation in the demo.
    ((driver_temp_celsius - 25.0) / 64.0) + 25.0
}

fn process_sample(sample: &icm426xx::Sample, mahony: &mut Mahony, log_decimation: &mut u32) -> bool {
    *log_decimation = log_decimation.wrapping_add(1);
    let should_log = log_decimation.is_multiple_of(LOG_EVERY_N_SAMPLES);

    if let (Some((gx, gy, gz)), Some((ax, ay, az))) = (sample.gyro, sample.accel) {
        let gyro = Vector3::new(gx, gy, gz);
        let accel = Vector3::new(ax, ay, az);
        let orientation = mahony.update_imu(gyro, accel);

        if should_log {
            log_orientation(orientation);
        }
    }

    if should_log {
        log_sample(sample);
    }

    should_log
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = StmConfig::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            fracn: None,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // used by SPI3. 100Mhz.
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);

    let mut spi_config = spi::Config::default();
    // 8 kHz FIFO traffic needs a fast SPI clock to avoid FIFO overrun.
    spi_config.frequency = mhz(SPI_FREQ_MHZ);

    let spi = spi::Spi::new(
        p.SPI1, p.PA5, p.PD7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, Irqs, spi_config,
    );
    let spi_bus = Mutex::<NoopRawMutex, _>::new(spi);
    let gyro_cs = Output::new(p.PC15, Level::High, Speed::Low);
    let spi_device = SpiDevice::new(&spi_bus, gyro_cs);

    let icm = icm426xx::ICM42688::new(spi_device);
    let imu_config = icm426xx::Config {
        int1_mode: icm426xx::InterruptMode::Pulsed,
        int1_polarity: icm426xx::InterruptPolarity::ActiveHigh,
        rate: icm426xx::OutputDataRate::Hz8000,
        timestamps_are_absolute: false,
    };
    let mut icm = icm.initialize(Delay, imu_config).await.unwrap();
    let mahony_params = MahonyParams::default();
    let sample_period = Duration::from_secs_f32(1.0 / IMU_ODR_HZ);
    let mut mahony = Mahony::new(sample_period, mahony_params);

    let mut gyro_drdy = exti::ExtiInput::new(p.PB2, p.EXTI2, Pull::None, Irqs);
    gyro_drdy.wait_for_low().await;

    let led = Output::new(p.PE3, Level::High, Speed::Low);
    spawner.spawn(blink_task(led).unwrap());

    let mut log_decimation = 0u32;
    loop {
        gyro_drdy.wait_for_rising_edge().await;

        loop {
            match icm.read_sample().await {
                Ok(Some((sample, has_more))) => {
                    let _ = process_sample(&sample, &mut mahony, &mut log_decimation);
                    if !has_more {
                        break;
                    }
                }
                Ok(None) => break,
                Err(icm426xx::Error::FifoOverflow) => {
                    info!("imu fifo overflow; flushing");
                    let _ = icm.reset_fifo().await;
                    break;
                }
                Err(icm426xx::Error::Bus(_)) => {
                    info!("imu bus read error");
                    break;
                }
                Err(icm426xx::Error::WhoAmIMismatch(whoami)) => {
                    info!("unexpected whoami mismatch during read: {}", whoami);
                    break;
                }
            }
        }
    }
}
