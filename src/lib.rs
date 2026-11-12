#![no_std]
#![allow(
    clippy::needless_doctest_main,
    reason = "This is readme example, not doctest"
)]
#![doc = include_str!("../README.md")]

mod calibration;
mod gyro_bias_calibration;
mod gyro_integration;
mod madgwick;
mod mahony;
pub(crate) mod mean_init_lfp;
mod remap;
mod traits;
mod vqf;

pub use traits::Ahrs;
pub use traits::AhrsWithDt;

pub use calibration::InertialCalibration;
pub use calibration::MagnetometerCalibration;
pub use gyro_bias_calibration::GyroBiasCalibration;
pub use gyro_bias_calibration::GyroBiasCalibrationError;
pub use gyro_bias_calibration::GyroBiasCalibrationParams;
pub use gyro_bias_calibration::GyroBiasCalibrationRejectionReason;
pub use gyro_bias_calibration::GyroBiasCalibrationStatistics;
pub use gyro_bias_calibration::GyroBiasCalibrationStatus;
pub use remap::Axis;
pub use remap::AxisRemap;

pub use madgwick::Madgwick;
pub use madgwick::MadgwickParams;

pub use mahony::Mahony;
pub use mahony::MahonyParams;

pub use vqf::Vqf;
pub use vqf::VqfParams;
