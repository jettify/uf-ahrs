#![allow(
    clippy::print_stdout,
    reason = "This example intentionally prints the calibration and orientation."
)]

use core::time::Duration;

use nalgebra::Vector3;
use uf_ahrs::Ahrs;
use uf_ahrs::GyroBiasCalibration;
use uf_ahrs::GyroBiasCalibrationParams;
use uf_ahrs::GyroBiasCalibrationStatus;
use uf_ahrs::InertialCalibration;
use uf_ahrs::Madgwick;
use uf_ahrs::MadgwickParams;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dt = Duration::from_millis(10);
    let mut calibration = GyroBiasCalibration::new(GyroBiasCalibrationParams::default())?;
    let simulated_bias = Vector3::new(0.02, -0.01, 0.03); // rad/s

    // Keep the device stationary during collection. This example simulates
    // readings in the sensor frame, before any sensitivity or alignment changes.
    for sample in 0..125 {
        let noise = if sample % 2 == 0 { 0.001 } else { -0.001 };
        let raw_gyro = simulated_bias + Vector3::repeat(noise);
        match calibration.update(dt, raw_gyro) {
            GyroBiasCalibrationStatus::Collecting => {}
            GyroBiasCalibrationStatus::MotionDetected => {
                println!("Motion detected; collecting a new stationary window.");
            }
            GyroBiasCalibrationStatus::InvalidSample => {
                println!("Invalid reading; collecting a new stationary window.");
            }
            GyroBiasCalibrationStatus::Complete => break,
        }
    }

    let offset = calibration.bias().ok_or("calibration has not completed")?;
    let gyro_calibration = InertialCalibration {
        offset,
        ..Default::default()
    };
    let mut filter = Madgwick::new(dt, MadgwickParams::default());
    let corrected_gyro = gyro_calibration.calibrate(simulated_bias);
    let orientation = filter.update_imu(corrected_gyro, Vector3::new(0.0, 0.0, 9.81));

    println!("Estimated gyro bias (rad/s): {offset:?}");
    println!("Corrected stationary gyro (rad/s): {corrected_gyro:?}");
    println!("Madgwick orientation: {:?}", orientation.euler_angles());
    Ok(())
}
