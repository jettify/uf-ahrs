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
    // Example NSD: 0.0028 deg/s/√Hz converted to rad/s/√Hz.
    // The estimator assumes white noise across the Nyquist bandwidth (fs / 2).
    // Tune the resulting limits using stationary captures of the actual sensor.
    let params =
        GyroBiasCalibrationParams::from_noise_density(Duration::from_millis(1250), dt, 4.8869e-5)?;
    let mut calibration = GyroBiasCalibration::new(params)?;
    let simulated_bias = Vector3::new(0.02, -0.01, 0.03); // rad/s

    // Keep the device stationary during collection. This example simulates
    // readings in the sensor frame, before any sensitivity or alignment changes.
    for sample in 0..126 {
        let noise = if sample % 2 == 0 { 0.001 } else { -0.001 };
        let raw_gyro = simulated_bias + Vector3::repeat(noise);
        match calibration.update(dt, raw_gyro) {
            GyroBiasCalibrationStatus::Collecting => {}
            GyroBiasCalibrationStatus::Rejected(reason) => {
                println!(
                    "Window rejected: {reason:?}; statistics: {:?}",
                    calibration.rejected_statistics()
                );
            }
            GyroBiasCalibrationStatus::InvalidSample => {
                println!("Invalid reading; collecting a new stationary window.");
            }
            GyroBiasCalibrationStatus::Complete(bias) => {
                println!("Calibration completed with bias (rad/s): {bias:?}");
                break;
            }
        }
    }

    let offset = calibration.bias().ok_or("calibration has not completed")?;
    if let Some(stats) = calibration.statistics() {
        println!("Minimum gyro (rad/s): {:?}", stats.minimum);
        println!("Maximum gyro (rad/s): {:?}", stats.maximum);
        println!("Peak-to-peak gyro (rad/s): {:?}", stats.peak_to_peak());
        println!("Drift rate (rad/s²): {:?}", stats.drift_rate);
    }
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
