use core::time::Duration;
use nalgebra::{Matrix3, Vector3};
use uf_ahrs::{Ahrs, InertialCalibration, MagnetometerCalibration, Mahony, MahonyParams};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);
    let mut mahony = Mahony::new(dt, MahonyParams::default());

    // Calibration coefficients determined by an external calibration
    // procedure (e.g. datasheet, factory calibration, or a calibration
    // routine run once per device).
    let accel_cal = InertialCalibration {
        misalignment: Matrix3::identity(),
        sensitivity: Vector3::new(1.002, 0.998, 1.001),
        offset: Vector3::new(0.01, -0.02, 0.03),
    };
    let mag_cal = MagnetometerCalibration {
        soft_iron: Matrix3::identity(),
        hard_iron: Vector3::new(2.0, -1.5, 0.5),
    };

    // Raw sensor readings, as they would arrive from the driver.
    let gyr = Vector3::new(0.0, 0.0, 0.0);
    let raw_acc = Vector3::new(0.0, 0.0, 9.80);
    let raw_mag = Vector3::new(22.0, -1.5, 0.5);

    // Apply calibration before feeding the filter.
    let acc = accel_cal.calibrate(raw_acc);
    let mag = mag_cal.calibrate(raw_mag);

    let orientation = mahony.update(gyr, acc, mag);

    println!("Mahony orientation: {:?}", orientation.euler_angles());
}
