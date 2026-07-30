#![allow(
    clippy::print_stdout,
    reason = "This example intentionally prints the estimated orientations."
)]

use core::time::Duration;

use nalgebra::Vector3;
use uf_ahrs::{Ahrs, AhrsWithDt, Madgwick, MadgwickParams, Mahony, MahonyParams};

fn main() {
    let nominal_dt = Duration::from_millis(10);
    let mut mahony = Mahony::new(nominal_dt, MahonyParams::default());
    let mut madgwick = Madgwick::new(nominal_dt, MadgwickParams::default());

    let sample_periods = [
        Duration::from_millis(10),
        Duration::from_millis(11),
        Duration::from_millis(9),
        Duration::from_millis(12),
    ];

    let gyroscope = Vector3::new(0.0, 0.0, 0.5);
    let accelerometer = Vector3::new(0.0, 0.0, 9.81);
    let magnetometer = Vector3::new(20.0, 0.0, 0.0);

    for dt in sample_periods {
        mahony.update_with_dt(dt, gyroscope, accelerometer, magnetometer);
        madgwick.update_with_dt(dt, gyroscope, accelerometer, magnetometer);
    }

    println!("Mahony:   {:?}", mahony.orientation().euler_angles());
    println!("Madgwick: {:?}", madgwick.orientation().euler_angles());
}
