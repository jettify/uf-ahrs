#![allow(
    clippy::print_stdout,
    reason = "This example prints the estimated heading."
)]

use core::time::Duration;

use uf_ahrs::Ahrs;
use uf_ahrs::Mahony;
use uf_ahrs::MahonyParams;

use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

fn main() {
    let dt = Duration::from_millis(10); // 100 Hz
    // Start with zero roll, pitch, and yaw (Euler angles are in radians).
    let initial_orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
    let mut filter =
        Mahony::new_with_orientation(dt, MahonyParams::default(), initial_orientation);

    // Set the initial yaw instantly, preserving the current roll and pitch.
    // Mahony::set_orientation also resets its bias estimate and accelerometer prefilter.
    let initial_heading = (-45.0_f32).to_radians();
    let (roll, pitch, _) = filter.orientation().euler_angles();
    filter.set_orientation(UnitQuaternion::from_euler_angles(
        roll,
        pitch,
        initial_heading,
    ));

    let (roll, pitch, yaw) = filter.orientation().euler_angles();
    println!(
        "Initial orientation: roll={:.1} deg pitch={:.1} deg yaw={:.1} deg",
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees(),
    );

    // Simulated stationary, level IMU and an external heading source.
    let gyro = Vector3::zeros(); // rad/s
    let accel = Vector3::new(0.0, 0.0, 9.81);
    // Heading uses the orientation's positive yaw convention, in radians.
    let heading = 90.0_f32.to_radians();

    // Feed each new sample into the filter. Heading converges gradually;
    // update_external_heading does not set yaw instantly.
    for _ in 0..1000 {
        filter.update_external_heading(gyro, accel, heading);
    }

    let (_, _, yaw) = filter.orientation().euler_angles();
    println!("External heading: {:.1} deg", heading.to_degrees());
    println!("Estimated yaw:    {:.1} deg", yaw.to_degrees());
}
