use core::time::Duration;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParams};

fn integrate_orientation(
    orientation: UnitQuaternion<f32>,
    gyro: Vector3<f32>,
    dt: f32,
) -> UnitQuaternion<f32> {
    let q = orientation.as_ref();
    let q_dot = q * Quaternion::from_parts(0.0, gyro) * 0.5;
    UnitQuaternion::from_quaternion(q + q_dot * dt)
}

fn print_result(name: &str, estimate: UnitQuaternion<f32>, target: UnitQuaternion<f32>) {
    let (roll, pitch, yaw) = estimate.euler_angles();
    let error = (estimate * target.inverse()).angle().to_degrees();

    println!(
        "{name:10} error={:>6.2} deg final=({:>7.2}, {:>7.2}, {:>7.2}) deg",
        error,
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees(),
    );
}

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);
    let gyro = Vector3::new(10.0_f32.to_radians(), 0.0, 35.0_f32.to_radians());
    let gravity = Vector3::new(0.0, 0.0, 9.81);
    let magnetic_field = Vector3::new(20.0, 0.0, -45.0);

    let mut target = UnitQuaternion::identity();
    let mut mahony = Mahony::new(dt, MahonyParams::default());
    let mut madgwick = Madgwick::new(dt, MadgwickParams::default());
    let mut vqf = Vqf::new(dt, VqfParams::default());

    for _ in 0..100_000 {
        target = integrate_orientation(target, gyro, dt.as_secs_f32());
        let accel = target.inverse_transform_vector(&gravity);
        let mag = target.inverse_transform_vector(&magnetic_field);

        mahony.update(gyro, accel, mag);
        madgwick.update(gyro, accel, mag);
        vqf.update(gyro, accel, mag);
    }

    println!("Comparison against synthetic motion:");
    print_result("Mahony", mahony.orientation(), target);
    print_result("Madgwick", madgwick.orientation(), target);
    print_result("VQF", vqf.orientation(), target);
}
