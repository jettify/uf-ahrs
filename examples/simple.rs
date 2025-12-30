use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParameters};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);

    let mut mahony = Mahony::new(dt, MahonyParams::default());
    let mut madgwick = Madgwick::new(dt, MadgwickParams::default());
    let mut vqf = Vqf::new(dt, VqfParameters::default());

    // Sensor data
    let gyr = Vector3::new(0.0, 0.0, 0.0);
    let acc = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(20.0, 0.0, 0.0);

    mahony.update(gyr, acc, mag);
    madgwick.update(gyr, acc, mag);
    vqf.update(gyr, acc, mag);

    // Get orientation as UnitQuaternion
    let q_mahony = mahony.orientation();
    let q_madgwick = madgwick.orientation();
    let q_vqf = vqf.orientation();

    println!("Mahony:   {:?}", q_mahony.euler_angles());
    println!("Madgwick: {:?}", q_madgwick.euler_angles());
    println!("VQF:      {:?}", q_vqf.euler_angles());
}
