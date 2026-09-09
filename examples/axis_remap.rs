use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Axis, AxisRemap, Mahony, MahonyParams};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);
    let mut mahony = Mahony::new(dt, MahonyParams::default());

    // IMU mounted rotated 90 degrees about Z relative to the airframe: the
    // body's X axis reads the sensor's Y axis, and the body's Y axis reads
    // the sensor's negated X axis.
    let remap = AxisRemap::new(Axis::YPos, Axis::XNeg, Axis::ZPos).expect("valid permutation");

    // Raw sensor readings, in the sensor's own mounting frame.
    let raw_gyr = Vector3::new(0.0, 0.0, 0.0);
    let raw_acc = Vector3::new(0.0, 0.0, 9.81);
    let raw_mag = Vector3::new(20.0, 0.0, 0.0);

    // Remap into the body frame before feeding the filter.
    let gyr = remap.remap(raw_gyr);
    let acc = remap.remap(raw_acc);
    let mag = remap.remap(raw_mag);

    let orientation = mahony.update(gyr, acc, mag);

    println!("Mahony orientation: {:?}", orientation.euler_angles());
}
