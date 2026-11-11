use core::fmt;

use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

// A half turn about X changes forward-right-down into forward-left-up.
// At level, this maps the demo's FRD body frame into Mahony's up-positive world frame.
pub const DOWN_TO_UP: UnitQuaternion<f32> =
    UnitQuaternion::new_unchecked(Quaternion::new(0.0, 1.0, 0.0, 0.0));

pub struct Telemetry {
    /// Mahony's attitude from the FRD body frame to its up-positive world frame.
    pub orientation: UnitQuaternion<f32>,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
}

impl fmt::Display for Telemetry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (roll, pitch, yaw) = (DOWN_TO_UP.inverse() * self.orientation).euler_angles();
        write!(
            f,
            "roll {:+7.2} pitch {:+7.2} yaw {:+7.2} deg | accel (m/s^2) xyz {:+7.2} {:+7.2} {:+7.2} | gyro (deg/s) xyz {:+8.2} {:+8.2} {:+8.2}",
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees(),
            self.accel.x,
            self.accel.y,
            self.accel.z,
            self.gyro.x.to_degrees(),
            self.gyro.y.to_degrees(),
            self.gyro.z.to_degrees(),
        )
    }
}
