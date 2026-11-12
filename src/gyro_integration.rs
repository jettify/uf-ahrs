use libm::cosf;
use libm::sinf;
use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

/// Integrates a body-frame angular velocity (rad/s), held constant for `dt` seconds.
pub(crate) fn integrate_gyro(
    orientation: UnitQuaternion<f32>,
    gyro: Vector3<f32>,
    dt: f32,
) -> UnitQuaternion<f32> {
    if dt == 0.0 {
        return orientation;
    }

    let half_rotation = gyro * (0.5 * dt);
    let half_angle = half_rotation.norm();
    // sin(x)/x approaches 1 at zero. This expansion also preserves tiny rotations
    // whose squared norm underflows, without dividing by their angular speed.
    let sinc = if half_angle < 1e-3 {
        1.0 - half_angle * half_angle / 6.0
    } else {
        sinf(half_angle) / half_angle
    };
    let increment = Quaternion::from_parts(cosf(half_angle), half_rotation * sinc);

    // Body-frame increments multiply on the right: q_next = q * exp([0, gyro] dt/2).
    UnitQuaternion::from_quaternion(orientation.quaternion() * increment)
}
