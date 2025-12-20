use core::f32;
use core::time::Duration;

use crate::traits::Ahrs;
use nalgebra::{Matrix4, Matrix6, Quaternion, UnitQuaternion, Vector2, Vector3, Vector4, Vector6};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MadgwickParams {
    pub beta: f32,
}

impl Default for MadgwickParams {
    fn default() -> Self {
        Self { beta: 0.05 }
    }
}

#[derive(Debug)]
pub struct Madgwick {
    dt: f32,
    params: MadgwickParams,
    quaternion: UnitQuaternion<f32>,
}

impl Default for Madgwick {
    fn default() -> Madgwick {
        Madgwick::new(
            Duration::from_secs_f32(1.0 / 256.0),
            MadgwickParams::default(),
        )
    }
}

impl Madgwick {
    pub fn new(sample_period: Duration, params: MadgwickParams) -> Self {
        Madgwick::new_with_orientation(
            sample_period,
            params,
            UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        )
    }

    pub fn new_with_orientation(
        sample_period: Duration,
        params: MadgwickParams,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        Madgwick {
            dt: sample_period.as_secs_f32(),
            params,
            quaternion: orientation,
        }
    }
}

impl Ahrs for Madgwick {
    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) {
        self.quaternion = quat;
    }

    fn orientation(&self) -> UnitQuaternion<f32> {
        self.quaternion
    }
    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let half: f32 = 0.5;
        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };

        let Some(mag) = magnetometer.try_normalize(f32::EPSILON) else {
            return self.update_imu(gyroscope, accelerometer);
        };

        let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
        let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);

        let f = Vector6::new(
            2.0 * (q[0] * q[2] - q[3] * q[1]) - accel[0],
            2.0 * (q[3] * q[0] + q[1] * q[2]) - accel[1],
            2.0 * (half - q[0] * q[0] - q[1] * q[1]) - accel[2],
            2.0 * b[0] * (half - q[1] * q[1] - q[2] * q[2])
                + 2.0 * b[2] * (q[0] * q[2] - q[3] * q[1])
                - mag[0],
            2.0 * b[0] * (q[0] * q[1] - q[3] * q[2]) + 2.0 * b[2] * (q[3] * q[0] + q[1] * q[2])
                - mag[1],
            2.0 * b[0] * (q[3] * q[1] + q[0] * q[2])
                + 2.0 * b[2] * (half - q[0] * q[0] - q[1] * q[1])
                - mag[2],
        );

        let j_t = Matrix6::new(
            -2.0 * q[1],
            2.0 * q[0],
            0.0,
            -2.0 * b[2] * q[1],
            -2.0 * b[0] * q[2] + 2.0 * b[2] * q[0],
            2.0 * b[0] * q[1],
            2.0 * q[2],
            2.0 * q[3],
            -4.0 * q[0],
            2.0 * b[2] * q[2],
            2.0 * b[0] * q[1] + 2.0 * b[2] * q[3],
            2.0 * b[0] * q[2] - 4.0 * b[2] * q[0],
            -2.0 * q[3],
            2.0 * q[2],
            -4.0 * q[1],
            -4.0 * b[0] * q[1] - 2.0 * b[2] * q[3],
            2.0 * b[0] * q[0] + 2.0 * b[2] * q[2],
            2.0 * b[0] * q[3] - 4.0 * b[2] * q[1],
            2.0 * q[0],
            2.0 * q[1],
            0.0,
            -4.0 * b[0] * q[2] + 2.0 * b[2] * q[0],
            -2.0 * b[0] * q[3] + 2.0 * b[2] * q[1],
            2.0 * b[0] * q[0],
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );

        let Some(step) = (j_t * f).try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };
        let q_dot = q * Quaternion::from_parts(0.0, gyroscope) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.params.beta;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * self.dt);
        self.quaternion
    }

    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let half: f32 = 0.5;

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };

        let f = Vector4::new(
            2.0 * (q[0] * q[2] - q[3] * q[1]) - accel[0],
            2.0 * (q[3] * q[0] + q[1] * q[2]) - accel[1],
            2.0 * (half - q[0] * q[0] - q[1] * q[1]) - accel[2],
            0.0,
        );

        let j_t = Matrix4::new(
            -2.0 * q[1],
            2.0 * q[0],
            0.0,
            0.0,
            2.0 * q[2],
            2.0 * q[3],
            -4.0 * q[0],
            0.0,
            -2.0 * q[3],
            2.0 * q[2],
            -4.0 * q[1],
            0.0,
            2.0 * q[0],
            2.0 * q[1],
            0.0,
            0.0,
        );
        let Some(step) = (j_t * f).try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };
        let q_dot = (q * Quaternion::from_parts(0.0, gyroscope)) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.params.beta;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * self.dt);
        self.quaternion
    }

    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();
        let half: f32 = 0.5;
        let q_dot = q * Quaternion::from_parts(0.0, gyroscope) * half;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * self.dt);
        self.quaternion
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use approx::assert_abs_diff_eq;
    use approx::assert_relative_eq;

    #[test]
    fn test_no_rotation() {
        let mut ahrs = Madgwick::default();
        let gyro = Vector3::new(0.0f32, 0.0f32, 0.0f32);
        ahrs.update_gyro(gyro);
        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_abs_diff_eq!(roll, 0.0);
        assert_abs_diff_eq!(pitch, 0.0);
        assert_abs_diff_eq!(yaw, 0.0);
    }

    #[test]
    fn test_gyro_roll_estimation() {
        let sample_period = Duration::from_millis(100);
        let mut ahrs = Madgwick::new(sample_period, MadgwickParams::default());
        ahrs.update_gyro(Vector3::new(0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_relative_eq!(roll, 0.05, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);

        ahrs.update_gyro(Vector3::new(-0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_pitch_estimation() {
        let sample_period = Duration::from_millis(100);
        let mut ahrs = Madgwick::new(sample_period, MadgwickParams::default());
        ahrs.update_gyro(Vector3::new(0.0f32, 0.5f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.05, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_yaw_estimation() {
        let sample_period = Duration::from_millis(100);
        let mut ahrs = Madgwick::new(sample_period, MadgwickParams::default());
        ahrs.update_gyro(Vector3::new(0.0f32, 0.0f32, 0.5f32));
        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.05, epsilon = 0.0001);
    }
}
