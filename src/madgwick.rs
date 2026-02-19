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
        let b = Quaternion::new(
            0.0,
            Vector2::new(h.coords.x, h.coords.y).norm(),
            0.0,
            h.coords.z,
        );

        let f = Vector6::new(
            2.0 * (q.coords.x * q.coords.z - q.coords.w * q.coords.y) - accel.x,
            2.0 * (q.coords.w * q.coords.x + q.coords.y * q.coords.z) - accel.y,
            2.0 * (half - q.coords.x * q.coords.x - q.coords.y * q.coords.y) - accel.z,
            2.0 * b.coords.x * (half - q.coords.y * q.coords.y - q.coords.z * q.coords.z)
                + 2.0 * b.coords.z * (q.coords.x * q.coords.z - q.coords.w * q.coords.y)
                - mag.x,
            2.0 * b.coords.x * (q.coords.x * q.coords.y - q.coords.w * q.coords.z)
                + 2.0 * b.coords.z * (q.coords.w * q.coords.x + q.coords.y * q.coords.z)
                - mag.y,
            2.0 * b.coords.x * (q.coords.w * q.coords.y + q.coords.x * q.coords.z)
                + 2.0 * b.coords.z * (half - q.coords.x * q.coords.x - q.coords.y * q.coords.y)
                - mag.z,
        );

        let j_t = Matrix6::new(
            -2.0 * q.coords.y,
            2.0 * q.coords.x,
            0.0,
            -2.0 * b.coords.z * q.coords.y,
            -2.0 * b.coords.x * q.coords.z + 2.0 * b.coords.z * q.coords.x,
            2.0 * b.coords.x * q.coords.y,
            2.0 * q.coords.z,
            2.0 * q.coords.w,
            -4.0 * q.coords.x,
            2.0 * b.coords.z * q.coords.z,
            2.0 * b.coords.x * q.coords.y + 2.0 * b.coords.z * q.coords.w,
            2.0 * b.coords.x * q.coords.z - 4.0 * b.coords.z * q.coords.x,
            -2.0 * q.coords.w,
            2.0 * q.coords.z,
            -4.0 * q.coords.y,
            -4.0 * b.coords.x * q.coords.y - 2.0 * b.coords.z * q.coords.w,
            2.0 * b.coords.x * q.coords.x + 2.0 * b.coords.z * q.coords.z,
            2.0 * b.coords.x * q.coords.w - 4.0 * b.coords.z * q.coords.y,
            2.0 * q.coords.x,
            2.0 * q.coords.y,
            0.0,
            -4.0 * b.coords.x * q.coords.z + 2.0 * b.coords.z * q.coords.x,
            -2.0 * b.coords.x * q.coords.w + 2.0 * b.coords.z * q.coords.y,
            2.0 * b.coords.x * q.coords.x,
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
            - Quaternion::new(step.x, step.y, step.z, step.w) * self.params.beta;
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
            2.0 * (q.coords.x * q.coords.z - q.coords.w * q.coords.y) - accel.x,
            2.0 * (q.coords.w * q.coords.x + q.coords.y * q.coords.z) - accel.y,
            2.0 * (half - q.coords.x * q.coords.x - q.coords.y * q.coords.y) - accel.z,
            0.0,
        );

        let j_t = Matrix4::new(
            -2.0 * q.coords.y,
            2.0 * q.coords.x,
            0.0,
            0.0,
            2.0 * q.coords.z,
            2.0 * q.coords.w,
            -4.0 * q.coords.x,
            0.0,
            -2.0 * q.coords.w,
            2.0 * q.coords.z,
            -4.0 * q.coords.y,
            0.0,
            2.0 * q.coords.x,
            2.0 * q.coords.y,
            0.0,
            0.0,
        );
        let Some(step) = (j_t * f).try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };
        let q_dot = (q * Quaternion::from_parts(0.0, gyroscope)) * half
            - Quaternion::new(step.x, step.y, step.z, step.w) * self.params.beta;
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

// Tests live in `tests/ahrs_test.rs` to share coverage across algorithms.
