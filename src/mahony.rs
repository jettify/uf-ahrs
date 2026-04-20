use core::f32;
use core::time::Duration;

use crate::traits::Ahrs;
use nalgebra::{Quaternion, UnitQuaternion, Vector2, Vector3};

/// Tuning parameters for the [`Mahony`] filter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MahonyParams {
    /// Proportional gain for orientation error correction.
    pub kp: f32,
    /// Integral gain for gyroscope bias estimation.
    pub ki: f32,
}

impl Default for MahonyParams {
    fn default() -> Self {
        Self {
            kp: 0.74,
            ki: 0.0012,
        }
    }
}

/// Mahony nonlinear complementary filter for orientation estimation.
#[derive(Debug)]
pub struct Mahony {
    dt: f32,
    params: MahonyParams,
    /// Estimated gyroscope bias in radians per second.
    pub bias: Vector3<f32>,
    quaternion: UnitQuaternion<f32>,
}

impl Default for Mahony {
    fn default() -> Mahony {
        Mahony::new(
            Duration::from_secs_f32(1.0 / 256.0),
            MahonyParams::default(),
        )
    }
}

impl Mahony {
    /// Creates a filter with identity orientation.
    ///
    /// `sample_period` controls gyroscope integration step size.
    pub fn new(sample_period: Duration, params: MahonyParams) -> Self {
        let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony::new_with_orientation(sample_period, params, orientation)
    }

    /// Creates a filter with a custom initial orientation.
    pub fn new_with_orientation(
        sample_period: Duration,
        params: MahonyParams,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        Mahony {
            dt: sample_period.as_secs_f32(),
            params,
            bias: Vector3::new(0.0, 0.0, 0.0),
            quaternion: orientation,
        }
    }

    /// Integrates gyroscope data for one step with an explicit `dt` in seconds.
    ///
    /// `gyroscope` must be in radians per second.
    pub fn update_gyro_with_dt(&mut self, gyroscope: Vector3<f32>, dt: f32) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();
        let q_dot = q * Quaternion::from_parts(0.0, gyroscope) * 0.5;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * dt);
        self.quaternion
    }
}

impl Ahrs for Mahony {
    fn orientation(&self) -> UnitQuaternion<f32> {
        self.quaternion
    }

    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) {
        self.bias = Vector3::new(0.0, 0.0, 0.0);
        self.quaternion = quat;
    }

    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32> {
        self.update_gyro_with_dt(gyroscope, self.dt)
    }

    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };
        let v = Vector3::new(
            2.0 * (q.coords.x * q.coords.z - q.coords.w * q.coords.y),
            2.0 * (q.coords.w * q.coords.x + q.coords.y * q.coords.z),
            q.coords.w * q.coords.w - q.coords.x * q.coords.x - q.coords.y * q.coords.y
                + q.coords.z * q.coords.z,
        );

        let e = accel.cross(&v);
        let b_dot = -self.params.ki * e;
        self.bias += b_dot * self.dt;
        let corrected = gyroscope + e * self.params.kp - self.bias;
        self.update_gyro(corrected)
    }

    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };

        let Some(mag) = magnetometer.try_normalize(f32::EPSILON) else {
            return self.update_imu(gyroscope, accelerometer);
        };

        let q = self.quaternion.as_ref();
        let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
        let b = Quaternion::new(
            0.0,
            Vector2::new(h.coords.x, h.coords.y).norm(),
            0.0,
            h.coords.z,
        );

        let two = 2.0;
        let v = Vector3::new(
            two * (q.coords.x * q.coords.z - q.coords.w * q.coords.y),
            two * (q.coords.w * q.coords.x + q.coords.y * q.coords.z),
            q.coords.w * q.coords.w - q.coords.x * q.coords.x - q.coords.y * q.coords.y
                + q.coords.z * q.coords.z,
        );

        let w = Vector3::new(
            2.0 * b.coords.x * (0.5 - q.coords.y * q.coords.y - q.coords.z * q.coords.z)
                + 2.0 * b.coords.z * (q.coords.x * q.coords.z - q.coords.w * q.coords.y),
            2.0 * b.coords.x * (q.coords.x * q.coords.y - q.coords.w * q.coords.z)
                + 2.0 * b.coords.z * (q.coords.w * q.coords.x + q.coords.y * q.coords.z),
            2.0 * b.coords.x * (q.coords.w * q.coords.y + q.coords.x * q.coords.z)
                + 2.0 * b.coords.z * (0.5 - q.coords.x * q.coords.x - q.coords.y * q.coords.y),
        );

        let e: Vector3<f32> = accel.cross(&v) + mag.cross(&w);
        let b_dot = -self.params.ki * e;
        self.bias += b_dot * self.dt;

        let corrected = gyroscope + e * self.params.kp - self.bias;
        self.update_gyro(corrected)
    }
}

// Tests live in `tests/ahrs_test.rs` to share coverage across algorithms.
