use core::f32;
use core::time::Duration;

use crate::traits::Ahrs;
use nalgebra::{Quaternion, UnitQuaternion, Vector2, Vector3};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MahonyParams {
    pub kp: f32,
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

#[derive(Debug)]
pub struct Mahony {
    dt: f32,
    params: MahonyParams,
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
    pub fn new(sample_period: Duration, params: MahonyParams) -> Self {
        let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony::new_with_orientation(sample_period, params, orientation)
    }

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
            2.0 * (q[0] * q[2] - q[3] * q[1]),
            2.0 * (q[3] * q[0] + q[1] * q[2]),
            q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2],
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
        let q = self.quaternion.as_ref();
        let two = 2.0;

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };

        let Some(mag) = magnetometer.try_normalize(f32::EPSILON) else {
            return self.update_imu(gyroscope, accelerometer);
        };

        let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
        let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);

        let v = Vector3::new(
            two * (q[0] * q[2] - q[3] * q[1]),
            two * (q[3] * q[0] + q[1] * q[2]),
            q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2],
        );

        let w = Vector3::new(
            2.0 * b[0] * (0.5 - q[1] * q[1] - q[2] * q[2])
                + 2.0 * b[2] * (q[0] * q[2] - q[3] * q[1]),
            2.0 * b[0] * (q[0] * q[1] - q[3] * q[2]) + 2.0 * b[2] * (q[3] * q[0] + q[1] * q[2]),
            2.0 * b[0] * (q[3] * q[1] + q[0] * q[2])
                + 2.0 * b[2] * (0.5 - q[0] * q[0] - q[1] * q[1]),
        );

        let e: Vector3<f32> = accel.cross(&v) + mag.cross(&w);
        let b_dot = -self.params.ki * e;
        self.bias += b_dot * self.dt;

        let corrected = gyroscope + e * self.params.kp - self.bias;
        self.update_gyro(corrected)
    }
}

// Tests live in `tests/ahrs_test.rs` to share coverage across algorithms.
