#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

use core::f32;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

#[derive(Debug)]
pub struct Mahony {
    dt: f32,
    kp: f32,
    ki: f32,
    e_int: Vector3<f32>,
    pub quaternion: UnitQuaternion<f32>,
}

impl Default for Mahony {
    fn default() -> Mahony {
        let quaternion = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony {
            dt: (1.0f32) / (256.0f32),
            kp: 0.5f32,
            ki: 0.0f32,
            e_int: Vector3::new(0.0, 0.0, 0.0),
            quaternion,
        }
    }
}

impl Mahony {
    pub fn new(dt: f32, kp: f32, ki: f32) -> Self {
        let quaternion = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony::new_with_quaternion(dt, kp, ki, quaternion)
    }

    pub fn new_with_quaternion(dt: f32, kp: f32, ki: f32, quaternion: UnitQuaternion<f32>) -> Self {
        Mahony {
            dt,
            kp,
            ki,
            e_int: Vector3::new(0.0, 0.0, 0.0),
            quaternion,
        }
    }

    pub fn update_gyro_with_dt(
        &mut self,
        gyroscope: &Vector3<f32>,
        dt: f32,
    ) -> &UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();
        let q_dot = q * Quaternion::from_parts(0.0, *gyroscope) * 0.5;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * dt);
        &self.quaternion
    }

    pub fn update_gyro(&mut self, gyroscope: &Vector3<f32>) -> &UnitQuaternion<f32> {
        self.update_gyro_with_dt(gyroscope, self.dt)
    }

    pub fn update_imu(
        &mut self,
        gyroscope: &Vector3<f32>,
        accelerometer: &Vector3<f32>,
    ) -> &UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro(gyroscope);
        };
        let v = Vector3::new(
            2.0 * (q[0] * q[2] - q[3] * q[1]),
            2.0 * (q[3] * q[0] + q[1] * q[2]),
            q[3] * q[3] - q[0] * q[0] - q[1] * q[1] + q[2] * q[2],
        );
        let v = self.quaternion.inverse() * Vector3::z();
        let e = accel.cross(&v);
        self.e_int += e * self.dt;

        let gyro = *gyroscope + e * self.kp + self.e_int * self.ki;

        let q_dot = q * Quaternion::from_parts(0.0, gyro) * 0.5;
        self.quaternion = UnitQuaternion::from_quaternion(q + q_dot * self.dt);
        &self.quaternion
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use approx::assert_abs_diff_eq;
    use approx::assert_relative_eq;

    #[test]
    fn test_zero_rotation() {
        let mut ahrs = Mahony::default();
        let gyro = Vector3::new(0.0f32, 0.0f32, 0.0f32);
        ahrs.update_gyro(&gyro);
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_abs_diff_eq!(roll, 0.0);
        assert_abs_diff_eq!(pitch, 0.0);
        assert_abs_diff_eq!(yaw, 0.0);
    }

    #[test]
    fn test_gyro_roll_estimation() {
        let mut ahrs = Mahony::new(0.1, 0.0, 0.0);
        ahrs.update_gyro(&Vector3::new(0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.05, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_pitch_estimation() {
        let mut ahrs = Mahony::new(0.1, 0.0, 0.0);
        ahrs.update_gyro(&Vector3::new(0.0f32, 0.5f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.05, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_yaw_estimation() {
        let mut ahrs = Mahony::new(0.1, 0.0, 0.0);
        ahrs.update_gyro(&Vector3::new(0.0f32, 0.0f32, 0.5f32));
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.05, epsilon = 0.0001);
    }
}
