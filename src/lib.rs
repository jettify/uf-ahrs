#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

use core::f32;

use nalgebra::{Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

pub fn quat_from_acc_mag(acc: &Vector3<f32>, mag: &Vector3<f32>) -> UnitQuaternion<f32> {
    let z_body = acc.normalize();
    let y_body = acc.cross(mag).normalize();
    let x_body = y_body.cross(&z_body).normalize();
    let rot = Rotation3::from_basis_unchecked(&[x_body, y_body, z_body]);
    UnitQuaternion::from_rotation_matrix(&rot)
}

#[derive(Debug)]
pub struct Mahony {
    dt: f32,
    kp: f32,
    ki: f32,
    pub bias: Vector3<f32>,
    pub quaternion: UnitQuaternion<f32>,
}

impl Default for Mahony {
    fn default() -> Mahony {
        let quaternion = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony {
            dt: (1.0f32) / (256.0f32),
            kp: 0.5f32,
            ki: 0.0f32,
            bias: Vector3::new(0.0, 0.0, 0.0),
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
            bias: Vector3::new(0.0, 0.0, 0.0),
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

        let e = accel.cross(&v);
        let b_dot = -self.ki * e;
        self.bias += b_dot * self.dt;
        let corrected = *gyroscope + e * self.kp - self.bias;
        self.update_gyro(&corrected)
    }

    pub fn update(
        &mut self,
        gyroscope: &Vector3<f32>,
        accelerometer: &Vector3<f32>,
        magnetometer: &Vector3<f32>,
    ) -> &UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();
        let two = 2.0;

        let Some(accel) = accelerometer.try_normalize(0.0) else {
            return self.update_gyro(gyroscope);
        };

        let Some(mag) = magnetometer.try_normalize(0.0) else {
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
        let b_dot = -self.ki * e;
        self.bias += b_dot * self.dt;

        let corrected = *gyroscope + e * self.kp - self.bias;
        self.update_gyro(&corrected)
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

        ahrs.update_gyro(&Vector3::new(-0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
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

    #[test]
    fn test_quat_from_acc_mag_identity() {
        // Body Z-axis points down (aligned with Earth Z)
        let acc = Vector3::new(0.0, 0.0, 1.0);
        // Body X-axis points North (aligned with Earth X)
        let mag = Vector3::new(1.0, 0.0, 0.0);
        let quat = quat_from_acc_mag(&acc, &mag);
        let (roll, pitch, yaw) = quat.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_quat_from_acc_mag_yaw_90() {
        // Body Z-axis points down
        let acc = Vector3::new(0.0, 0.0, 1.0);
        // Body Y-axis points North
        let mag = Vector3::new(0.0, 1.0, 0.0);
        let quat = quat_from_acc_mag(&acc, &mag);
        let (roll, pitch, yaw) = quat.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
    }

    #[test]
    fn test_quat_from_acc_mag_pitch_neg_90() {
        // Body X-axis points down
        let acc = Vector3::new(1.0, 0.0, 0.0);
        // Body Y-axis points North
        let mag = Vector3::new(0.0, 1.0, 0.0);
        let quat = quat_from_acc_mag(&acc, &mag);
        let (roll, pitch, yaw) = quat.euler_angles();
        assert_relative_eq!(roll, core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
    }
}
