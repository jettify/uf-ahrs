use core::f32;

use crate::traits::Ahrs;
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector2, Vector3};

pub fn quat_from_acc_mag(acc: &Vector3<f32>, mag: &Vector3<f32>) -> UnitQuaternion<f32> {
    let z = acc;
    let x_temp = z.cross(&-mag);
    let x = x_temp.cross(z);
    let y = z.cross(&x);
    let mat = Matrix3::from_columns(&[x.normalize(), y.normalize(), z.normalize()]);
    UnitQuaternion::from_matrix(&mat)
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
}

impl Ahrs for Mahony {
    fn update_gyro(&mut self, gyroscope: &Vector3<f32>) -> &UnitQuaternion<f32> {
        self.update_gyro_with_dt(gyroscope, self.dt)
    }

    fn update_imu(
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

    fn update(
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
        let acc = Vector3::new(0.0, 0.0, 9.81);
        let mag = Vector3::new(-1.0, 0.0, 0.0);
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
        assert_relative_eq!(yaw, -core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
    }

    #[test]
    fn test_quat_from_acc_mag_pitch_neg_90() {
        // Body X-axis points up (-90 deg pitch)
        let acc = Vector3::new(1.0, 0.0, 0.0);
        // Body Y-axis points North
        let mag = Vector3::new(0.0, 1.0, 0.0);
        let quat = quat_from_acc_mag(&acc, &mag);
        let (roll, pitch, yaw) = quat.euler_angles();
        assert_relative_eq!(roll, -core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, -core::f32::consts::FRAC_PI_2, epsilon = 0.0001);
    }

    #[test]
    fn test_quat_from_acc_mag_compare() {
        let acc = Vector3::new(-0.23618742, -0.36316485, 9.91931498);
        let mag = Vector3::new(0.22260694, 15.82182425, -38.33789427);

        let quat = quat_from_acc_mag(&acc, &mag);
        assert_relative_eq!(quat.w, 0.72379941);
        assert_relative_eq!(quat.i, 0.02145037);
        assert_relative_eq!(quat.j, 0.00400592);
        assert_relative_eq!(quat.k, -0.68966532);

        let (roll, pitch, yaw) = quat.euler_angles();
        assert_relative_eq!(roll, 0.0255, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.03539, epsilon = 0.0001);
        assert_relative_eq!(yaw, -core::f32::consts::FRAC_PI_2, epsilon = 0.1);
    }

    #[test]
    fn test_imu_level() {
        let mut ahrs = Mahony::default();
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        for _ in 0..100 {
            ahrs.update_imu(&gyro, &accel);
        }
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_imu_pitch_45() {
        let mut ahrs = Mahony::new(0.01, 0.5, 0.0);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        // Corresponds to 45 deg pitch, assuming gravity is (0, 0, 1)
        let rad_45 = core::f32::consts::FRAC_PI_4;
        let accel = Vector3::new(-rad_45.sin(), 0.0, rad_45.cos());

        for _ in 0..1000 {
            ahrs.update_imu(&gyro, &accel);
        }

        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.01);
        assert_relative_eq!(pitch, rad_45, epsilon = 0.01);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_imu_roll_45() {
        let mut ahrs = Mahony::new(0.01, 0.5, 0.0);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        // Corresponds to 45 deg roll, assuming gravity is (0, 0, 1)
        let rad_45 = core::f32::consts::FRAC_PI_4;
        let accel = Vector3::new(0.0, rad_45.sin(), rad_45.cos());

        for _ in 0..1000 {
            ahrs.update_imu(&gyro, &accel);
        }

        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, rad_45, epsilon = 0.01);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.01);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_imu_zero_accelerometer() {
        let mut ahrs = Mahony::default();
        let initial_quat = ahrs.quaternion;

        let gyro = Vector3::new(0.1, 0.2, 0.3);
        let accel = Vector3::new(0.0, 0.0, 0.0);

        ahrs.update_imu(&gyro, &accel);

        let mut expected_ahrs = Mahony::default();
        expected_ahrs.update_gyro(&gyro);

        assert_relative_eq!(ahrs.quaternion, expected_ahrs.quaternion);
        assert_ne!(ahrs.quaternion, initial_quat);
    }

    #[test]
    fn test_update_level() {
        let mut ahrs = Mahony::new(0.01, 0.5, 0.0);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        // Magnetometer pointing North (along X axis)
        let mag = Vector3::new(1.0, 0.0, 0.0);
        for _ in 0..2000 {
            ahrs.update(&gyro, &accel, &mag);
        }
        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.01);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.01);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_update_zero_magnetometer() {
        let mut ahrs_update = Mahony::new(0.01, 0.5, 0.0);
        let mut ahrs_imu = Mahony::new(0.01, 0.5, 0.0);

        let gyro = Vector3::new(0.01, 0.02, 0.03);
        let accel = Vector3::new(0.1, 0.2, 9.8);
        let mag = Vector3::new(0.0, 0.0, 0.0);

        ahrs_update.update(&gyro, &accel, &mag);
        ahrs_imu.update_imu(&gyro, &accel);

        assert_relative_eq!(ahrs_update.quaternion, ahrs_imu.quaternion);
        assert_relative_eq!(ahrs_update.bias, ahrs_imu.bias);
    }

    #[test]
    fn test_update_zero_accelerometer() {
        let mut ahrs_update = Mahony::new(0.01, 0.5, 0.0);
        let mut ahrs_gyro = Mahony::new(0.01, 0.5, 0.0);

        let gyro = Vector3::new(0.01, 0.02, 0.03);
        let accel = Vector3::new(0.0, 0.0, 0.0);
        let mag = Vector3::new(0.1, 0.2, 0.3);

        ahrs_update.update(&gyro, &accel, &mag);
        ahrs_gyro.update_gyro(&gyro);

        assert_relative_eq!(ahrs_update.quaternion, ahrs_gyro.quaternion);
        assert_relative_eq!(ahrs_update.bias, ahrs_gyro.bias);
    }

    #[test]
    fn test_update_yaw_90() {
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        let accel = Vector3::new(0.0, 0.0, 9.81);
        let mag = Vector3::new(0.0, -1.0, 0.0);
        let init_quat = quat_from_acc_mag(&accel, &mag);
        let mut ahrs = Mahony::new_with_quaternion(0.01, 0.5, 0.001, init_quat);

        for _ in 0..10 {
            ahrs.update(&gyro, &accel, &mag);
        }

        let (roll, pitch, yaw) = ahrs.quaternion.euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.01);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.01);
        assert_relative_eq!(yaw, core::f32::consts::FRAC_PI_2, epsilon = 0.01);
    }
}
