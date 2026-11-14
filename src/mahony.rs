use core::time::Duration;

use crate::accel_prefilter::{AccelPrefilter, configure, direction, finite_orientation};
use crate::gyro_integration::integrate_gyro;
use crate::traits::Ahrs;
use crate::traits::AhrsWithDt;

use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use nalgebra::Vector2;
use nalgebra::Vector3;

/// Tuning parameters for the [`Mahony`] filter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MahonyParams {
    /// Proportional gain for orientation error correction.
    pub kp: f32,
    /// Integral gain for gyroscope bias estimation.
    pub ki: f32,
    /// Acceleration low-pass time constant in a gyro-stabilized frame.
    /// Zero (the default) disables preprocessing and retains original feedback.
    /// Based on Laidig and Seel, VQF (2023): <https://arxiv.org/abs/2203.17024>.
    pub tau_accelerometer: Duration,
}

impl Default for MahonyParams {
    fn default() -> Self {
        Self {
            kp: 0.74,
            ki: 0.0012,
            tau_accelerometer: Duration::ZERO,
        }
    }
}

/// Mahony nonlinear complementary filter for orientation estimation.
///
/// Integrates the feedback-corrected angular velocity using a quaternion
/// exponential, with angular velocity held constant during each sample interval.
#[derive(Debug)]
pub struct Mahony {
    sample_period: Duration,
    prefilter: Option<AccelPrefilter>,
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
    #[must_use]
    pub fn new(sample_period: Duration, params: MahonyParams) -> Self {
        let orientation = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0);
        Mahony::new_with_orientation(sample_period, params, orientation)
    }

    /// Creates a filter with a custom initial orientation.
    #[must_use]
    pub fn new_with_orientation(
        sample_period: Duration,
        params: MahonyParams,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        Mahony {
            sample_period,
            prefilter: configure(None, params.tau_accelerometer),
            params,
            bias: Vector3::new(0.0, 0.0, 0.0),
            quaternion: orientation,
        }
    }

    /// Returns the current tuning parameters.
    #[must_use]
    pub fn params(&self) -> MahonyParams {
        self.params
    }

    /// Replaces the tuning parameters; they apply from the next update.
    /// Changing tau clears acceleration history, preserving attitude and the
    /// independent frame when both old and new tau are positive. Gain-only
    /// changes preserve history. Zero tau drops the preprocessing state.
    ///
    /// The estimated gyroscope [`bias`](Self::bias) is kept.
    pub fn set_params(&mut self, params: MahonyParams) {
        if params.tau_accelerometer != self.params.tau_accelerometer {
            self.prefilter = configure(self.prefilter.take(), params.tau_accelerometer);
        }
        self.params = params;
    }

    fn update_gyro_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        self.quaternion = integrate_gyro(self.quaternion, gyroscope, dt);
        self.quaternion
    }

    fn update_imu_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro_with_dt_seconds(gyroscope, dt);
        };
        let v = Vector3::new(
            2.0 * (q.coords.x * q.coords.z - q.coords.w * q.coords.y),
            2.0 * (q.coords.w * q.coords.x + q.coords.y * q.coords.z),
            q.coords.w * q.coords.w - q.coords.x * q.coords.x - q.coords.y * q.coords.y
                + q.coords.z * q.coords.z,
        );

        let e = accel.cross(&v);
        let b_dot = -self.params.ki * e;
        self.bias += b_dot * dt;
        let corrected = gyroscope + e * self.params.kp - self.bias;
        self.update_gyro_with_dt_seconds(corrected, dt)
    }

    fn update_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            return self.update_gyro_with_dt_seconds(gyroscope, dt);
        };

        let Some(mag) = magnetometer.try_normalize(f32::EPSILON) else {
            return self.update_imu_with_dt_seconds(gyroscope, accelerometer, dt);
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
        self.bias += b_dot * dt;

        let corrected = gyroscope + e * self.params.kp - self.bias;
        self.update_gyro_with_dt_seconds(corrected, dt)
    }
}

impl Mahony {
    fn step(
        &mut self,
        dt: Duration,
        gyro: Vector3<f32>,
        accel: Option<Vector3<f32>>,
        mag: Option<Vector3<f32>>,
    ) -> UnitQuaternion<f32> {
        let seconds = dt.as_secs_f32();
        let Some(prefilter) = self.prefilter.as_mut() else {
            // True bypass: retain raw input, normalization and legacy update order.
            return match (accel, mag) {
                (Some(accel), Some(mag)) => self.update_with_dt_seconds(gyro, accel, mag, seconds),
                (Some(accel), None) => self.update_imu_with_dt_seconds(gyro, accel, seconds),
                (None, _) => self.update_gyro_with_dt_seconds(gyro, seconds),
            };
        };
        let previous_bias = self.bias;
        if dt.is_zero()
            || !gyro
                .iter()
                .chain(previous_bias.iter())
                .all(|v| v.is_finite())
        {
            return self.quaternion;
        }
        let fallback = integrate_gyro(self.quaternion, gyro, seconds);
        if !finite_orientation(fallback) || !prefilter.advance(gyro - previous_bias, dt) {
            return self.quaternion;
        }
        let accel = accel.and_then(|accel| prefilter.filter(accel, self.params.tau_accelerometer));
        let candidate = match (accel, mag.and_then(direction)) {
            (Some(accel), Some(mag)) => self.update_with_dt_seconds(gyro, accel, mag, seconds),
            (Some(accel), None) => self.update_imu_with_dt_seconds(gyro, accel, seconds),
            (None, _) => self.update_gyro_with_dt_seconds(gyro, seconds),
        };
        if !finite_orientation(candidate) || !self.bias.iter().all(|v| v.is_finite()) {
            self.quaternion = fallback;
            self.bias = previous_bias;
        }
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
        self.prefilter = configure(None, self.params.tau_accelerometer);
    }

    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32> {
        self.step(self.sample_period, gyroscope, None, None)
    }

    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(self.sample_period, gyroscope, Some(accelerometer), None)
    }

    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(
            self.sample_period,
            gyroscope,
            Some(accelerometer),
            Some(magnetometer),
        )
    }
}

impl AhrsWithDt for Mahony {
    fn update_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(dt, gyroscope, Some(accelerometer), Some(magnetometer))
    }

    fn update_imu_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(dt, gyroscope, Some(accelerometer), None)
    }

    fn update_gyro_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(dt, gyroscope, None, None)
    }
}

// Tests live in `tests/ahrs_test.rs` to share coverage across algorithms.
