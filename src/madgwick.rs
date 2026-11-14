use core::time::Duration;

use crate::accel_prefilter::{AccelPrefilter, configure, direction, finite_orientation};
use crate::gyro_integration::integrate_gyro;
use crate::traits::Ahrs;
use crate::traits::AhrsWithDt;

use nalgebra::Matrix4;
use nalgebra::Matrix6;
use nalgebra::Quaternion;
use nalgebra::UnitQuaternion;
use nalgebra::Vector2;
use nalgebra::Vector3;
use nalgebra::Vector4;
use nalgebra::Vector6;

/// Tuning parameters for the [`Madgwick`] filter.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MadgwickParams {
    /// Gradient descent gain.
    ///
    /// Larger values increase correction aggressiveness and noise sensitivity.
    pub beta: f32,
    /// Acceleration low-pass time constant in a gyro-stabilized frame.
    /// Zero (the default) disables preprocessing and retains original feedback.
    /// Based on Laidig and Seel, VQF (2023): <https://arxiv.org/abs/2203.17024>.
    pub tau_accelerometer: Duration,
}

impl Default for MadgwickParams {
    fn default() -> Self {
        Self {
            beta: 0.05,
            tau_accelerometer: Duration::ZERO,
        }
    }
}

/// Madgwick gradient-descent AHRS filter.
///
/// Uses a quaternion exponential for gyroscope propagation. The gradient
/// correction is evaluated at the start of the sample and applied with a
/// first-order step, followed by quaternion normalization.
#[derive(Debug)]
pub struct Madgwick {
    sample_period: Duration,
    prefilter: Option<AccelPrefilter>,
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
    /// Creates a filter with identity orientation.
    ///
    /// `sample_period` controls gyroscope integration step size.
    #[must_use]
    pub fn new(sample_period: Duration, params: MadgwickParams) -> Self {
        Madgwick::new_with_orientation(
            sample_period,
            params,
            UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
        )
    }

    /// Creates a filter with a custom initial orientation.
    #[must_use]
    pub fn new_with_orientation(
        sample_period: Duration,
        params: MadgwickParams,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        Madgwick {
            sample_period,
            prefilter: configure(None, params.tau_accelerometer),
            params,
            quaternion: orientation,
        }
    }

    /// Returns the current tuning parameters.
    #[must_use]
    pub fn params(&self) -> MadgwickParams {
        self.params
    }

    /// Replaces the tuning parameters; they apply from the next update.
    /// Changing tau clears acceleration history, preserving attitude and the
    /// independent frame when both old and new tau are positive. Gain-only
    /// changes preserve history. Zero tau drops the preprocessing state.
    ///
    /// Useful for gain scheduling, e.g. reducing `beta` while the
    /// accelerometer is dominated by non-gravitational acceleration.
    pub fn set_params(&mut self, params: MadgwickParams) {
        if params.tau_accelerometer != self.params.tau_accelerometer {
            self.prefilter = configure(self.prefilter.take(), params.tau_accelerometer);
        }
        self.params = params;
    }
}

impl Madgwick {
    fn update_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        let predicted = integrate_gyro(self.quaternion, gyroscope, dt);
        self.correct_marg(accelerometer, magnetometer, dt, predicted)
    }

    fn correct_marg(
        &mut self,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
        dt: f32,
        predicted: UnitQuaternion<f32>,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let half: f32 = 0.5;
        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            self.quaternion = predicted;
            return self.quaternion;
        };

        let Some(mag) = magnetometer.try_normalize(f32::EPSILON) else {
            return self.correct_imu(accelerometer, dt, predicted);
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
            self.quaternion = predicted;
            return self.quaternion;
        };
        let correction = Quaternion::new(step.x, step.y, step.z, step.w) * self.params.beta;
        self.quaternion = UnitQuaternion::from_quaternion(predicted.into_inner() - correction * dt);
        self.quaternion
    }

    fn update_imu_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        let predicted = integrate_gyro(self.quaternion, gyroscope, dt);
        self.correct_imu(accelerometer, dt, predicted)
    }

    fn correct_imu(
        &mut self,
        accelerometer: Vector3<f32>,
        dt: f32,
        predicted: UnitQuaternion<f32>,
    ) -> UnitQuaternion<f32> {
        let q = self.quaternion.as_ref();

        let half: f32 = 0.5;

        let Some(accel) = accelerometer.try_normalize(f32::EPSILON) else {
            self.quaternion = predicted;
            return self.quaternion;
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
            self.quaternion = predicted;
            return self.quaternion;
        };
        let correction = Quaternion::new(step.x, step.y, step.z, step.w) * self.params.beta;
        self.quaternion = UnitQuaternion::from_quaternion(predicted.into_inner() - correction * dt);
        self.quaternion
    }

    fn update_gyro_with_dt_seconds(
        &mut self,
        gyroscope: Vector3<f32>,
        dt: f32,
    ) -> UnitQuaternion<f32> {
        self.quaternion = integrate_gyro(self.quaternion, gyroscope, dt);
        self.quaternion
    }
}

impl Madgwick {
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
        if dt.is_zero() || !gyro.iter().all(|v| v.is_finite()) {
            return self.quaternion;
        }
        let fallback = integrate_gyro(self.quaternion, gyro, seconds);
        if !finite_orientation(fallback) || !prefilter.advance(gyro, dt) {
            return self.quaternion;
        }
        let accel = accel.and_then(|accel| prefilter.filter(accel, self.params.tau_accelerometer));
        let candidate = match (accel, mag.and_then(direction)) {
            (Some(accel), Some(mag)) => self.correct_marg(accel, mag, seconds, fallback),
            (Some(accel), None) => self.correct_imu(accel, seconds, fallback),
            (None, _) => {
                self.quaternion = fallback;
                fallback
            }
        };
        if !finite_orientation(candidate) {
            self.quaternion = fallback;
        }
        self.quaternion
    }
}

impl Ahrs for Madgwick {
    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) {
        self.quaternion = quat;
        self.prefilter = configure(None, self.params.tau_accelerometer);
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
        self.step(
            self.sample_period,
            gyroscope,
            Some(accelerometer),
            Some(magnetometer),
        )
    }

    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.step(self.sample_period, gyroscope, Some(accelerometer), None)
    }

    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32> {
        self.step(self.sample_period, gyroscope, None, None)
    }
}

impl AhrsWithDt for Madgwick {
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

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn test_finite_gyro_with_overflowing_norm_preserves_all_state() {
        let dt = Duration::from_millis(5);
        let mut filter = Madgwick::new(
            dt,
            MadgwickParams {
                tau_accelerometer: Duration::from_secs(2),
                ..Default::default()
            },
        );
        filter.update_imu(Vector3::new(0.1, 0.2, 0.3), Vector3::z());
        let orientation = filter.orientation();
        let history = std::format!("{:?}", filter.prefilter);
        let gyro = Vector3::repeat(1e25);
        assert!((gyro * dt.as_secs_f32()).iter().all(|v| v.is_finite()));
        assert_eq!(filter.update(gyro, Vector3::x(), Vector3::z()), orientation);
        assert_eq!(std::format!("{:?}", filter.prefilter), history);
    }

    #[test]
    fn test_feedback_overflow_reuses_valid_prediction() {
        let dt = Duration::from_millis(5);
        let mut filter = Madgwick::new(
            dt,
            MadgwickParams {
                beta: f32::MAX,
                tau_accelerometer: Duration::from_secs(2),
            },
        );
        let gyro = Vector3::new(0.1, -0.2, 0.3);
        let expected = integrate_gyro(filter.orientation(), gyro, dt.as_secs_f32());
        assert_eq!(filter.update(gyro, Vector3::x(), Vector3::z()), expected);
    }
}
