use core::fmt;
use core::time::Duration;

use nalgebra::Vector3;

/// Settings for stationary gyroscope bias calibration.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroBiasCalibrationParams {
    /// Minimum duration of a calibration window, with at least two samples.
    pub duration: Duration,
    /// Maximum sample standard deviation on each axis, in radians/second.
    ///
    /// Zero accepts only windows with zero variance. Tune this threshold to
    /// the sensor's noise level and the sampling/filtering configuration.
    pub max_standard_deviation: f32,
}

impl Default for GyroBiasCalibrationParams {
    fn default() -> Self {
        Self {
            duration: Duration::from_millis(1250),
            max_standard_deviation: 0.01,
        }
    }
}

/// Invalid settings for [`GyroBiasCalibration`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GyroBiasCalibrationError {
    /// The calibration window duration must be nonzero.
    ZeroDuration,
    /// The standard deviation threshold must be finite and nonnegative.
    InvalidStandardDeviation,
}

impl fmt::Display for GyroBiasCalibrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroDuration => f.write_str("calibration duration must be nonzero"),
            Self::InvalidStandardDeviation => {
                f.write_str("standard deviation threshold must be finite and nonnegative")
            }
        }
    }
}

impl core::error::Error for GyroBiasCalibrationError {}

/// Outcome of a stationary calibration update.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GyroBiasCalibrationStatus {
    /// More samples or elapsed time are required.
    Collecting,
    /// A completed window exceeded the variance threshold and was discarded.
    /// The next update starts a fresh window automatically.
    MotionDetected,
    /// A nonfinite sample or unrepresentable statistic caused the current
    /// window to be discarded. The next update starts a fresh window.
    InvalidSample,
    /// Calibration succeeded. The result is held until reset.
    Complete,
}

/// Estimates a fixed gyroscope offset while the device is stationary.
///
/// Feed approximately uniformly sampled gyroscope readings in radians/second.
/// Each `dt` is the time represented by that sample. The ordinary sample mean
/// and sample standard deviation are computed using Welford's algorithm,
/// without buffering samples. Timing jitter changes the window duration but
/// does not weight individual samples.
///
/// A window is accepted when it spans the configured duration, contains at
/// least two samples, and every axis meets the standard deviation threshold.
/// Rejected windows restart automatically. A successful estimate remains fixed
/// until [`Self::reset`]; this type does not track bias online.
///
/// The caller must ensure physical rest: constant rotation can have low
/// variance and be mistaken for bias. Collect and subtract the offset in the
/// same units and coordinate frame. If readings already have an offset removed,
/// this estimate is a residual correction rather than a replacement offset.
#[derive(Debug, Clone)]
pub struct GyroBiasCalibration {
    params: GyroBiasCalibrationParams,
    elapsed: Duration,
    sample_count: u32,
    mean: Vector3<f32>,
    squared_deviations: Vector3<f32>,
    bias: Option<Vector3<f32>>,
}

impl GyroBiasCalibration {
    /// Creates an empty calibration window.
    ///
    /// # Errors
    /// Returns an error for zero duration or a nonfinite/negative threshold.
    pub fn new(params: GyroBiasCalibrationParams) -> Result<Self, GyroBiasCalibrationError> {
        if params.duration.is_zero() {
            return Err(GyroBiasCalibrationError::ZeroDuration);
        }
        if !params.max_standard_deviation.is_finite() || params.max_standard_deviation < 0.0 {
            return Err(GyroBiasCalibrationError::InvalidStandardDeviation);
        }
        Ok(Self {
            params,
            elapsed: Duration::ZERO,
            sample_count: 0,
            mean: Vector3::zeros(),
            squared_deviations: Vector3::zeros(),
            bias: None,
        })
    }

    /// Adds a sample and checks a window once its minimum duration has elapsed.
    ///
    /// Zero `dt` ignores the sample entirely. Nonfinite samples and numerical
    /// overflow discard the current window. Once complete, further calls return
    /// [`GyroBiasCalibrationStatus::Complete`] without changing the result.
    #[must_use]
    pub fn update(&mut self, dt: Duration, gyro: Vector3<f32>) -> GyroBiasCalibrationStatus {
        if self.bias.is_some() {
            return GyroBiasCalibrationStatus::Complete;
        }
        if dt.is_zero() {
            return GyroBiasCalibrationStatus::Collecting;
        }
        if !gyro.iter().all(|value| value.is_finite()) {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        }
        let Some(sample_count) = self.sample_count.checked_add(1) else {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        };

        let delta = gyro - self.mean;
        let mean = self.mean + delta / sample_count as f32;
        let squared_deviations = self.squared_deviations + delta.component_mul(&(gyro - mean));
        if !mean.iter().all(|value| value.is_finite())
            || !squared_deviations.iter().all(|value| value.is_finite())
        {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        }
        self.sample_count = sample_count;
        self.mean = mean;
        self.squared_deviations = squared_deviations;
        self.elapsed = self.elapsed.saturating_add(dt);

        if self.elapsed < self.params.duration || self.sample_count < 2 {
            return GyroBiasCalibrationStatus::Collecting;
        }
        // Use sample variance (n - 1). Clamp small negative roundoff before sqrt.
        let divisor = (self.sample_count - 1) as f32;
        if self.squared_deviations.iter().any(|value| {
            libm::sqrtf((*value / divisor).max(0.0)) > self.params.max_standard_deviation
        }) {
            self.reset();
            return GyroBiasCalibrationStatus::MotionDetected;
        }
        self.bias = Some(self.mean);
        GyroBiasCalibrationStatus::Complete
    }

    /// Returns the calibration settings.
    #[must_use]
    pub fn params(&self) -> GyroBiasCalibrationParams {
        self.params
    }

    /// Returns the accumulated duration of the current or completed window.
    #[must_use]
    pub fn elapsed(&self) -> Duration {
        self.elapsed
    }

    /// Returns `true` once a stationary window has been accepted.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.bias.is_some()
    }

    /// Returns the accepted offset in radians/second, or `None` while collecting.
    ///
    /// For readings in the same frame, use this as
    /// [`crate::InertialCalibration::offset`] before sensitivity and misalignment
    /// correction.
    #[must_use]
    pub fn bias(&self) -> Option<Vector3<f32>> {
        self.bias
    }

    /// Clears the estimate and collected samples while preserving settings.
    pub fn reset(&mut self) {
        self.elapsed = Duration::ZERO;
        self.sample_count = 0;
        self.mean = Vector3::zeros();
        self.squared_deviations = Vector3::zeros();
        self.bias = None;
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::InertialCalibration;

    const DT: Duration = Duration::from_millis(10);

    fn calibration(duration: Duration, threshold: f32) -> GyroBiasCalibration {
        GyroBiasCalibration::new(GyroBiasCalibrationParams {
            duration,
            max_standard_deviation: threshold,
        })
        .unwrap()
    }

    #[test]
    fn test_getters_track_params_progress_and_reset() {
        let params = GyroBiasCalibrationParams {
            duration: DT * 2,
            max_standard_deviation: 0.005,
        };
        let mut calibration = GyroBiasCalibration::new(params).unwrap();
        assert_eq!(calibration.params(), params);
        assert_eq!(calibration.elapsed(), Duration::ZERO);
        assert!(!calibration.is_complete());

        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(calibration.elapsed(), DT);
        assert!(!calibration.is_complete());

        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Complete
        );
        assert_eq!(calibration.elapsed(), DT * 2);
        assert!(calibration.is_complete());

        calibration.reset();
        assert_eq!(calibration.params(), params);
        assert_eq!(calibration.elapsed(), Duration::ZERO);
        assert!(!calibration.is_complete());
    }

    #[test]
    fn test_recovers_offset_and_applies_inertial_calibration() {
        let mut calibration = calibration(Duration::from_millis(40), 0.001);
        let bias = Vector3::new(0.02, -0.03, 0.01);
        for _ in 0..3 {
            assert_eq!(
                calibration.update(DT, bias),
                GyroBiasCalibrationStatus::Collecting
            );
            assert_eq!(calibration.bias(), None);
        }
        assert_eq!(
            calibration.update(DT, bias),
            GyroBiasCalibrationStatus::Complete
        );
        let offset = calibration.bias().unwrap();
        assert_abs_diff_eq!(offset, bias, epsilon = 1e-7);
        let model = InertialCalibration {
            offset,
            ..Default::default()
        };
        let rotation = Vector3::new(0.4, -0.2, 0.1);
        assert_abs_diff_eq!(model.calibrate(rotation + bias), rotation, epsilon = 1e-7);
    }

    #[test]
    fn test_accepts_noise_and_estimates_mean() {
        let mut calibration = calibration(Duration::from_millis(40), 0.002);
        let bias = Vector3::new(0.02, -0.03, 0.01);
        for noise in [-0.001, 0.001, -0.001, 0.001] {
            let _status = calibration.update(DT, bias + Vector3::repeat(noise));
        }
        assert_abs_diff_eq!(calibration.bias().unwrap(), bias, epsilon = 1e-7);
    }

    #[test]
    fn test_rejects_motion_on_each_axis_and_retries_without_old_samples() {
        for axis in 0..3 {
            let mut calibration = calibration(Duration::from_millis(20), 0.01);
            let mut moving = Vector3::zeros();
            moving[axis] = 1.0;
            assert_eq!(
                calibration.update(DT, moving),
                GyroBiasCalibrationStatus::Collecting
            );
            assert_eq!(
                calibration.update(DT, -moving),
                GyroBiasCalibrationStatus::MotionDetected
            );
            assert_eq!(calibration.bias(), None);
            let bias = Vector3::repeat(0.03);
            assert_eq!(
                calibration.update(DT, bias),
                GyroBiasCalibrationStatus::Collecting
            );
            assert_eq!(
                calibration.update(DT, bias),
                GyroBiasCalibrationStatus::Complete
            );
            assert_abs_diff_eq!(calibration.bias().unwrap(), bias, epsilon = 1e-7);
        }
    }

    #[test]
    fn test_uses_sample_standard_deviation_and_accepts_threshold_equality() {
        let threshold = libm::sqrtf(2.0);
        for (limit, expected) in [
            (threshold, GyroBiasCalibrationStatus::Complete),
            (threshold * 0.99, GyroBiasCalibrationStatus::MotionDetected),
        ] {
            let mut calibration = calibration(DT * 2, limit);
            let _status = calibration.update(DT, Vector3::repeat(-1.0));
            assert_eq!(calibration.update(DT, Vector3::repeat(1.0)), expected);
        }
    }

    #[test]
    fn test_variable_timing_reaches_window_boundary() {
        let mut calibration = calibration(Duration::from_millis(30), 0.0);
        let gyro = Vector3::repeat(0.02);
        assert_eq!(
            calibration.update(Duration::from_millis(11), gyro),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(Duration::from_millis(18), gyro),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(Duration::from_millis(1), gyro),
            GyroBiasCalibrationStatus::Complete
        );
        assert_eq!(calibration.bias(), Some(gyro));
    }

    #[test]
    fn test_requires_two_samples_even_after_window_duration() {
        let mut calibration = calibration(DT, 0.0);
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(calibration.bias(), None);
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Complete
        );
    }

    #[test]
    fn test_zero_dt_ignores_sample_and_preserves_window() {
        let mut calibration = calibration(DT * 2, 0.0);
        let gyro = Vector3::repeat(0.01);
        let _status = calibration.update(DT, gyro);
        assert_eq!(
            calibration.update(Duration::ZERO, Vector3::repeat(f32::NAN)),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(DT, gyro),
            GyroBiasCalibrationStatus::Complete
        );
        assert_eq!(calibration.bias(), Some(gyro));
    }

    #[test]
    fn test_nonfinite_sample_discards_window() {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for axis in 0..3 {
                let mut calibration = calibration(DT * 2, 0.0);
                let _status = calibration.update(DT, Vector3::repeat(1.0));
                let mut sample = Vector3::zeros();
                sample[axis] = invalid;
                assert_eq!(
                    calibration.update(DT, sample),
                    GyroBiasCalibrationStatus::InvalidSample
                );
                assert_eq!(calibration.bias(), None);
                assert_eq!(
                    calibration.update(DT, Vector3::zeros()),
                    GyroBiasCalibrationStatus::Collecting
                );
                assert_eq!(
                    calibration.update(DT, Vector3::zeros()),
                    GyroBiasCalibrationStatus::Complete
                );
                assert_eq!(calibration.bias(), Some(Vector3::zeros()));
            }
        }
    }

    #[test]
    fn test_statistics_overflow_discards_window() {
        let mut calibration = calibration(DT * 2, 0.01);
        let _status = calibration.update(DT, Vector3::repeat(f32::MAX));
        assert_eq!(
            calibration.update(DT, Vector3::repeat(-f32::MAX)),
            GyroBiasCalibrationStatus::InvalidSample
        );
        assert_eq!(calibration.bias(), None);
        let _status = calibration.update(DT, Vector3::zeros());
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Complete
        );
    }

    #[test]
    fn test_completed_result_is_held_until_reset() {
        let mut calibration = calibration(DT * 2, 0.0);
        let gyro = Vector3::repeat(0.01);
        let _status = calibration.update(DT, gyro);
        let _status = calibration.update(DT, gyro);
        assert_eq!(
            calibration.update(DT, Vector3::repeat(f32::NAN)),
            GyroBiasCalibrationStatus::Complete
        );
        assert_eq!(calibration.bias(), Some(gyro));
        calibration.reset();
        assert_eq!(calibration.bias(), None);
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Complete
        );
        assert_eq!(calibration.bias(), Some(Vector3::zeros()));
    }

    #[test]
    fn test_reset_discards_partial_window() {
        let mut calibration = calibration(DT * 2, 0.0);
        let _status = calibration.update(DT, Vector3::repeat(1.0));
        calibration.reset();
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(DT, Vector3::zeros()),
            GyroBiasCalibrationStatus::Complete
        );
    }

    #[test]
    fn test_rejects_invalid_parameters() {
        assert_eq!(
            GyroBiasCalibration::new(GyroBiasCalibrationParams {
                duration: Duration::ZERO,
                ..Default::default()
            })
            .unwrap_err(),
            GyroBiasCalibrationError::ZeroDuration
        );
        for threshold in [-0.01, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(
                GyroBiasCalibration::new(GyroBiasCalibrationParams {
                    max_standard_deviation: threshold,
                    ..Default::default()
                })
                .unwrap_err(),
                GyroBiasCalibrationError::InvalidStandardDeviation
            );
        }
        assert!(GyroBiasCalibration::new(GyroBiasCalibrationParams::default()).is_ok());
    }
}
