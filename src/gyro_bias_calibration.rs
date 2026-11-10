use core::fmt;
use core::time::Duration;

use nalgebra::Vector3;

/// Settings for stationary gyroscope bias calibration.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroBiasCalibrationParams {
    /// Minimum duration of a calibration window; also requires `min_samples`.
    pub duration: Duration,
    /// Minimum sample count before acceptance, in addition to `duration`.
    /// Must be at least two. Defaults to three.
    pub min_samples: u32,
    /// Maximum sample standard deviation on each axis, in radians/second.
    ///
    /// Zero accepts only windows with zero variance. Tune this threshold to
    /// the sensor's noise level and the sampling/filtering configuration.
    pub max_standard_deviation: f32,
    /// Maximum absolute linear drift rate on each axis, in radians/second².
    /// Zero accepts only windows with zero fitted slope.
    pub max_drift_rate: f32,
    /// Maximum peak-to-peak excursion on each axis, in radians/second.
    /// Exceeded ranges discard the window immediately. Zero requires constant readings.
    pub max_peak_to_peak: f32,
}

impl Default for GyroBiasCalibrationParams {
    fn default() -> Self {
        Self {
            duration: Duration::from_millis(1250),
            min_samples: 3,
            max_standard_deviation: 0.01,
            // Engineering starting limits for a 1.25 s startup window;
            // tune these to the sensor, filtering, and tolerated drift.
            max_drift_rate: 0.002,
            max_peak_to_peak: 0.06,
        }
    }
}

impl GyroBiasCalibrationParams {
    /// Estimates stationary thresholds from one-sided noise spectral density.
    ///
    /// `noise_density` is in rad/s/√Hz. For example, a datasheet
    /// value of 0.0028 deg/s/√Hz is approximately 4.8869e-5 rad/s/√Hz.
    /// Assumes flat white noise from DC to Nyquist (`ENBW = 1 / (2 * sample_period)`),
    /// a 1.6 PCB noise margin, and approximately uniform sampling. The limits
    /// are 12 times RMS noise for peak-to-peak, twice RMS noise for standard
    /// deviation, and eight times the independent-sample OLS slope error.
    ///
    /// These are starting estimates, not calibrated false-alarm probabilities:
    /// filtering correlates samples and can increase slope uncertainty. Tune
    /// the returned fields using stationary captures and acceptable drift.
    /// The estimator does not configure hardware filters or account for aliasing.
    ///
    /// The expected sample count is `ceil(duration / sample_period)`, with a
    /// minimum of three, matching the default `min_samples`. Collection requires
    /// both duration and `min_samples`; the requested duration is preserved.
    /// Changing `min_samples` later does not recompute the estimated thresholds.
    /// Zero noise density produces zero thresholds.
    ///
    /// # Errors
    /// Requires nonzero duration and sample period, nonnegative finite noise
    /// density, and representable thresholds.
    pub fn from_noise_density(
        duration: Duration,
        sample_period: Duration,
        noise_density: f32,
    ) -> Result<Self, GyroBiasCalibrationError> {
        if duration.is_zero() {
            return Err(GyroBiasCalibrationError::ZeroDuration);
        }
        if sample_period.is_zero() {
            return Err(GyroBiasCalibrationError::ZeroSamplePeriod);
        }
        if !noise_density.is_finite() || noise_density < 0.0 {
            return Err(GyroBiasCalibrationError::InvalidNoiseDensity);
        }
        // Configuration uses f64 to avoid intermediate overflow and retain
        // precision. Integer ceiling division avoids rounding sample counts up
        // spuriously when duration is an exact multiple of the sample period.
        let min_samples = Self::default().min_samples;
        let n = duration
            .as_nanos()
            .div_ceil(sample_period.as_nanos())
            .max(u128::from(min_samples)) as f64;
        let fs = 1.0 / sample_period.as_secs_f64();
        // Theoretical white-noise RMS is NSD * sqrt(ENBW), where ENBW = 0.5 * fs
        // is the Nyquist bandwidth. The 1.6 multiplier is an empirical PCB/system
        // noise margin (+60%) accounting for power-supply ripple, ADC quantization,
        // package stress, and minor board-level micro-vibrations above ideal lab
        // datasheet conditions.
        let sigma = 1.6 * f64::from(noise_density) * libm::sqrt(0.5 * fs);
        // Exact finite-N slope standard error: sigma / sqrt(S_tt), where
        // S_tt = N * (N² - 1) / (12 * fs²) for uniformly spaced samples.
        let sigma_slope = libm::sqrt(12.0) * sigma * (fs / n) / libm::sqrt(n - 1.0 / n);
        let params = Self {
            duration,
            min_samples,
            max_standard_deviation: (2.0 * sigma) as f32,
            max_drift_rate: (8.0 * sigma_slope) as f32,
            max_peak_to_peak: (12.0 * sigma) as f32,
        };
        params.validate()?;
        Ok(params)
    }

    fn validate(&self) -> Result<(), GyroBiasCalibrationError> {
        if self.duration.is_zero() {
            return Err(GyroBiasCalibrationError::ZeroDuration);
        }
        if self.min_samples < 2 {
            return Err(GyroBiasCalibrationError::InvalidMinSamples);
        }
        for (threshold, error) in [
            (
                self.max_standard_deviation,
                GyroBiasCalibrationError::InvalidStandardDeviation,
            ),
            (
                self.max_drift_rate,
                GyroBiasCalibrationError::InvalidDriftRate,
            ),
            (
                self.max_peak_to_peak,
                GyroBiasCalibrationError::InvalidPeakToPeak,
            ),
        ] {
            if !threshold.is_finite() || threshold < 0.0 {
                return Err(error);
            }
        }
        Ok(())
    }
}

/// Invalid settings for [`GyroBiasCalibration`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum GyroBiasCalibrationError {
    /// The calibration window duration must be nonzero.
    ZeroDuration,
    /// The minimum sample count must be at least two.
    InvalidMinSamples,
    /// The standard deviation threshold must be finite and nonnegative.
    InvalidStandardDeviation,
    /// The drift rate threshold must be finite and nonnegative.
    InvalidDriftRate,
    /// The peak-to-peak threshold must be finite and nonnegative.
    InvalidPeakToPeak,
    /// The sample period must be nonzero.
    ZeroSamplePeriod,
    /// Noise spectral density must be finite and nonnegative.
    InvalidNoiseDensity,
}

impl fmt::Display for GyroBiasCalibrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroDuration => f.write_str("calibration duration must be nonzero"),
            Self::InvalidMinSamples => f.write_str("minimum sample count must be at least two"),
            Self::InvalidStandardDeviation => {
                f.write_str("standard deviation threshold must be finite and nonnegative")
            }
            Self::InvalidDriftRate => {
                f.write_str("drift rate threshold must be finite and nonnegative")
            }
            Self::InvalidPeakToPeak => {
                f.write_str("peak-to-peak threshold must be finite and nonnegative")
            }
            Self::ZeroSamplePeriod => f.write_str("sample period must be nonzero"),
            Self::InvalidNoiseDensity => {
                f.write_str("noise spectral density must be finite and nonnegative")
            }
        }
    }
}

impl core::error::Error for GyroBiasCalibrationError {}

/// The statistical limit exceeded by a calibration window.
///
/// Each value is the largest per-axis magnitude for that statistic. Drift uses
/// the absolute slope, so values and thresholds are always nonnegative.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GyroBiasCalibrationRejectionReason {
    /// An axis exceeded the sample standard deviation limit.
    StandardDeviation {
        /// Largest per-axis sample standard deviation, in rad/s.
        value: f32,
        /// Configured maximum standard deviation, in rad/s.
        threshold: f32,
    },
    /// An axis exceeded the absolute linear drift limit.
    DriftRate {
        /// Largest absolute per-axis fitted slope, in rad/s².
        value: f32,
        /// Configured maximum absolute drift rate, in rad/s².
        threshold: f32,
    },
    /// An axis exceeded the peak-to-peak excursion limit.
    PeakToPeak {
        /// Largest per-axis peak-to-peak excursion, in rad/s.
        value: f32,
        /// Configured maximum peak-to-peak excursion, in rad/s.
        threshold: f32,
    },
}

/// Outcome of a stationary calibration update.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GyroBiasCalibrationStatus {
    /// More samples or elapsed time are required.
    Collecting,
    /// A window exceeded its standard deviation, drift, or peak-to-peak limit.
    /// Peak-to-peak excursions reject immediately; other gates run at window end.
    /// The next update starts a fresh window automatically.
    Rejected(GyroBiasCalibrationRejectionReason),
    /// A nonfinite sample or unrepresentable statistic caused the current
    /// window to be discarded. The next update starts a fresh window.
    InvalidSample,
    /// Calibration succeeded with this bias in rad/s. Held until reset.
    Complete(Vector3<f32>),
}

/// Statistics of the current or completed calibration window.
///
/// All vectors contain per-axis values in the input coordinate frame.
/// Snapshots require at least two samples and representable statistics.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct GyroBiasCalibrationStatistics {
    /// Number of collected samples.
    pub sample_count: u32,
    /// Minimum reading on each axis, in rad/s.
    pub minimum: Vector3<f32>,
    /// Maximum reading on each axis, in rad/s. Peak-to-peak is `maximum - minimum`.
    pub maximum: Vector3<f32>,
    /// Ordinary sample mean, in rad/s.
    pub mean: Vector3<f32>,
    /// Total sample standard deviation, in rad/s.
    pub standard_deviation: Vector3<f32>,
    /// Fitted linear slope, in rad/s².
    pub drift_rate: Vector3<f32>,
}

impl GyroBiasCalibrationStatistics {
    /// Returns the peak-to-peak excursion (`maximum - minimum`) on each axis, in rad/s.
    #[must_use]
    pub fn peak_to_peak(&self) -> Vector3<f32> {
        self.maximum - self.minimum
    }
}

/// Estimates a fixed gyroscope offset while the device is stationary.
///
/// Feed approximately uniformly sampled gyroscope readings in radians/second.
/// Each reading is assigned to the endpoint of `dt`, the interval since the
/// preceding sample. Prefer sensor timing over host delivery jitter. Duration
/// includes the first `dt`; measurement span excludes it. Welford's algorithm
/// computes mean, total variance, and time covariance without buffering samples.
/// Irregular intervals affect regression timestamps, not sample weights.
///
/// A window is accepted when it spans the configured duration, contains at
/// least `min_samples` samples, and every axis meets all three thresholds. Peak-to-peak
/// excursions discard the window immediately. Slope only gates acceptance;
/// the accepted bias is the sample mean, with no extrapolation.
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
    mean_time: f32,
    time_squared_deviations: f32,
    time_gyro_codeviations: Vector3<f32>,
    minimum: Vector3<f32>,
    maximum: Vector3<f32>,
    bias: Option<Vector3<f32>>,
    rejected_statistics: Option<GyroBiasCalibrationStatistics>,
}

impl Default for GyroBiasCalibration {
    fn default() -> Self {
        Self::from_validated_params(GyroBiasCalibrationParams::default())
    }
}

impl GyroBiasCalibration {
    /// Creates an empty calibration window.
    ///
    /// # Errors
    /// Returns an error for zero duration, fewer than two minimum samples, or
    /// a nonfinite/negative threshold.
    pub fn new(params: GyroBiasCalibrationParams) -> Result<Self, GyroBiasCalibrationError> {
        params.validate()?;
        Ok(Self::from_validated_params(params))
    }

    fn from_validated_params(params: GyroBiasCalibrationParams) -> Self {
        Self {
            params,
            elapsed: Duration::ZERO,
            sample_count: 0,
            mean: Vector3::zeros(),
            squared_deviations: Vector3::zeros(),
            mean_time: 0.0,
            time_squared_deviations: 0.0,
            time_gyro_codeviations: Vector3::zeros(),
            minimum: Vector3::zeros(),
            maximum: Vector3::zeros(),
            bias: None,
            rejected_statistics: None,
        }
    }

    /// Adds a sample and checks a window once its minimum duration has elapsed.
    ///
    /// Zero `dt` ignores the sample entirely. Nonfinite samples and numerical
    /// overflow discard the current window. Once complete, further calls return
    /// [`GyroBiasCalibrationStatus::Complete`] without changing the result.
    #[must_use]
    pub fn update(&mut self, dt: Duration, gyro: Vector3<f32>) -> GyroBiasCalibrationStatus {
        if let Some(bias) = self.bias {
            return GyroBiasCalibrationStatus::Complete(bias);
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

        let Some(elapsed) = self.elapsed.checked_add(dt) else {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        };
        let time = elapsed.as_secs_f32();
        if self.sample_count > 0 && time <= self.elapsed.as_secs_f32() {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        }
        let delta = gyro - self.mean;
        let delta_time = time - self.mean_time;
        let mean = self.mean + delta / sample_count as f32;
        let mean_time = self.mean_time + delta_time / sample_count as f32;
        let squared_deviations = self.squared_deviations + delta.component_mul(&(gyro - mean));
        let time_squared_deviations =
            self.time_squared_deviations + delta_time * (time - mean_time);
        let time_gyro_codeviations = self.time_gyro_codeviations + delta * (time - mean_time);
        let (minimum, maximum) = if self.sample_count == 0 {
            (gyro, gyro)
        } else {
            (
                self.minimum.zip_map(&gyro, f32::min),
                self.maximum.zip_map(&gyro, f32::max),
            )
        };
        let peak_to_peak = maximum - minimum;
        if !mean_time.is_finite()
            || !time_squared_deviations.is_finite()
            || [
                mean,
                squared_deviations,
                time_gyro_codeviations,
                peak_to_peak,
            ]
            .iter()
            .any(|vector| !vector.iter().all(|value| value.is_finite()))
        {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        }
        self.sample_count = sample_count;
        self.mean = mean;
        self.squared_deviations = squared_deviations;
        self.elapsed = elapsed;
        self.mean_time = mean_time;
        self.time_squared_deviations = time_squared_deviations;
        self.time_gyro_codeviations = time_gyro_codeviations;
        self.minimum = minimum;
        self.maximum = maximum;

        let peak_to_peak = peak_to_peak.max();
        if peak_to_peak > self.params.max_peak_to_peak {
            return self.reject(GyroBiasCalibrationRejectionReason::PeakToPeak {
                value: peak_to_peak,
                threshold: self.params.max_peak_to_peak,
            });
        }
        if self.elapsed < self.params.duration || self.sample_count < self.params.min_samples {
            return GyroBiasCalibrationStatus::Collecting;
        }
        let Some(drift_rate) = self.drift_rate() else {
            self.reset();
            return GyroBiasCalibrationStatus::InvalidSample;
        };
        let standard_deviation = self.standard_deviation().max();
        if standard_deviation > self.params.max_standard_deviation {
            return self.reject(GyroBiasCalibrationRejectionReason::StandardDeviation {
                value: standard_deviation,
                threshold: self.params.max_standard_deviation,
            });
        }
        let drift_rate = drift_rate.map(f32::abs).max();
        if drift_rate > self.params.max_drift_rate {
            return self.reject(GyroBiasCalibrationRejectionReason::DriftRate {
                value: drift_rate,
                threshold: self.params.max_drift_rate,
            });
        }
        self.bias = Some(self.mean);
        GyroBiasCalibrationStatus::Complete(self.mean)
    }

    fn reject(&mut self, reason: GyroBiasCalibrationRejectionReason) -> GyroBiasCalibrationStatus {
        let statistics = self.statistics();
        self.reset();
        self.rejected_statistics = statistics;
        GyroBiasCalibrationStatus::Rejected(reason)
    }

    /// Returns the most recently rejected window's statistics.
    ///
    /// The snapshot survives collection and completion of subsequent windows.
    /// A new rejection replaces it; explicit reset or invalid input clears it.
    /// Returns `None` if the rejected window could not produce complete statistics.
    #[must_use]
    pub fn rejected_statistics(&self) -> Option<GyroBiasCalibrationStatistics> {
        self.rejected_statistics
    }

    fn standard_deviation(&self) -> Vector3<f32> {
        // Called only for n >= 2. Clamp small negative roundoff before sqrt.
        let divisor = (self.sample_count - 1) as f32;
        self.squared_deviations
            .map(|value| libm::sqrtf((value / divisor).max(0.0)))
    }

    fn drift_rate(&self) -> Option<Vector3<f32>> {
        if self.sample_count < 2 || self.time_squared_deviations <= 0.0 {
            return None;
        }
        let slope = self.time_gyro_codeviations / self.time_squared_deviations;
        slope.iter().all(|value| value.is_finite()).then_some(slope)
    }

    /// Returns a complete snapshot of the current or completed window.
    ///
    /// Returns `None` before two samples or if the fitted slope is unrepresentable.
    /// This availability does not depend on the configured `min_samples`.
    /// Rejection and reset clear the current window; completion preserves it.
    /// See [`Self::rejected_statistics`] for the most recently rejected window.
    #[must_use]
    pub fn statistics(&self) -> Option<GyroBiasCalibrationStatistics> {
        let drift_rate = self.drift_rate()?;
        Some(GyroBiasCalibrationStatistics {
            sample_count: self.sample_count,
            minimum: self.minimum,
            maximum: self.maximum,
            mean: self.mean,
            standard_deviation: self.standard_deviation(),
            drift_rate,
        })
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
        self.mean_time = 0.0;
        self.time_squared_deviations = 0.0;
        self.time_gyro_codeviations = Vector3::zeros();
        self.minimum = Vector3::zeros();
        self.maximum = Vector3::zeros();
        self.bias = None;
        self.rejected_statistics = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::InertialCalibration;

    use approx::assert_abs_diff_eq;

    const DT: Duration = Duration::from_millis(10);

    fn calibration(duration: Duration, threshold: f32) -> GyroBiasCalibration {
        GyroBiasCalibration::new(GyroBiasCalibrationParams {
            duration,
            min_samples: 2,
            max_standard_deviation: threshold,
            max_drift_rate: 1000.0,
            max_peak_to_peak: 1000.0,
        })
        .unwrap()
    }

    #[test]
    fn test_getters_track_params_progress_and_reset() {
        let params = GyroBiasCalibrationParams {
            duration: DT * 2,
            min_samples: 2,
            max_standard_deviation: 0.005,
            ..Default::default()
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
                GyroBiasCalibrationStatus::Rejected(
                    GyroBiasCalibrationRejectionReason::StandardDeviation {
                        value: libm::sqrtf(2.0),
                        threshold: 0.01,
                    }
                )
            );
            assert_eq!(calibration.bias(), None);
            let bias = Vector3::repeat(0.03);
            assert_eq!(
                calibration.update(DT, bias),
                GyroBiasCalibrationStatus::Collecting
            );
            assert_eq!(
                calibration.update(DT, bias),
                GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
            );
            assert_abs_diff_eq!(calibration.bias().unwrap(), bias, epsilon = 1e-7);
        }
    }

    #[test]
    fn test_uses_sample_standard_deviation_and_accepts_threshold_equality() {
        let threshold = libm::sqrtf(2.0);
        for (limit, expected) in [
            (
                threshold,
                GyroBiasCalibrationStatus::Complete(Vector3::zeros()),
            ),
            (
                threshold * 0.99,
                GyroBiasCalibrationStatus::Rejected(
                    GyroBiasCalibrationRejectionReason::StandardDeviation {
                        value: threshold,
                        threshold: threshold * 0.99,
                    },
                ),
            ),
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
                    GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
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
    #[test]
    fn test_noise_density_estimator_matches_reference_and_scaling() {
        let period = Duration::from_millis(10);
        let params =
            GyroBiasCalibrationParams::from_noise_density(period * 2, period, 0.001).unwrap();
        let sigma = 1.6_f64 * 0.001 * 50.0_f64.sqrt();
        assert_eq!(params.duration, period * 2);
        assert_abs_diff_eq!(
            params.max_standard_deviation,
            (2.0 * sigma) as f32,
            epsilon = 1e-8
        );
        assert_abs_diff_eq!(
            params.max_peak_to_peak,
            (12.0 * sigma) as f32,
            epsilon = 1e-7
        );
        assert_abs_diff_eq!(
            params.max_drift_rate,
            (8.0 * sigma * 100.0 / 2.0_f64.sqrt()) as f32,
            epsilon = 1e-5
        );
        for (sample_period, density, noise_scale, slope_scale) in [
            (period, 0.002, 2.0, 2.0),
            (Duration::from_micros(2500), 0.001, 2.0, 8.0),
        ] {
            let scaled = GyroBiasCalibrationParams::from_noise_density(
                sample_period * 2,
                sample_period,
                density,
            )
            .unwrap();
            assert_abs_diff_eq!(
                scaled.max_peak_to_peak,
                params.max_peak_to_peak * noise_scale,
                epsilon = 1e-7
            );
            assert_abs_diff_eq!(
                scaled.max_standard_deviation,
                params.max_standard_deviation * noise_scale,
                epsilon = 1e-8
            );
            assert_abs_diff_eq!(
                scaled.max_drift_rate,
                params.max_drift_rate * slope_scale,
                epsilon = 1e-4
            );
        }
        let longer =
            GyroBiasCalibrationParams::from_noise_density(period * 4, period, 0.001).unwrap();
        assert_abs_diff_eq!(
            longer.max_drift_rate,
            params.max_drift_rate / 2.5_f32.sqrt(),
            epsilon = 1e-5
        );
        // Nonmultiples round sample count up, while preserving duration exactly.
        let rounded =
            GyroBiasCalibrationParams::from_noise_density(Duration::from_millis(21), period, 0.001)
                .unwrap();
        let three =
            GyroBiasCalibrationParams::from_noise_density(period * 3, period, 0.001).unwrap();
        assert_eq!(rounded.duration, Duration::from_millis(21));
        assert_eq!(rounded.max_drift_rate, three.max_drift_rate);
        let short =
            GyroBiasCalibrationParams::from_noise_density(Duration::from_nanos(1), period, 0.001)
                .unwrap();
        assert_eq!(short.max_drift_rate, params.max_drift_rate);
        let zero = GyroBiasCalibrationParams::from_noise_density(period * 2, period, 0.0).unwrap();
        assert_eq!(zero.max_standard_deviation, 0.0);
        assert_eq!(zero.max_drift_rate, 0.0);
        assert_eq!(zero.max_peak_to_peak, 0.0);
    }

    #[test]
    fn test_noise_density_estimator_rejects_invalid_inputs_and_overflow() {
        let period = Duration::from_millis(10);
        assert_eq!(
            GyroBiasCalibrationParams::from_noise_density(Duration::ZERO, period, 0.001),
            Err(GyroBiasCalibrationError::ZeroDuration)
        );
        assert_eq!(
            GyroBiasCalibrationParams::from_noise_density(period, Duration::ZERO, 0.001),
            Err(GyroBiasCalibrationError::ZeroSamplePeriod)
        );
        for density in [-1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(
                GyroBiasCalibrationParams::from_noise_density(period * 2, period, density),
                Err(GyroBiasCalibrationError::InvalidNoiseDensity)
            );
        }
        let half_second = Duration::from_millis(500);
        assert_eq!(
            GyroBiasCalibrationParams::from_noise_density(
                Duration::from_secs(1),
                half_second,
                f32::MAX
            ),
            Err(GyroBiasCalibrationError::InvalidStandardDeviation)
        );
        assert_eq!(
            GyroBiasCalibrationParams::from_noise_density(
                Duration::from_secs(50),
                half_second,
                2e37
            ),
            Err(GyroBiasCalibrationError::InvalidPeakToPeak)
        );
        assert_eq!(
            GyroBiasCalibrationParams::from_noise_density(
                Duration::from_nanos(2),
                Duration::from_nanos(1),
                1e25
            ),
            Err(GyroBiasCalibrationError::InvalidDriftRate)
        );
        assert!(
            GyroBiasCalibrationParams::from_noise_density(
                Duration::MAX,
                Duration::from_nanos(1),
                0.0
            )
            .is_ok()
        );
    }

    #[test]
    fn test_min_samples_validation() {
        for min_samples in [0, 1] {
            assert_eq!(
                GyroBiasCalibration::new(GyroBiasCalibrationParams {
                    min_samples,
                    ..Default::default()
                })
                .unwrap_err(),
                GyroBiasCalibrationError::InvalidMinSamples
            );
        }
    }

    #[test]
    fn test_acceptance_requires_duration_and_configured_sample_count() {
        for (duration, expected_samples) in [(DT, 5), (DT * 8, 8)] {
            let mut calibration = GyroBiasCalibration::new(GyroBiasCalibrationParams {
                duration,
                min_samples: 5,
                ..Default::default()
            })
            .unwrap();
            let gyro = Vector3::repeat(0.01);
            for sample_count in 1..expected_samples {
                assert_eq!(
                    calibration.update(DT, gyro),
                    GyroBiasCalibrationStatus::Collecting
                );
                if sample_count == 1 {
                    assert_eq!(calibration.statistics(), None);
                } else {
                    let stats = calibration.statistics().unwrap();
                    assert_eq!(stats.sample_count, sample_count);
                    assert_eq!(stats.standard_deviation, Vector3::zeros());
                    assert_eq!(stats.drift_rate, Vector3::zeros());
                }
            }
            assert_eq!(
                calibration.update(DT, gyro),
                GyroBiasCalibrationStatus::Complete(gyro)
            );
        }
    }

    #[test]
    fn test_statistics_require_representable_slope_even_for_rejected_windows() {
        let mut calibration = calibration(DT * 10, 1.0);
        let _status = calibration.update(DT, Vector3::zeros());
        let _status = calibration.update(DT, Vector3::zeros());
        assert!(calibration.statistics().is_some());
        calibration.time_gyro_codeviations = Vector3::repeat(f32::MAX);
        assert_eq!(calibration.statistics(), None);
        calibration.params.max_peak_to_peak = 0.01;
        assert_eq!(
            calibration.update(DT, Vector3::repeat(0.02)),
            GyroBiasCalibrationStatus::Rejected(GyroBiasCalibrationRejectionReason::PeakToPeak {
                value: 0.02,
                threshold: 0.01
            })
        );
        assert_eq!(calibration.rejected_statistics(), None);
    }

    #[test]
    fn test_default_calibrator_and_completion_payload() {
        let mut calibration = GyroBiasCalibration::default();
        assert_eq!(calibration.params(), GyroBiasCalibrationParams::default());
        let bias = Vector3::new(0.01, -0.02, 0.03);
        let duration = calibration.params().duration;
        assert_eq!(
            calibration.update(duration / 2, bias),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(duration / 2, bias),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(Duration::from_millis(1), bias),
            GyroBiasCalibrationStatus::Complete(bias)
        );
        assert_eq!(
            calibration.update(Duration::ZERO, Vector3::repeat(f32::NAN)),
            GyroBiasCalibrationStatus::Complete(bias)
        );
    }

    #[test]
    fn test_rejection_snapshot_survives_retry_and_clears_on_reset() {
        let mut calibration = calibration(DT * 2, 1.0);
        calibration.params.max_peak_to_peak = 0.01;
        let _status = calibration.update(DT, Vector3::zeros());
        assert_eq!(
            calibration.update(DT, Vector3::repeat(0.02)),
            GyroBiasCalibrationStatus::Rejected(GyroBiasCalibrationRejectionReason::PeakToPeak {
                value: 0.02,
                threshold: 0.01
            })
        );
        let rejected = calibration.rejected_statistics().unwrap();
        assert_eq!(rejected.sample_count, 2);
        assert_eq!(rejected.peak_to_peak(), Vector3::repeat(0.02));
        assert_eq!(calibration.statistics(), None);
        let _status = calibration.update(DT, Vector3::zeros());
        assert_eq!(calibration.rejected_statistics(), Some(rejected));
        let _status = calibration.update(DT, Vector3::zeros());
        assert_eq!(calibration.rejected_statistics(), Some(rejected));
        calibration.reset();
        assert_eq!(calibration.rejected_statistics(), None);
    }

    #[test]
    fn test_new_threshold_validation() {
        for invalid in [-1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for (params, expected) in [
                (
                    GyroBiasCalibrationParams {
                        max_drift_rate: invalid,
                        ..Default::default()
                    },
                    GyroBiasCalibrationError::InvalidDriftRate,
                ),
                (
                    GyroBiasCalibrationParams {
                        max_peak_to_peak: invalid,
                        ..Default::default()
                    },
                    GyroBiasCalibrationError::InvalidPeakToPeak,
                ),
            ] {
                assert_eq!(GyroBiasCalibration::new(params).unwrap_err(), expected);
            }
        }
        assert!(
            GyroBiasCalibration::new(GyroBiasCalibrationParams {
                max_standard_deviation: 0.0,
                max_drift_rate: 0.0,
                max_peak_to_peak: 0.0,
                ..Default::default()
            })
            .is_ok()
        );
    }

    #[test]
    fn test_online_statistics_match_batch_reference() {
        // Include uniform and irregular timestamps, offsets with opposite signs,
        // and deterministic nonlinear variation around the fitted line.
        for intervals in [[10, 10, 10, 10, 10], [7, 23, 11, 40, 19]] {
            let mut calibration = calibration(Duration::from_secs(1), 1.0);
            let mut times = [0.0_f64; 5];
            let mut readings = [Vector3::<f64>::zeros(); 5];
            let mut elapsed = Duration::ZERO;
            for (i, millis) in intervals.into_iter().enumerate() {
                let dt = Duration::from_millis(millis);
                elapsed += dt;
                times[i] = elapsed.as_secs_f64();
                let t = elapsed.as_secs_f32();
                let gyro = Vector3::new(0.4 + 0.03 * t, -0.2 - 0.07 * t, 0.1 + 0.02 * t * t);
                readings[i] = gyro.cast();
                assert_eq!(
                    calibration.update(dt, gyro),
                    GyroBiasCalibrationStatus::Collecting
                );
            }
            let mean_time = times.iter().sum::<f64>() / 5.0;
            let mean = readings.iter().copied().sum::<Vector3<f64>>() / 5.0;
            let mut syy = Vector3::<f64>::zeros();
            let mut sty = Vector3::<f64>::zeros();
            let mut stt = 0.0;
            for (t, y) in times.into_iter().zip(readings) {
                let dy = y - mean;
                syy += dy.component_mul(&dy);
                sty += dy * (t - mean_time);
                stt += (t - mean_time).powi(2);
            }
            let stats = calibration.statistics().unwrap();
            assert_eq!(stats.sample_count, 5);
            assert_abs_diff_eq!(stats.mean.cast::<f64>(), mean, epsilon = 1e-7);
            assert_abs_diff_eq!(
                stats.standard_deviation.cast::<f64>(),
                (syy / 4.0).map(f64::sqrt),
                epsilon = 1e-7
            );
            assert_abs_diff_eq!(stats.drift_rate.cast::<f64>(), sty / stt, epsilon = 2e-6);
            let min = readings
                .iter()
                .fold(readings[0], |a, b| a.zip_map(b, f64::min));
            let max = readings
                .iter()
                .fold(readings[0], |a, b| a.zip_map(b, f64::max));
            assert_eq!(stats.minimum.cast::<f64>(), min);
            assert_eq!(stats.maximum.cast::<f64>(), max);
            assert_abs_diff_eq!(
                stats.peak_to_peak().cast::<f64>(),
                max - min,
                epsilon = 1e-7
            );
        }
    }

    #[test]
    fn test_rejects_drift_on_each_axis_and_accepts_flat_retry() {
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut calibration = GyroBiasCalibration::new(GyroBiasCalibrationParams {
                    duration: Duration::from_secs(1),
                    max_drift_rate: 0.002,
                    ..Default::default()
                })
                .unwrap();
                for i in 1..=100 {
                    let mut gyro = Vector3::repeat(0.02);
                    gyro[axis] += sign * 0.005 * i as f32 * 0.01;
                    let status = calibration.update(DT, gyro);
                    if i == 100 {
                        let GyroBiasCalibrationStatus::Rejected(
                            GyroBiasCalibrationRejectionReason::DriftRate { value, threshold },
                        ) = status
                        else {
                            panic!("expected drift rejection, got {status:?}");
                        };
                        assert_abs_diff_eq!(value, 0.005, epsilon = 1e-6);
                        assert_eq!(threshold, 0.002);
                    } else {
                        assert_eq!(status, GyroBiasCalibrationStatus::Collecting);
                    }
                    if i == 99 {
                        let stats = calibration.statistics().unwrap();
                        assert!(stats.standard_deviation.max() < 0.01);
                        assert_abs_diff_eq!(stats.drift_rate[axis], sign * 0.005, epsilon = 1e-6);
                    }
                }
                assert_eq!(calibration.statistics(), None);
                for _ in 0..100 {
                    let _status = calibration.update(DT, Vector3::repeat(-0.01));
                }
                assert_eq!(calibration.bias(), Some(Vector3::repeat(-0.01)));
            }
        }
    }

    #[test]
    fn test_bumps_reject_immediately_on_each_axis_and_retry_cleanly() {
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut calibration = calibration(DT * 100, 1.0);
                calibration.params.max_peak_to_peak = 0.01;
                let baseline = Vector3::repeat(sign * 0.03);
                for _ in 0..5 {
                    let _status = calibration.update(DT, baseline);
                }
                let mut bump = baseline;
                bump[axis] += sign * 0.02;
                assert_eq!(
                    calibration.update(DT, bump),
                    GyroBiasCalibrationStatus::Rejected(
                        GyroBiasCalibrationRejectionReason::PeakToPeak {
                            value: (bump[axis] - baseline[axis]).abs(),
                            threshold: 0.01,
                        }
                    )
                );
                assert_eq!(calibration.elapsed(), Duration::ZERO);
                assert_eq!(calibration.statistics(), None);
                for _ in 0..100 {
                    let _status = calibration.update(DT, baseline);
                }
                let stats = calibration.statistics().unwrap();
                assert_eq!(calibration.bias(), Some(baseline));
                assert_eq!(stats.minimum, baseline);
                assert_eq!(stats.maximum, baseline);
            }
        }
    }

    #[test]
    fn test_range_and_slope_threshold_equality() {
        // Binary-exact timestamps and readings make both equalities exact.
        for (slope_limit, range_limit, expected) in [
            (
                1.0,
                0.25,
                GyroBiasCalibrationStatus::Complete(Vector3::repeat(0.375)),
            ),
            (
                0.99,
                0.25,
                GyroBiasCalibrationStatus::Rejected(
                    GyroBiasCalibrationRejectionReason::DriftRate {
                        value: 1.0,
                        threshold: 0.99,
                    },
                ),
            ),
            (
                1.0,
                0.24,
                GyroBiasCalibrationStatus::Rejected(
                    GyroBiasCalibrationRejectionReason::PeakToPeak {
                        value: 0.25,
                        threshold: 0.24,
                    },
                ),
            ),
        ] {
            let dt = Duration::from_millis(250);
            let mut calibration = calibration(dt * 2, 1.0);
            calibration.params.max_drift_rate = slope_limit;
            calibration.params.max_peak_to_peak = range_limit;
            let _status = calibration.update(dt, Vector3::repeat(0.25));
            assert_eq!(calibration.update(dt, Vector3::repeat(0.5)), expected);
        }
    }

    #[test]
    fn test_symmetric_noise_statistics_lifecycle() {
        let mut calibration = calibration(DT * 4, 0.01);
        calibration.params.max_drift_rate = 1e-6;
        assert_eq!(calibration.statistics(), None);
        let _status = calibration.update(DT, Vector3::repeat(-0.001));
        assert_eq!(calibration.statistics(), None);
        for value in [0.001, 0.001, -0.001] {
            let _status = calibration.update(DT, Vector3::repeat(value));
        }
        assert!(calibration.is_complete());
        let completed = calibration.statistics();
        let _status = calibration.update(DT, Vector3::repeat(f32::NAN));
        assert_eq!(calibration.statistics(), completed);
        calibration.reset();
        assert_eq!(calibration.statistics(), None);
        let _status = calibration.update(DT, Vector3::repeat(1.0));
        let _status = calibration.update(DT, Vector3::repeat(f32::NAN));
        assert_eq!(calibration.statistics(), None);
    }

    #[test]
    fn test_timing_and_count_failures_discard_window() {
        let mut calibration = calibration(Duration::MAX, 1.0);
        let gyro = Vector3::zeros();
        let _status = calibration.update(Duration::from_secs(1 << 24), gyro);
        assert_eq!(
            calibration.update(Duration::from_nanos(1), gyro),
            GyroBiasCalibrationStatus::InvalidSample
        );
        assert_eq!(calibration.statistics(), None);
        calibration.elapsed = Duration::MAX;
        assert_eq!(
            calibration.update(DT, gyro),
            GyroBiasCalibrationStatus::InvalidSample
        );
        calibration.sample_count = u32::MAX;
        assert_eq!(
            calibration.update(DT, gyro),
            GyroBiasCalibrationStatus::InvalidSample
        );
        let _status = calibration.update(DT, gyro);
        calibration.params.duration = DT * 2;
        calibration.time_squared_deviations = -1.0;
        assert_eq!(
            calibration.update(DT, gyro),
            GyroBiasCalibrationStatus::InvalidSample
        );
        assert_eq!(calibration.statistics(), None);
        let _status = calibration.update(DT, gyro);
        calibration.time_gyro_codeviations = Vector3::repeat(f32::MAX);
        // Finite moments can still yield a nonfinite slope when divided.
        assert_eq!(
            calibration.update(DT, gyro),
            GyroBiasCalibrationStatus::InvalidSample
        );
        assert_eq!(calibration.statistics(), None);
    }
    #[test]
    fn test_nanosecond_window_uses_positive_time_variance_without_absolute_epsilon() {
        let dt = Duration::from_nanos(1);
        let mut calibration = GyroBiasCalibration::new(GyroBiasCalibrationParams {
            duration: dt * 2,
            min_samples: 2,
            max_standard_deviation: 0.0,
            max_peak_to_peak: 0.0,
            max_drift_rate: 0.0,
        })
        .unwrap();
        let gyro = Vector3::repeat(0.02);
        assert_eq!(
            calibration.update(dt, gyro),
            GyroBiasCalibrationStatus::Collecting
        );
        assert_eq!(
            calibration.update(dt, gyro),
            GyroBiasCalibrationStatus::Complete(calibration.bias().unwrap())
        );
        assert_eq!(
            calibration.statistics().unwrap().drift_rate,
            Vector3::zeros()
        );
    }
}
