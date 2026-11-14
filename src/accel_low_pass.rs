use core::time::Duration;

use nalgebra::Vector3;

/// Mean-initialized second-order Butterworth low-pass in a gyro-stabilized frame.
/// Acceleration preprocessing follows Laidig and Seel, VQF (2023):
/// <https://arxiv.org/abs/2203.17024>.
/// The continuous poles are (-1 +/- i)/tau. Exact zero-order-hold propagation
/// keeps the filter stable with variable sample periods without changing its
/// memory when the period changes.
#[derive(Debug, Clone, Default)]
pub(crate) struct AccelLowPass {
    output: Option<Vector3<f32>>,
    // tau * derivative, so both states have acceleration units.
    scaled_derivative: Vector3<f32>,
    warmup_elapsed: Duration,
    coefficients: Option<ButterworthCoefficients>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ButterworthCoefficients {
    dt_over_tau: f32,
    decay: f32,
    sin: f32,
    cos: f32,
}

fn is_finite_vector(vector: Vector3<f32>) -> bool {
    vector.iter().all(|v| v.is_finite())
}

impl AccelLowPass {
    #[cfg(test)]
    pub(crate) fn snapshot(&self) -> (Option<Vector3<f32>>, Vector3<f32>, Duration) {
        (self.output, self.scaled_derivative, self.warmup_elapsed)
    }

    fn cached_coefficients(&self, dt_over_tau: f32) -> ButterworthCoefficients {
        // Exact equality is intentional: memoize the same scalar functions
        // without rounding the actual sample interval.
        match self.coefficients {
            Some(cached) if cached.dt_over_tau == dt_over_tau => cached,
            _ => ButterworthCoefficients {
                dt_over_tau,
                decay: libm::expf(-dt_over_tau),
                sin: libm::sinf(dt_over_tau),
                cos: libm::cosf(dt_over_tau),
            },
        }
    }

    fn propagate_butterworth(
        &self,
        previous_output: Vector3<f32>,
        input: Vector3<f32>,
        dt_over_tau: f32,
    ) -> (Vector3<f32>, Vector3<f32>, Option<ButterworthCoefficients>) {
        if dt_over_tau > 50.0 {
            // The transient has decayed below f32 precision after a gap.
            return (input, Vector3::zeros(), self.coefficients);
        }

        let coeffs = self.cached_coefficients(dt_over_tau);
        let ButterworthCoefficients {
            decay, sin, cos, ..
        } = coeffs;
        let offset = previous_output - input;
        let output = input + decay * ((cos + sin) * offset + sin * self.scaled_derivative);
        let scaled_derivative = decay * ((cos - sin) * self.scaled_derivative - 2.0 * sin * offset);
        (output, scaled_derivative, Some(coeffs))
    }

    pub(crate) fn filter(
        &mut self,
        input: Vector3<f32>,
        dt: Duration,
        tau: Duration,
    ) -> Option<Vector3<f32>> {
        if !is_finite_vector(input) || dt.is_zero() {
            return None;
        }

        let (output, scaled_derivative, warmup_elapsed, coefficients) = if tau.is_zero() {
            (input, Vector3::zeros(), Duration::ZERO, self.coefficients)
        } else if let Some(previous_output) = self.output {
            if self.warmup_elapsed < tau {
                let warmup_elapsed = self.warmup_elapsed.saturating_add(dt);
                let weight = dt.as_secs_f32() / warmup_elapsed.as_secs_f32();
                let output = previous_output + (input - previous_output) * weight;
                (output, Vector3::zeros(), warmup_elapsed, self.coefficients)
            } else {
                let dt_over_tau = dt.as_secs_f32() / tau.as_secs_f32();
                let (output, scaled_derivative, coefficients) =
                    self.propagate_butterworth(previous_output, input, dt_over_tau);
                (output, scaled_derivative, self.warmup_elapsed, coefficients)
            }
        } else {
            (input, Vector3::zeros(), dt, self.coefficients)
        };

        if !is_finite_vector(output) || !is_finite_vector(scaled_derivative) {
            return None;
        }

        self.coefficients = coefficients;
        self.output = Some(output);
        self.scaled_derivative = scaled_derivative;
        self.warmup_elapsed = warmup_elapsed;
        Some(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_variable_period_propagation_matches_single_interval() {
        let tau = Duration::from_secs(1);
        let mut single = AccelLowPass::default();
        single.filter(Vector3::z(), tau, tau).unwrap();
        let mut split = single.clone();
        let input = Vector3::new(0.5, -0.2, 1.0);
        let expected = single
            .filter(input, Duration::from_millis(100), tau)
            .unwrap();
        for dt in [10, 25, 15, 50] {
            split.filter(input, Duration::from_millis(dt), tau).unwrap();
        }
        assert_relative_eq!(split.output.unwrap(), expected, epsilon = 4e-7);
        assert_relative_eq!(
            split.scaled_derivative,
            single.scaled_derivative,
            epsilon = 4e-7
        );
    }

    #[test]
    fn test_initialization_is_time_weighted_and_zero_tau_passes_through() {
        let tau = Duration::from_secs(1);
        let mut filter = AccelLowPass::default();
        filter
            .filter(Vector3::x(), Duration::from_millis(250), tau)
            .unwrap();
        let average = filter
            .filter(Vector3::y(), Duration::from_millis(750), tau)
            .unwrap();
        assert_relative_eq!(average, Vector3::new(0.25, 0.75, 0.0), epsilon = 1e-7);
        assert_eq!(
            filter
                .filter(Vector3::z(), Duration::from_millis(1), Duration::ZERO)
                .unwrap(),
            Vector3::z()
        );
    }

    #[test]
    fn test_invalid_measurements_do_not_poison_history() {
        let tau = Duration::from_secs(1);
        let mut filter = AccelLowPass::default();
        filter.filter(Vector3::z(), tau, tau).unwrap();
        let reference = filter.clone();
        assert!(filter.filter(Vector3::repeat(f32::NAN), tau, tau).is_none());
        assert!(filter.filter(Vector3::x(), Duration::ZERO, tau).is_none());
        assert_eq!(filter.output, reference.output);
        assert_eq!(filter.scaled_derivative, reference.scaled_derivative);
        assert_eq!(filter.warmup_elapsed, reference.warmup_elapsed);
        assert_eq!(filter.coefficients, reference.coefficients);
        assert_eq!(filter.snapshot(), reference.snapshot());
        assert_relative_eq!(
            filter.filter(Vector3::z(), tau, tau).unwrap(),
            Vector3::z(),
            epsilon = 1e-7
        );
    }
    #[test]
    fn test_cache_refresh_and_failed_candidate_are_atomic() {
        let tau = Duration::from_secs(1);
        let mut f = AccelLowPass::default();
        f.filter(Vector3::z(), tau, tau).unwrap();
        assert!(f.coefficients.is_none());
        f.filter(Vector3::x(), Duration::from_millis(5), tau)
            .unwrap();
        let cached = f.coefficients;
        assert_eq!(cached.unwrap().dt_over_tau, 0.005);
        f.filter(Vector3::y(), Duration::from_millis(5), tau)
            .unwrap();
        assert_eq!(f.coefficients, cached);
        f.filter(Vector3::z(), Duration::from_millis(10), tau)
            .unwrap();
        assert_eq!(f.coefficients.unwrap().dt_over_tau, 0.01);
        // Overflow in recurrence must preserve physical state and cache together.
        f.output = Some(Vector3::repeat(f32::MAX));
        let previous = f.clone();
        assert!(
            f.filter(Vector3::repeat(-f32::MAX), Duration::from_millis(7), tau)
                .is_none()
        );
        assert_eq!(f.snapshot(), previous.snapshot());
        assert_eq!(f.coefficients, previous.coefficients);
    }
}
