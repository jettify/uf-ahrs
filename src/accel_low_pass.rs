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
    position: Option<Vector3<f32>>,
    // tau * derivative, so both states have acceleration units.
    velocity: Vector3<f32>,
    elapsed: Duration,
    coefficients: Option<Coefficients>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Coefficients {
    ratio: f32,
    decay: f32,
    sin: f32,
    cos: f32,
}

impl AccelLowPass {
    #[cfg(test)]
    pub(crate) fn snapshot(&self) -> (Option<Vector3<f32>>, Vector3<f32>, Duration) {
        (self.position, self.velocity, self.elapsed)
    }

    pub(crate) fn filter(
        &mut self,
        input: Vector3<f32>,
        dt: Duration,
        tau: Duration,
    ) -> Option<Vector3<f32>> {
        if !input.iter().all(|v| v.is_finite()) || dt.is_zero() {
            return None;
        }
        let mut coefficients = self.coefficients;
        let (position, velocity, elapsed) = if tau.is_zero() {
            (input, Vector3::zeros(), Duration::ZERO)
        } else if let Some(previous) = self.position {
            if self.elapsed < tau {
                let elapsed = self.elapsed.saturating_add(dt);
                let weight = dt.as_secs_f32() / elapsed.as_secs_f32();
                (
                    previous + (input - previous) * weight,
                    Vector3::zeros(),
                    elapsed,
                )
            } else {
                let ratio = dt.as_secs_f32() / tau.as_secs_f32();
                if ratio > 50.0 {
                    // The transient has decayed below f32 precision after a gap.
                    (input, Vector3::zeros(), self.elapsed)
                } else {
                    // Exact equality is intentional: memoize the same scalar functions
                    // without rounding the actual sample interval.
                    let values = match coefficients {
                        Some(values) if values.ratio == ratio => values,
                        _ => Coefficients {
                            ratio,
                            decay: libm::expf(-ratio),
                            sin: libm::sinf(ratio),
                            cos: libm::cosf(ratio),
                        },
                    };
                    coefficients = Some(values);
                    let Coefficients {
                        decay, sin, cos, ..
                    } = values;
                    let offset = previous - input;
                    let position = input + decay * ((cos + sin) * offset + sin * self.velocity);
                    let velocity = decay * ((cos - sin) * self.velocity - 2.0 * sin * offset);
                    (position, velocity, self.elapsed)
                }
            }
        } else {
            (input, Vector3::zeros(), dt)
        };
        if !position
            .iter()
            .chain(velocity.iter())
            .all(|v| v.is_finite())
        {
            return None;
        }
        self.coefficients = coefficients;
        self.position = Some(position);
        self.velocity = velocity;
        self.elapsed = elapsed;
        Some(position)
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
        assert_relative_eq!(split.position.unwrap(), expected, epsilon = 4e-7);
        assert_relative_eq!(split.velocity, single.velocity, epsilon = 4e-7);
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
        assert_eq!(filter.position, reference.position);
        assert_eq!(filter.velocity, reference.velocity);
        assert_eq!(filter.elapsed, reference.elapsed);
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
        assert_eq!(cached.unwrap().ratio, 0.005);
        f.filter(Vector3::y(), Duration::from_millis(5), tau)
            .unwrap();
        assert_eq!(f.coefficients, cached);
        f.filter(Vector3::z(), Duration::from_millis(10), tau)
            .unwrap();
        assert_eq!(f.coefficients.unwrap().ratio, 0.01);
        // Overflow in recurrence must preserve physical state and cache together.
        f.position = Some(Vector3::repeat(f32::MAX));
        let previous = f.clone();
        assert!(
            f.filter(Vector3::repeat(-f32::MAX), Duration::from_millis(7), tau)
                .is_none()
        );
        assert_eq!(f.snapshot(), previous.snapshot());
        assert_eq!(f.coefficients, previous.coefficients);
    }
}
