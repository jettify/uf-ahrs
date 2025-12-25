use core::f32::consts::{PI, SQRT_2};
use core::time::Duration;

use idsp::iir::Biquad;
use libm::tanf;
use nalgebra::SMatrix;

#[derive(Debug, Clone, PartialEq)]
pub struct MeanInitializedLowPassFilter<const N: usize, const M: usize> {
    sample_count: u32,
    initialized: bool,
    biquad: Biquad<f32>,
    tau: Duration,
    sampling_rate: Duration,
    state: [SMatrix<f32, N, M>; 2],
    pub last_output: SMatrix<f32, N, M>,
}

impl<const N: usize, const M: usize> MeanInitializedLowPassFilter<N, M> {
    pub fn new(tau: Duration, sampling_rate: Duration) -> Self {
        let (b, a) = second_order_butterworth(tau, sampling_rate);
        Self::with_coeffs(tau, sampling_rate, b, a)
    }

    pub fn with_coeffs(tau: Duration, sampling_rate: Duration, b: [f32; 3], a: [f32; 2]) -> Self {
        let biquad = Biquad::from([b[0], b[1], b[2], a[0], a[1]]);

        Self {
            sample_count: 0,
            initialized: false,
            biquad,
            tau,
            sampling_rate,
            last_output: SMatrix::from_element(f32::NAN),
            state: [SMatrix::zeros(), SMatrix::zeros()],
        }
    }

    #[inline]
    #[must_use]
    pub fn filter(&mut self, x: SMatrix<f32, N, M>) -> SMatrix<f32, N, M> {
        if !self.initialized {
            return self.filter_arithmetic_mean(x);
        }

        let &[b0, b1, b2, a1, a2] = self.biquad.ba();
        let y = self.state[0] + x * b0;
        self.state[0] = self.state[1] + x * b1 - y * a1;
        self.state[1] = x * b2 - y * a2;
        self.last_output = y;
        y
    }

    #[inline]
    #[allow(clippy::cast_precision_loss)]
    fn filter_arithmetic_mean(&mut self, x: SMatrix<f32, N, M>) -> SMatrix<f32, N, M> {
        self.sample_count += 1;
        self.state[1] += x;

        let elapsed = self.sample_count as f32 * self.sampling_rate.as_secs_f32();
        if elapsed >= self.tau.as_secs_f32() {
            let x0 = self.state[1] / self.sample_count as f32;

            // Set initial state based on the mean value
            let b = [
                self.biquad.ba()[0],
                self.biquad.ba()[1],
                self.biquad.ba()[2],
            ];
            let a = [self.biquad.ba()[3], self.biquad.ba()[4]];
            self.state = Self::filter_initial_state(x0, b, a);

            self.initialized = true;
            self.last_output = x0;
            return x0;
        }

        // return the arithmetic mean of all samples up until this point.
        self.state[1] / self.sample_count as f32
    }

    #[inline]
    fn filter_initial_state(
        initial_state: SMatrix<f32, N, M>,
        b: [f32; 3],
        a: [f32; 2],
    ) -> [SMatrix<f32, N, M>; 2] {
        let state0 = initial_state * (1.0 - b[0]);
        let state1 = initial_state * (b[2] - a[1]);
        [state0, state1]
    }
}

pub fn second_order_butterworth(tau: Duration, sampling_time: Duration) -> ([f32; 3], [f32; 2]) {
    let tau = tau.as_secs_f32();
    let sampling_time = sampling_time.as_secs_f32();

    let fc = SQRT_2 / (2.0 * PI * tau);
    let c = tanf(PI * fc * sampling_time);
    let d = c * c + SQRT_2 * c + 1.0;

    let b0 = c * c / d;
    let b1 = 2.0 * b0;
    let b2 = b0;

    let a1 = (2.0 * (c * c - 1.0)) / d;
    let a2 = (1.0 - SQRT_2 * c + c * c) / d;

    ([b0, b1, b2], [a1, a2])
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use approx::assert_relative_eq;
    use core::time::Duration;
    use nalgebra::SMatrix;

    #[test]
    fn test_second_order_butterworth() {
        let tau = Duration::from_secs_f32(0.1);
        let sampling_time = Duration::from_secs_f32(0.01);
        let (b, a) = second_order_butterworth(tau, sampling_time);

        assert_relative_eq!(b[0], 0.004539257, epsilon = 1e-6);
        assert_relative_eq!(b[1], 0.009078514, epsilon = 1e-6);
        assert_relative_eq!(b[2], 0.004539257, epsilon = 1e-6);
        assert_relative_eq!(a[0], -1.8005754, epsilon = 1e-5);
        assert_relative_eq!(a[1], 0.8187324, epsilon = 1e-5);
    }

    #[test]
    fn test_filter_initialization() {
        let tau = Duration::from_secs_f32(0.1);
        let sampling_rate = Duration::from_secs_f32(0.05);
        let mut filter = MeanInitializedLowPassFilter::<1, 1>::new(tau, sampling_rate);

        assert!(!filter.initialized);
        assert_eq!(filter.sample_count, 0);

        let input = SMatrix::<f32, 1, 1>::from_element(1.0);
        let output = filter.filter(input);

        assert!(!filter.initialized);
        assert_eq!(filter.sample_count, 1);
        assert_relative_eq!(output[(0, 0)], 1.0, epsilon = 1e-6);

        let input2 = SMatrix::<f32, 1, 1>::from_element(3.0);
        let output2 = filter.filter(input2);

        assert!(filter.initialized);
        assert_eq!(filter.sample_count, 2);
        assert_relative_eq!(output2[(0, 0)], 2.0, epsilon = 1e-6);
        assert_relative_eq!(filter.last_output[(0, 0)], 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_filter_after_initialization() {
        let tau = Duration::from_secs_f32(0.1);
        let sampling_rate = Duration::from_secs_f32(0.01);
        let mut filter = MeanInitializedLowPassFilter::<1, 1>::new(tau, sampling_rate);

        // Initialize filter
        for i in 0..11 {
            let input = SMatrix::<f32, 1, 1>::from_element(i as f32);
            let _ = filter.filter(input);
        }
        assert!(filter.initialized);
        assert_relative_eq!(filter.last_output[(0, 0)], 5.0, epsilon = 1e-6);

        // Test with constant input
        let input = SMatrix::<f32, 1, 1>::from_element(1.0);
        let mut prev_output = filter.filter(input);
        for _ in 0..100 {
            let output = filter.filter(input);
            // The output should converge to the input value
            assert!((output[(0, 0)] - prev_output[(0, 0)]).abs() < 1.0);
            prev_output = output;
        }
        assert_relative_eq!(prev_output[(0, 0)], 1.0, epsilon = 1e-3);
    }

    #[test]
    fn test_filter_initial_state() {
        let initial_state = SMatrix::<f32, 1, 1>::from_element(2.0);
        let b = [0.1, 0.2, 0.3];
        let a = [-1.5, 0.6];
        let state = MeanInitializedLowPassFilter::<1, 1>::filter_initial_state(initial_state, b, a);

        assert_relative_eq!(state[0][(0, 0)], 2.0 * (1.0 - 0.1), epsilon = 1e-6);
        assert_relative_eq!(state[1][(0, 0)], 2.0 * (0.3 - 0.6), epsilon = 1e-6);
    }
}
