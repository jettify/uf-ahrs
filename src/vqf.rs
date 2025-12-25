use crate::traits::Ahrs;
use core::f32;
use core::time::Duration;
use libm::{cosf, powf, sinf, sqrtf};

use nalgebra::{Matrix3, Quaternion, SVector, SimdPartialOrd, UnitQuaternion, Vector2, Vector3};

use crate::mean_init_lfp::{MeanInitializedLowPassFilter, second_order_butterworth};

const BIAS_SCALE: f32 = 100.0;

#[derive(Debug, Clone, PartialEq)]
pub struct VqfState {
    pub gyroscope: UnitQuaternion<f32>,
    pub accelerometer: UnitQuaternion<f32>,
    pub delta: f32,
    rest: Option<Duration>,
    pub is_magnetometer_disturbed: bool,
    pub magnetometer_k_init: f32,
    pub last_magnetometer_disagreement_angle: f32,
    pub last_magnetometer_correction_angular_rate: f32,
    pub accelerometer_low_pass: MeanInitializedLowPassFilter<3, 1>,
    pub rest_gyro_low_pass: MeanInitializedLowPassFilter<3, 1>,
    pub rest_accel_low_pass: MeanInitializedLowPassFilter<3, 1>,
    pub motion_bias_estimate_rotation_low_pass: MeanInitializedLowPassFilter<3, 3>,
    pub motion_bias_estimate_low_pass: MeanInitializedLowPassFilter<2, 1>,
    pub magnetometer_norm_dip_low_pass: MeanInitializedLowPassFilter<2, 1>,
    pub bias: SVector<f32, 3>,
    pub bias_p: Matrix3<f32>,
    pub magnetometer_reference_norm: f32,
    pub magnetometer_reference_dip: f32,
    pub magnetometer_undisturbed_duration: Duration,
    pub magnetometer_rejection_duration: Duration,
    pub magnetometer_candidate_norm: f32,
    pub magnetometer_candidate_dip: f32,
    pub magnetometer_candidate_duration: Duration,
}

impl VqfState {
    #[must_use]
    pub fn new(
        coefficients: &VqfCoefficients,
        params: &VqfParameters,
        gyro_rate: Duration,
        accel_rate: Duration,
        mag_rate: Duration,
    ) -> Self {
        Self {
            gyroscope: UnitQuaternion::identity(),
            accelerometer: UnitQuaternion::identity(),
            delta: 0.0,
            rest: None,
            is_magnetometer_disturbed: true,
            magnetometer_k_init: 1.0,
            last_magnetometer_disagreement_angle: 0.0,
            last_magnetometer_correction_angular_rate: 0.0,
            accelerometer_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.tau_accelerometer,
                accel_rate,
                coefficients.accel_b,
                coefficients.accel_a,
            ),
            rest_gyro_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.rest_filter_tau,
                gyro_rate,
                coefficients.rest_gyro_b,
                coefficients.rest_gyro_a,
            ),
            rest_accel_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.rest_filter_tau,
                accel_rate,
                coefficients.rest_accel_b,
                coefficients.rest_accel_a,
            ),
            motion_bias_estimate_rotation_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.tau_accelerometer,
                accel_rate,
                coefficients.accel_b,
                coefficients.accel_a,
            ),
            motion_bias_estimate_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.tau_accelerometer,
                accel_rate,
                coefficients.accel_b,
                coefficients.accel_a,
            ),
            magnetometer_norm_dip_low_pass: MeanInitializedLowPassFilter::with_coeffs(
                params.magnetometer_current_tau,
                mag_rate,
                coefficients.magnetometer_norm_dip_b,
                coefficients.magnetometer_norm_dip_a,
            ),
            bias: Vector3::zeros(),
            bias_p: coefficients.bias.p0 * Matrix3::identity(),
            magnetometer_reference_norm: 0.0,
            magnetometer_reference_dip: 0.0,
            magnetometer_undisturbed_duration: Duration::ZERO,
            magnetometer_rejection_duration: params.magnetometer_max_rejection_time,
            magnetometer_candidate_norm: -1.0,
            magnetometer_candidate_dip: 0.0,
            magnetometer_candidate_duration: Duration::ZERO,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct VqfParameters {
    pub tau_accelerometer: Duration,
    pub tau_magnetometer: Duration,
    pub do_magnetometer_update: bool,
    pub do_bias_estimation: bool,
    pub do_rest_bias_estimation: bool,
    pub bias_sigma_initial: f32,
    pub bias_forgetting_time: Duration,
    pub bias_clip: f32,
    pub bias_sigma_motion: f32,
    pub bias_vertical_forgetting_factor: f32,
    pub bias_sigma_rest: f32,
    pub rest_min_duration: Duration,
    pub rest_filter_tau: Duration,
    pub rest_threshold_gyro: f32,
    pub rest_threshold_accel: f32,
    pub magnetometer_current_tau: Duration,
    pub magnetometer_reference_tau: Duration,
    pub magnetometer_norm_threshold: f32,
    pub magnetometer_dip_threshold: f32,
    pub magnetometer_new_time: Duration,
    pub magnetometer_new_first_time: Duration,
    pub magnetometer_new_min_gyro: f32,
    pub magnetometer_min_undisturbed_time: Duration,
    pub magnetometer_max_rejection_time: Duration,
    pub magnetometer_rejection_factor: f32,
}

impl Default for VqfParameters {
    fn default() -> Self {
        Self {
            tau_accelerometer: Duration::from_secs_f32(3.0),
            tau_magnetometer: Duration::from_secs_f32(9.0),
            do_magnetometer_update: true,
            do_bias_estimation: true,
            do_rest_bias_estimation: true,
            bias_sigma_initial: 0.5,
            bias_forgetting_time: Duration::from_secs(100),
            bias_clip: 2.0,
            bias_sigma_motion: 0.1,
            bias_vertical_forgetting_factor: 0.0001,
            bias_sigma_rest: 0.03,
            rest_min_duration: Duration::from_secs_f32(1.5),
            rest_filter_tau: Duration::from_secs_f32(0.5),
            rest_threshold_gyro: 2.0,
            rest_threshold_accel: 0.5,
            magnetometer_current_tau: Duration::from_secs_f32(0.05),
            magnetometer_reference_tau: Duration::from_secs_f32(20.0),
            magnetometer_norm_threshold: 0.1,
            magnetometer_dip_threshold: 10.0,
            magnetometer_new_time: Duration::from_secs(20),
            magnetometer_new_first_time: Duration::from_secs(5),
            magnetometer_new_min_gyro: 20.0,
            magnetometer_min_undisturbed_time: Duration::from_secs_f32(0.5),
            magnetometer_max_rejection_time: Duration::from_secs(60),
            magnetometer_rejection_factor: 2.0,
        }
    }
}

/// Coefficients for gyroscope bias estimation.
#[derive(Debug, Clone, PartialEq)]
pub struct VqfBiasCoefficients {
    p0: f32,
    v: f32,
    motion_w: f32,
    vertical_w: f32,
    rest_w: f32,
}

impl VqfBiasCoefficients {
    #[must_use]
    fn new(accelerometer_rate: Duration, params: &VqfParameters) -> Self {
        // line 17 of Algorithm 2, the initial variance of the bias
        let p0 = powf(params.bias_sigma_initial * BIAS_SCALE, 2.0);

        // line 18 of Algorithm 2
        // System noise increases the variance from 0 to (0.1 °/s)^2 in
        // `bias_forgetting_time` duration
        let v = powf(0.1 * 100_f32, 2.0) * accelerometer_rate.as_secs_f32()
            / params.bias_forgetting_time.as_secs_f32();

        // line 19 of Algorithm 2
        let p_motion = powf(params.bias_sigma_motion * BIAS_SCALE, 2.0);
        let motion_w = p_motion * p_motion / v + p_motion;
        let vertical_w = motion_w / params.bias_vertical_forgetting_factor.max(1e-10);

        // line 20 of Algorithm 2
        let p_rest = powf(params.bias_sigma_rest * BIAS_SCALE, 2.0);
        let rest_w = p_rest * p_rest / v + p_rest;

        Self {
            p0,
            v,
            motion_w,
            vertical_w,
            rest_w,
        }
    }
}

/// Coefficients used for in a [`Vqf`] system.
#[derive(Debug, Clone, PartialEq)]
pub struct VqfCoefficients {
    accel_b: [f32; 3],
    accel_a: [f32; 2],
    rest_gyro_b: [f32; 3],
    rest_gyro_a: [f32; 2],
    rest_accel_b: [f32; 3],
    rest_accel_a: [f32; 2],
    magnetometer_norm_dip_b: [f32; 3],
    magnetometer_norm_dip_a: [f32; 2],
    magnetometer_k: f32,
    magnetometer_reference_k: f32,
    bias: VqfBiasCoefficients,
}

pub struct Vqf {
    pub coefficients: VqfCoefficients,
    parameters: VqfParameters,
    pub state: VqfState,
    gyro_rate: Duration,
    accel_rate: Duration,
    mag_rate: Duration,
}

fn gain_from_tau(tau: Duration, t_s: Duration) -> f32 {
    if tau.is_zero() {
        1.0
    } else if tau.as_secs_f32() < 0.0 {
        0.0
    } else {
        1.0 - libm::expf(-t_s.as_secs_f32() / tau.as_secs_f32())
    }
}

impl Vqf {
    #[must_use]
    pub fn new_with_sensor_rates(
        gyro_sampling_rate: Duration,
        accel_sampling_rate: Duration,
        mag_sampling_rate: Duration,
        params: VqfParameters,
    ) -> Self {
        let (accel_coefficients_b, accel_coefficients_a) =
            second_order_butterworth(params.tau_accelerometer, accel_sampling_rate);

        let (rest_gyro_coefficients_b, rest_gyro_coefficients_a) =
            second_order_butterworth(params.rest_filter_tau, gyro_sampling_rate);

        let (rest_accel_coefficients_b, rest_accel_coefficients_a) =
            second_order_butterworth(params.rest_filter_tau, accel_sampling_rate);

        let (mag_norm_dip_b, mag_norm_dip_a) =
            second_order_butterworth(params.magnetometer_current_tau, mag_sampling_rate);

        let magnetometer_k = gain_from_tau(params.tau_magnetometer, mag_sampling_rate);
        let magnetometer_reference_k =
            gain_from_tau(params.magnetometer_reference_tau, mag_sampling_rate);

        let bias = VqfBiasCoefficients::new(accel_sampling_rate, &params);

        let coefficients = VqfCoefficients {
            accel_b: accel_coefficients_b,
            accel_a: accel_coefficients_a,
            rest_gyro_b: rest_gyro_coefficients_b,
            rest_gyro_a: rest_gyro_coefficients_a,
            rest_accel_b: rest_accel_coefficients_b,
            rest_accel_a: rest_accel_coefficients_a,
            magnetometer_norm_dip_b: mag_norm_dip_b,
            magnetometer_norm_dip_a: mag_norm_dip_a,
            magnetometer_k,
            magnetometer_reference_k,
            bias,
        };

        Self {
            state: VqfState::new(
                &coefficients,
                &params,
                gyro_sampling_rate,
                accel_sampling_rate,
                mag_sampling_rate,
            ),
            coefficients,
            parameters: params,
            gyro_rate: gyro_sampling_rate,
            accel_rate: accel_sampling_rate,
            mag_rate: mag_sampling_rate,
        }
    }

    #[must_use]
    pub fn new(sample_period: Duration, params: VqfParameters) -> Self {
        Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params)
    }

    #[must_use]
    pub fn new_with_orientation(
        sample_period: Duration,
        params: VqfParameters,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        Vqf::new_with_sensor_rates_and_orientation(
            sample_period,
            sample_period,
            sample_period,
            params,
            orientation,
        )
    }

    #[must_use]
    pub fn new_with_sensor_rates_and_orientation(
        gyro_sampling_rate: Duration,
        accel_sampling_rate: Duration,
        mag_sampling_rate: Duration,
        params: VqfParameters,
        orientation: UnitQuaternion<f32>,
    ) -> Self {
        let mut vqf = Vqf::new_with_sensor_rates(
            gyro_sampling_rate,
            accel_sampling_rate,
            mag_sampling_rate,
            params,
        );
        vqf.state.gyroscope = orientation;
        vqf
    }

    #[inline]
    #[must_use]
    pub fn is_rest_phase(&self) -> bool {
        self.state
            .rest
            .is_some_and(|rest_duration| rest_duration >= self.parameters.rest_min_duration)
    }

    #[must_use]
    pub fn orientation(&self) -> UnitQuaternion<f32> {
        self.state.accelerometer * self.state.gyroscope
    }

    pub fn quaternion(&self) -> UnitQuaternion<f32> {
        self.state.accelerometer * self.state.gyroscope
    }
    pub fn quaternion2(&self) -> UnitQuaternion<f32> {
        self.state.gyroscope
    }

    #[must_use]
    pub fn heading_orientation(&self) -> UnitQuaternion<f32> {
        let heading_quat = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.state.delta);
        heading_quat * self.orientation()
    }

    pub fn update2(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>) {
        self.gyroscope_update(gyro);
        self.accelerometer_update(accel);
    }

    pub fn magnetometer_update(&mut self, mag: Vector3<f32>) {
        if mag == Vector3::zeros() {
            return;
        }

        let mag_earth = self.orientation() * mag;

        if self.parameters.do_magnetometer_update {
            let mut mag_norm_dip = Vector2::new(
                mag_earth.norm(),
                -libm::asinf(mag_earth.z / mag_earth.norm()),
            );
            if self.parameters.magnetometer_current_tau > Duration::ZERO {
                mag_norm_dip = self
                    .state
                    .magnetometer_norm_dip_low_pass
                    .filter(mag_norm_dip);
            }

            let mag_norm_th = self.parameters.magnetometer_norm_threshold
                * self.state.magnetometer_reference_norm;
            let mag_dip_th = self.parameters.magnetometer_dip_threshold.to_radians();

            if (mag_norm_dip.x - self.state.magnetometer_reference_norm).abs() < mag_norm_th
                && (mag_norm_dip.y - self.state.magnetometer_reference_dip).abs() < mag_dip_th
            {
                self.state.magnetometer_undisturbed_duration += self.mag_rate;
                if self.state.magnetometer_undisturbed_duration
                    >= self.parameters.magnetometer_min_undisturbed_time
                {
                    self.state.is_magnetometer_disturbed = false;
                    self.state.magnetometer_reference_norm +=
                        self.coefficients.magnetometer_reference_k
                            * (mag_norm_dip.x - self.state.magnetometer_reference_norm);
                    self.state.magnetometer_reference_dip +=
                        self.coefficients.magnetometer_reference_k
                            * (mag_norm_dip.y - self.state.magnetometer_reference_dip);
                }
            } else {
                self.state.magnetometer_undisturbed_duration = Duration::ZERO;
                self.state.is_magnetometer_disturbed = true;
            }

            let mag_candidate_norm_th = self.parameters.magnetometer_norm_threshold
                * self.state.magnetometer_candidate_norm;
            if (mag_norm_dip.x - self.state.magnetometer_candidate_norm).abs()
                < mag_candidate_norm_th
                && (mag_norm_dip.y - self.state.magnetometer_candidate_dip).abs() < mag_dip_th
            {
                if self.state.rest_gyro_low_pass.last_output.norm()
                    >= self.parameters.magnetometer_new_min_gyro.to_radians()
                {
                    self.state.magnetometer_candidate_duration += self.mag_rate;
                }
                self.state.magnetometer_candidate_norm +=
                    self.coefficients.magnetometer_reference_k
                        * (mag_norm_dip.x - self.state.magnetometer_candidate_norm);
                self.state.magnetometer_candidate_dip += self.coefficients.magnetometer_reference_k
                    * (mag_norm_dip.y - self.state.magnetometer_candidate_dip);

                if self.state.is_magnetometer_disturbed
                    && (self.state.magnetometer_candidate_duration
                        >= self.parameters.magnetometer_new_time
                        || (self.state.magnetometer_reference_norm == 0.0
                            && self.state.magnetometer_candidate_duration
                                >= self.parameters.magnetometer_new_first_time))
                {
                    self.state.magnetometer_reference_norm = self.state.magnetometer_candidate_norm;
                    self.state.magnetometer_reference_dip = self.state.magnetometer_candidate_dip;
                    self.state.is_magnetometer_disturbed = false;
                    self.state.magnetometer_undisturbed_duration =
                        self.parameters.magnetometer_min_undisturbed_time;
                }
            } else {
                self.state.magnetometer_candidate_duration = Duration::ZERO;
                self.state.magnetometer_candidate_norm = mag_norm_dip.x;
                self.state.magnetometer_candidate_dip = mag_norm_dip.y;
            }
        }

        self.state.last_magnetometer_disagreement_angle =
            libm::atan2f(-mag_earth.y, mag_earth.x) - self.state.delta;
        //libm::atan2f(mag_earth.x, mag_earth.y) - self.state.delta;
        if self.state.last_magnetometer_disagreement_angle > core::f32::consts::PI {
            self.state.last_magnetometer_disagreement_angle -= 2.0 * core::f32::consts::PI;
        } else if self.state.last_magnetometer_disagreement_angle < -core::f32::consts::PI {
            self.state.last_magnetometer_disagreement_angle += 2.0 * core::f32::consts::PI;
        }

        let mut k = self.coefficients.magnetometer_k;
        if self.parameters.do_magnetometer_update {
            if self.state.is_magnetometer_disturbed {
                if self.state.magnetometer_rejection_duration
                    <= self.parameters.magnetometer_max_rejection_time
                {
                    self.state.magnetometer_rejection_duration += self.mag_rate;
                    k = 0.0;
                } else {
                    k /= self.parameters.magnetometer_rejection_factor;
                }
            } else {
                let reduction =
                    self.parameters.magnetometer_rejection_factor * self.mag_rate.as_secs_f32();
                self.state.magnetometer_rejection_duration = Duration::from_secs_f32(
                    (self.state.magnetometer_rejection_duration.as_secs_f32() - reduction).max(0.0),
                );
            }
        }

        if self.state.magnetometer_k_init != 0.0 {
            if k < self.state.magnetometer_k_init {
                k = self.state.magnetometer_k_init;
            }
            self.state.magnetometer_k_init /= self.state.magnetometer_k_init + 1.0;
            if self.state.magnetometer_k_init * self.parameters.tau_magnetometer.as_secs_f32()
                < self.mag_rate.as_secs_f32()
            {
                self.state.magnetometer_k_init = 0.0;
            }
        }

        self.state.delta += k * self.state.last_magnetometer_disagreement_angle;
        self.state.last_magnetometer_correction_angular_rate =
            k * self.state.last_magnetometer_disagreement_angle / self.mag_rate.as_secs_f32();

        if self.state.delta > core::f32::consts::PI {
            self.state.delta -= 2.0 * core::f32::consts::PI;
        } else if self.state.delta < -core::f32::consts::PI {
            self.state.delta += 2.0 * core::f32::consts::PI;
        }
    }

    pub fn gyroscope_update(&mut self, gyro: Vector3<f32>) {
        if self.parameters.do_rest_bias_estimation {
            self.gyro_rest_detection(gyro);
        }

        let unbiased_gyro = gyro - self.state.bias;
        let gyro_norm = unbiased_gyro.norm();

        // if the gyro norm is approaching zero, do not perform update
        if gyro_norm <= f32::EPSILON {
            return;
        }

        // predict the new orientation (eq. 3)
        let half_angle = (self.gyro_rate.as_secs_f32() * gyro_norm) / 2.;

        let cosine = cosf(half_angle);
        let sine = sinf(half_angle) / gyro_norm;
        let gyro_step = Quaternion::new(cosine, sine * gyro.x, sine * gyro.y, sine * gyro.z);

        self.state.gyroscope =
            UnitQuaternion::from_quaternion(self.state.gyroscope.quaternion() * gyro_step);
    }

    #[inline]
    fn gyro_rest_detection(&mut self, gyro: Vector3<f32>) {
        let gyro_lp = self.state.rest_gyro_low_pass.filter(gyro);
        let deviation = gyro - gyro_lp;
        let squared_deviation = deviation.dot(&deviation);

        let bias_clip = self.parameters.bias_clip.to_radians();
        if squared_deviation
            >= self.parameters.rest_threshold_gyro * self.parameters.rest_threshold_gyro
            || gyro_lp.abs().max() > bias_clip
        {
            self.state.rest = None;
        }
    }

    pub fn accelerometer_update(&mut self, accel: Vector3<f32>) {
        if accel == Vector3::zeros() {
            return;
        }

        if self.parameters.do_rest_bias_estimation {
            self.accel_rest_detection(accel);
        }

        let acc_earth = self.state.gyroscope * accel;
        let accel_low_pass = self.state.accelerometer_low_pass.filter(acc_earth);
        let acc_earth = (self.state.accelerometer * accel_low_pass).normalize();
        let q_w = sqrtf((acc_earth.z + 1.0) / 2.0); // equation 4
        // equation 5
        let inclination_correction = if q_w > f32::EPSILON {
            let q_x = acc_earth.y / (2.0 * q_w);
            let q_y = -acc_earth.x / (2.0 * q_w);
            let q_z = 0.0;

            let inclination_correction = Quaternion::new(q_w, q_x, q_y, q_z);
            UnitQuaternion::from_quaternion(inclination_correction)
        } else {
            UnitQuaternion::from_quaternion(Quaternion::new(0., 1., 0., 0.))
        };

        self.state.accelerometer = inclination_correction * self.state.accelerometer;
        self.bias_estimation_step(acc_earth);
    }

    #[inline]
    fn accel_rest_detection(&mut self, acc: Vector3<f32>) {
        let accel_lp = self.state.rest_accel_low_pass.filter(acc);
        let deviation = acc - accel_lp;
        let squared_deviation = deviation.dot(&deviation);

        if squared_deviation
            >= self.parameters.rest_threshold_accel * self.parameters.rest_threshold_accel
        {
            self.state.rest = None;
        } else {
            self.state.rest = Some(self.state.rest.unwrap_or_default() + self.accel_rate);
        }
    }

    fn bias_estimation_step(&mut self, acc_earth: Vector3<f32>) {
        let bias_clip = self.parameters.bias_clip.to_radians();
        let mut bias = self.state.bias;

        let accel_gyro_quat = self.orientation();
        // R from line 23
        let r = accel_gyro_quat.to_rotation_matrix();

        // R b_hat from line 25, only x and y components are used
        // as the z component is the bias of the magnetometer
        let rb_hat = Vector2::new(
            r[(0, 0)] * bias[0] + r[(0, 1)] * bias[1] + r[(0, 2)] * bias[2],
            r[(1, 0)] * bias[0] + r[(1, 1)] * bias[1] + r[(1, 2)] * bias[2],
        );

        // line 24 from Algorithm 2
        let r = self
            .state
            .motion_bias_estimate_rotation_low_pass
            .filter(*r.matrix());

        // line 25 from Algorithm 2
        let bias_lp = self.state.motion_bias_estimate_low_pass.filter(rb_hat);

        // update the bias estimate for the respective Kalman filter
        let (e, r, w) = if self.is_rest_phase() && self.parameters.do_rest_bias_estimation {
            (
                self.state.rest_gyro_low_pass.last_output - bias,
                Matrix3::identity(),
                Some(Vector3::from_element(self.coefficients.bias.rest_w)),
            )
        } else if self.parameters.do_bias_estimation {
            let acc_rate = self.accel_rate.as_secs_f32();
            (
                Vector3::new(
                    -acc_earth.y / acc_rate + bias_lp.x
                        - r[(0, 0)] * bias[0]
                        - r[(0, 1)] * bias[1]
                        - r[(0, 2)] * bias[2],
                    acc_earth.x / acc_rate + bias_lp.y
                        - r[(1, 0)] * bias[0]
                        - r[(1, 1)] * bias[1]
                        - r[(1, 2)] * bias[2],
                    0.0,
                ),
                r,
                Some(Vector3::new(
                    self.coefficients.bias.motion_w,
                    self.coefficients.bias.motion_w,
                    self.coefficients.bias.vertical_w,
                )),
            )
        } else {
            (Vector3::zeros(), r, None)
        };

        // Kalman filter update
        // step 1: P = P + V (also increase covariance if there is no measurement
        // update!)
        if self.state.bias_p[(0, 0)] < self.coefficients.bias.p0 {
            self.state.bias_p[(0, 0)] += self.coefficients.bias.v;
        }

        if self.state.bias_p[(1, 1)] < self.coefficients.bias.p0 {
            self.state.bias_p[(1, 1)] += self.coefficients.bias.v;
        }

        if self.state.bias_p[(2, 2)] < self.coefficients.bias.p0 {
            self.state.bias_p[(2, 2)] += self.coefficients.bias.v;
        }

        if let Some(w) = w {
            // clip disagreement to -2..2 degrees
            let bias_clip_vector = Vector3::repeat(bias_clip);
            let e = e.simd_clamp(-bias_clip_vector, bias_clip_vector);

            // step 2: K = P  R^T (W + R P R^T)^-1 (line 36)
            let w_diag = Matrix3::from_diagonal(&w);
            let r_transpose = r.transpose();
            let r_p = r * self.state.bias_p;

            let w_r_p_r_t = w_diag + (r_p * r_transpose);
            let w_r_p_r_t_inv = w_r_p_r_t
                .try_inverse()
                .expect("(W + R P R^T) isn't a square matrix");
            let k = self.state.bias_p * r_transpose * w_r_p_r_t_inv;

            // step 3: b = b + k e (line 37)
            bias += k * e;

            // step 4: P = P - K R P (line 38)
            self.state.bias_p -= k * r_p;

            // ensure that the new bias estimate is within the allowed range
            self.state.bias = bias.simd_clamp(-bias_clip_vector, bias_clip_vector);
        }
    }

    /// Resets the filter orientation to a specific quaternion.
    ///
    /// This will reset both the gyroscope and accelerometer quaternions,
    /// reinitialize the bias estimation, and reset all filter states.
    pub fn reset_orientation(&mut self, quat: UnitQuaternion<f32>) {
        // Reset quaternions
        self.state.gyroscope = quat;
        self.state.accelerometer = UnitQuaternion::identity();

        // Reset rest detection
        self.state.rest = None;

        // Reset all filter states
        self.state.accelerometer_low_pass = MeanInitializedLowPassFilter::with_coeffs(
            self.parameters.tau_accelerometer,
            self.accel_rate,
            self.coefficients.accel_b,
            self.coefficients.accel_a,
        );

        self.state.rest_gyro_low_pass = MeanInitializedLowPassFilter::with_coeffs(
            self.parameters.rest_filter_tau,
            self.gyro_rate,
            self.coefficients.rest_gyro_b,
            self.coefficients.rest_gyro_a,
        );

        self.state.rest_accel_low_pass = MeanInitializedLowPassFilter::with_coeffs(
            self.parameters.rest_filter_tau,
            self.accel_rate,
            self.coefficients.rest_accel_b,
            self.coefficients.rest_accel_a,
        );

        self.state.motion_bias_estimate_rotation_low_pass = MeanInitializedLowPassFilter::with_coeffs(
            self.parameters.tau_accelerometer,
            self.accel_rate,
            self.coefficients.accel_b,
            self.coefficients.accel_a,
        );

        self.state.motion_bias_estimate_low_pass = MeanInitializedLowPassFilter::with_coeffs(
            self.parameters.tau_accelerometer,
            self.accel_rate,
            self.coefficients.accel_b,
            self.coefficients.accel_a,
        );

        // Reset bias estimation
        self.state.bias = Vector3::zeros();
        self.state.bias_p = self.coefficients.bias.p0 * Matrix3::identity();
    }
}

impl Default for Vqf {
    fn default() -> Self {
        Vqf::new(Duration::from_millis(10), VqfParameters::default())
    }
}

impl Ahrs for Vqf {
    fn orientation(&self) -> UnitQuaternion<f32> {
        self.heading_orientation()
    }
    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) {
        self.reset_orientation(quat);
    }

    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32> {
        self.gyroscope_update(gyroscope);
        self.orientation()
    }

    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.gyroscope_update(gyroscope);
        self.accelerometer_update(accelerometer);
        self.heading_orientation()
    }
    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32> {
        self.gyroscope_update(gyroscope);
        self.accelerometer_update(accelerometer);
        self.magnetometer_update(magnetometer);
        self.heading_orientation()
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
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(10);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);

        let gyro = Vector3::new(0.0f32, 0.0f32, 0.0f32);
        ahrs.gyroscope_update(gyro);

        let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
        assert_abs_diff_eq!(roll, 0.0);
        assert_abs_diff_eq!(pitch, 0.0);
        assert_abs_diff_eq!(yaw, 0.0);
    }

    #[test]
    fn test_gyro_roll_estimation() {
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(100);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);

        ahrs.update_gyro(Vector3::new(0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, 0.05, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);

        ahrs.update_gyro(Vector3::new(-0.5f32, 0.0f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_pitch_estimation() {
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(100);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);
        ahrs.update_gyro(Vector3::new(0.0f32, 0.5f32, 0.0f32));
        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.05, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.0001);
    }

    #[test]
    fn test_gyro_yaw_estimation() {
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(100);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);
        ahrs.update_gyro(Vector3::new(0.0f32, 0.0f32, 0.5f32));
        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.0001);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.0001);
        assert_relative_eq!(yaw, 0.05, epsilon = 0.0001);
    }

    #[test]
    fn test_imu_pitch_45() {
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(100);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        // Corresponds to 45 deg pitch, assuming gravity is (0, 0, 1)
        let rad_45 = core::f32::consts::FRAC_PI_4;
        let accel = Vector3::new(-rad_45.sin(), 0.0, rad_45.cos());

        for _ in 0..1000 {
            ahrs.update_imu(gyro, accel);
        }

        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, 0.0, epsilon = 0.01);
        assert_relative_eq!(pitch, rad_45, epsilon = 0.01);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.01);
    }

    #[test]
    fn test_imu_roll_45() {
        let params = VqfParameters::default();
        let sample_period = Duration::from_millis(100);
        let mut ahrs =
            Vqf::new_with_sensor_rates(sample_period, sample_period, sample_period, params);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        // Corresponds to 45 deg roll, assuming gravity is (0, 0, 1)
        let rad_45 = core::f32::consts::FRAC_PI_4;
        let accel = Vector3::new(0.0, rad_45.sin(), rad_45.cos());

        for _ in 0..1000 {
            ahrs.update_imu(gyro, accel);
        }

        let (roll, pitch, yaw) = ahrs.quaternion().euler_angles();
        assert_relative_eq!(roll, rad_45, epsilon = 0.01);
        assert_relative_eq!(pitch, 0.0, epsilon = 0.01);
        assert_relative_eq!(yaw, 0.0, epsilon = 0.01);
    }
}
