use core::time::Duration;

use crate::accel_low_pass::AccelLowPass;
use crate::gyro_integration::integrate_gyro;

use nalgebra::{UnitQuaternion, Vector3};

/// Coordinates the independent gyro frame and elapsed time between valid samples.
#[derive(Debug)]
pub(crate) struct AccelPrefilter {
    frame: UnitQuaternion<f32>,
    low_pass: AccelLowPass,
    pending: Duration,
}

pub(crate) fn configure(previous: Option<AccelPrefilter>, tau: Duration) -> Option<AccelPrefilter> {
    if tau.is_zero() {
        None
    } else {
        Some(AccelPrefilter {
            frame: previous.map_or_else(UnitQuaternion::identity, |state| state.frame),
            low_pass: AccelLowPass::default(),
            pending: Duration::ZERO,
        })
    }
}

pub(crate) fn direction(vector: Vector3<f32>) -> Option<Vector3<f32>> {
    if !vector.iter().all(|v| v.is_finite()) {
        return None;
    }
    let scale = vector.amax();
    if scale <= f32::EPSILON {
        return None;
    }
    (vector / scale).try_normalize(f32::EPSILON)
}

pub(crate) fn finite_orientation(q: UnitQuaternion<f32>) -> bool {
    q.coords.iter().all(|v| v.is_finite()) && (q.norm_squared() - 1.0).abs() < 1e-3
}

impl AccelPrefilter {
    pub(crate) fn advance(&mut self, rate: Vector3<f32>, dt: Duration) -> bool {
        let frame = integrate_gyro(self.frame, rate, dt.as_secs_f32());
        if !finite_orientation(frame) {
            return false;
        }
        self.frame = frame;
        self.pending = self.pending.saturating_add(dt);
        true
    }

    pub(crate) fn filter(&mut self, accel: Vector3<f32>, tau: Duration) -> Option<Vector3<f32>> {
        // Validate raw input without normalizing a direction we would discard.
        // After scaling by amax, one component would have magnitude one, so
        // direction() cannot reject any additional finite input here.
        if !accel.iter().all(|v| v.is_finite()) || accel.amax() <= f32::EPSILON {
            return None;
        }
        let filtered =
            self.low_pass
                .filter(self.frame.transform_vector(&accel), self.pending, tau)?;
        self.pending = Duration::ZERO;
        direction(self.frame.inverse_transform_vector(&filtered))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Ahrs, AhrsWithDt, Madgwick, MadgwickParams, Mahony, MahonyParams};

    #[test]
    fn test_raw_mean_and_dropouts_use_full_elapsed_time() {
        let tau = Duration::from_secs(1);
        let mut f = configure(None, tau).unwrap();
        assert!(f.advance(Vector3::zeros(), Duration::from_millis(250)));
        f.filter(Vector3::z(), tau).unwrap();
        assert!(f.advance(Vector3::zeros(), Duration::from_millis(500)));
        assert!(f.filter(Vector3::zeros(), tau).is_none());
        assert_eq!(f.pending, Duration::from_millis(500));
        assert!(f.advance(Vector3::zeros(), Duration::from_millis(250)));
        let actual = f.filter(Vector3::x() * 3.0, tau).unwrap();
        let expected = direction(Vector3::new(2.25, 0.0, 0.25)).unwrap();
        assert!((actual - expected).norm() < 1e-6);
        assert_eq!(f.pending, Duration::ZERO);
    }

    #[test]
    fn test_tau_changes_preserve_frame_but_reset_history() {
        let mut f = configure(None, Duration::from_secs(1)).unwrap();
        assert!(f.advance(Vector3::new(0.2, 0.3, 0.4), Duration::from_secs(1)));
        f.filter(Vector3::z(), Duration::from_secs(1)).unwrap();
        let frame = f.frame;
        let mut f = configure(Some(f), Duration::from_secs(2)).unwrap();
        assert_eq!(f.frame, frame);
        assert_eq!(f.pending, Duration::ZERO);
        assert!(f.advance(Vector3::zeros(), Duration::from_millis(5)));
        assert!(
            (f.filter(Vector3::x(), Duration::from_secs(2)).unwrap() - Vector3::x()).norm() < 1e-6
        );
        assert!(configure(Some(f), Duration::ZERO).is_none());
    }

    #[test]
    fn test_rotation_and_irregular_periods_do_not_create_filter_lag() {
        for initial in [
            UnitQuaternion::identity(),
            UnitQuaternion::from_euler_angles(0.7, -0.4, 0.2),
            UnitQuaternion::from_euler_angles(3.0, 0.1, -0.2),
        ] {
            let mut mahony = Mahony::new_with_orientation(
                Duration::from_millis(5),
                MahonyParams {
                    ki: 0.0,
                    tau_accelerometer: Duration::from_millis(750),
                    ..Default::default()
                },
                initial,
            );
            let mut madgwick = Madgwick::new_with_orientation(
                Duration::from_millis(5),
                MadgwickParams {
                    tau_accelerometer: Duration::from_secs(2),
                    ..Default::default()
                },
                initial,
            );
            let mut truth = initial;
            for i in 0..2000 {
                let dt = Duration::from_micros([2000, 3500, 5000, 8000][i % 4]);
                let t = i as f32 * 0.01;
                let gyro = Vector3::new(t.sin(), 0.4 * t.cos(), 0.3);
                truth = integrate_gyro(truth, gyro, dt.as_secs_f32());
                let accel = truth.inverse_transform_vector(&Vector3::z());
                for q in [
                    mahony.update_imu_with_dt(dt, gyro, accel),
                    madgwick.update_imu_with_dt(dt, gyro, accel),
                ] {
                    assert!((truth.inverse() * q).angle() < 0.015);
                }
            }
        }
    }

    #[test]
    fn test_translation_pulse_is_smoothed_and_recovers() {
        let dt = Duration::from_millis(5);
        let mut filtered = Mahony::new(
            dt,
            MahonyParams {
                ki: 0.0,
                tau_accelerometer: Duration::from_millis(750),
                ..Default::default()
            },
        );
        let mut original = Mahony::new(
            dt,
            MahonyParams {
                ki: 0.0,
                ..Default::default()
            },
        );
        let mut peaks = [0.0_f32; 2];
        for i in 0..6000 {
            let accel = if (1000..1040).contains(&i) {
                Vector3::new(1.0, 0.0, 1.0)
            } else {
                Vector3::z()
            };
            peaks[0] = peaks[0].max(filtered.update_imu(Vector3::zeros(), accel).angle());
            peaks[1] = peaks[1].max(original.update_imu(Vector3::zeros(), accel).angle());
        }
        assert!(peaks[0] < peaks[1] * 0.9);
        assert!(filtered.orientation().angle() < 0.001);
    }
    #[test]
    fn test_raw_validation_retains_direction_acceptance_at_extremes() {
        let tau = Duration::from_secs(1);
        for accel in [
            Vector3::zeros(),
            Vector3::repeat(f32::NAN),
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::MIN_POSITIVE),
            Vector3::new(f32::EPSILON, 0.0, 0.0),
            Vector3::new(f32::from_bits(f32::EPSILON.to_bits() + 1), 0.0, 0.0),
            Vector3::new(f32::MAX, -f32::MAX, f32::MAX),
            Vector3::new(0.0, -1.0, 0.0),
        ] {
            let mut f = configure(None, tau).unwrap();
            assert!(f.advance(Vector3::zeros(), Duration::from_millis(5)));
            assert_eq!(
                f.filter(accel, tau).is_some(),
                direction(accel).is_some(),
                "{accel:?}"
            );
        }
    }
}
