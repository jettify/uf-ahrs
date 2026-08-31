use nalgebra::{Matrix3, Vector3};

/// Calibration model for gyroscope or accelerometer measurements.
///
/// Applies misalignment, sensitivity, and offset correction to a raw sensor
/// reading:
///
/// `calibrated = misalignment * (sensitivity ⊙ (uncalibrated - offset))`
///
/// where `⊙` denotes element-wise (Hadamard) multiplication. The default
/// value is an identity model (no correction applied), so it is always safe
/// to wire an [`InertialCalibration`] into a sensor pipeline even before
/// real calibration coefficients are known.
///
/// This model does not determine calibration coefficients; it only applies
/// coefficients determined by an external calibration procedure.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct InertialCalibration {
    /// Misalignment matrix correcting for non-orthogonality between sensor
    /// axes and rotational offset between the sensor and body frame.
    pub misalignment: Matrix3<f32>,
    /// Per-axis sensitivity scale factors.
    pub sensitivity: Vector3<f32>,
    /// Per-axis offset (bias) subtracted before scaling.
    pub offset: Vector3<f32>,
}

impl Default for InertialCalibration {
    fn default() -> Self {
        Self {
            misalignment: Matrix3::identity(),
            sensitivity: Vector3::new(1.0, 1.0, 1.0),
            offset: Vector3::zeros(),
        }
    }
}

impl InertialCalibration {
    /// Applies the calibration model to a raw gyroscope or accelerometer
    /// reading.
    #[must_use]
    pub fn calibrate(&self, uncalibrated: Vector3<f32>) -> Vector3<f32> {
        self.misalignment
            * self
                .sensitivity
                .component_mul(&(uncalibrated - self.offset))
    }
}

/// Calibration model for magnetometer measurements.
///
/// Applies soft-iron and hard-iron correction to a raw magnetometer
/// reading:
///
/// `calibrated = soft_iron * (uncalibrated - hard_iron)`
///
/// The default value is an identity model (no correction applied).
///
/// This model does not determine calibration coefficients; it only applies
/// coefficients determined by an external calibration procedure.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct MagnetometerCalibration {
    /// Soft-iron correction matrix.
    pub soft_iron: Matrix3<f32>,
    /// Hard-iron offset vector.
    pub hard_iron: Vector3<f32>,
}

impl Default for MagnetometerCalibration {
    fn default() -> Self {
        Self {
            soft_iron: Matrix3::identity(),
            hard_iron: Vector3::zeros(),
        }
    }
}

impl MagnetometerCalibration {
    /// Applies the calibration model to a raw magnetometer reading.
    #[must_use]
    pub fn calibrate(&self, uncalibrated: Vector3<f32>) -> Vector3<f32> {
        self.soft_iron * (uncalibrated - self.hard_iron)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn default_inertial_calibration_is_identity_passthrough() {
        let cal = InertialCalibration::default();
        let raw = Vector3::new(1.0, -2.5, 3.3);
        assert_abs_diff_eq!(cal.calibrate(raw), raw, epsilon = 1e-6);
    }

    #[test]
    fn inertial_calibration_subtracts_offset() {
        let cal = InertialCalibration {
            offset: Vector3::new(1.0, 2.0, 3.0),
            ..Default::default()
        };
        let calibrated = cal.calibrate(Vector3::new(1.0, 2.0, 3.0));
        assert_abs_diff_eq!(calibrated, Vector3::zeros(), epsilon = 1e-6);
    }

    #[test]
    fn inertial_calibration_scales_by_sensitivity() {
        let cal = InertialCalibration {
            sensitivity: Vector3::new(2.0, 0.5, 1.0),
            ..Default::default()
        };
        let calibrated = cal.calibrate(Vector3::new(1.0, 1.0, 1.0));
        assert_abs_diff_eq!(calibrated, Vector3::new(2.0, 0.5, 1.0), epsilon = 1e-6);
    }

    #[test]
    fn inertial_calibration_applies_misalignment_rotation() {
        // 90 degree rotation about Z: x -> y
        #[rustfmt::skip]
        let rotate_z_90 = Matrix3::new(
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
        );
        let cal = InertialCalibration {
            misalignment: rotate_z_90,
            ..Default::default()
        };
        let calibrated = cal.calibrate(Vector3::new(1.0, 0.0, 0.0));
        assert_abs_diff_eq!(calibrated, Vector3::new(0.0, 1.0, 0.0), epsilon = 1e-6);
    }

    #[test]
    fn default_magnetometer_calibration_is_identity_passthrough() {
        let cal = MagnetometerCalibration::default();
        let raw = Vector3::new(10.0, -5.0, 2.0);
        assert_abs_diff_eq!(cal.calibrate(raw), raw, epsilon = 1e-6);
    }

    #[test]
    fn magnetometer_calibration_subtracts_hard_iron_offset() {
        let cal = MagnetometerCalibration {
            hard_iron: Vector3::new(5.0, -3.0, 1.0),
            ..Default::default()
        };
        let calibrated = cal.calibrate(Vector3::new(5.0, -3.0, 1.0));
        assert_abs_diff_eq!(calibrated, Vector3::zeros(), epsilon = 1e-6);
    }

    #[test]
    fn magnetometer_calibration_applies_soft_iron_scaling() {
        #[rustfmt::skip]
        let soft_iron = Matrix3::new(
            2.0, 0.0, 0.0,
            0.0, 0.5, 0.0,
            0.0, 0.0, 1.0,
        );
        let cal = MagnetometerCalibration {
            soft_iron,
            ..Default::default()
        };
        let calibrated = cal.calibrate(Vector3::new(1.0, 1.0, 1.0));
        assert_abs_diff_eq!(calibrated, Vector3::new(2.0, 0.5, 1.0), epsilon = 1e-6);
    }
}
