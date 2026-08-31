use nalgebra::Vector3;

/// A signed sensor axis, expressed relative to the sensor's own mounting
/// frame.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Axis {
    /// Positive X.
    XPos,
    /// Negative X.
    XNeg,
    /// Positive Y.
    YPos,
    /// Negative Y.
    YNeg,
    /// Positive Z.
    ZPos,
    /// Negative Z.
    ZNeg,
}

impl Axis {
    /// The physical axis (0 = X, 1 = Y, 2 = Z) this variant refers to,
    /// ignoring sign.
    const fn physical(self) -> u8 {
        match self {
            Axis::XPos | Axis::XNeg => 0,
            Axis::YPos | Axis::YNeg => 1,
            Axis::ZPos | Axis::ZNeg => 2,
        }
    }

    fn select(self, v: &Vector3<f32>) -> f32 {
        match self {
            Axis::XPos => v.x,
            Axis::XNeg => -v.x,
            Axis::YPos => v.y,
            Axis::YNeg => -v.y,
            Axis::ZPos => v.z,
            Axis::ZNeg => -v.z,
        }
    }
}

/// Remaps a sensor vector from the sensor's mounting frame to the body
/// frame.
///
/// Sensors are often mounted such that their axes are not aligned with the
/// body axes (e.g. an IMU mounted rotated 90 degrees, or upside down, on a
/// PCB). `AxisRemap` describes, for each body axis, which signed sensor
/// axis it corresponds to.
///
/// [`AxisRemap::remap`] applies the same permutation to gyroscope,
/// accelerometer, and magnetometer readings, since all three sensors share
/// the same rigid mounting frame.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AxisRemap {
    x: Axis,
    y: Axis,
    z: Axis,
}

impl AxisRemap {
    /// No remapping: the sensor frame already matches the body frame.
    pub const IDENTITY: Self = Self {
        x: Axis::XPos,
        y: Axis::YPos,
        z: Axis::ZPos,
    };

    /// Creates a remap from the sensor axis that should be read as the
    /// body's X, Y, and Z axis respectively.
    ///
    /// Returns `None` if `x`, `y`, and `z` do not refer to three distinct
    /// physical sensor axes (for example, `x` and `y` both referring to the
    /// physical X axis), since that would not describe a valid coordinate
    /// frame permutation.
    #[must_use]
    pub fn new(x: Axis, y: Axis, z: Axis) -> Option<Self> {
        if x.physical() == y.physical()
            || y.physical() == z.physical()
            || x.physical() == z.physical()
        {
            None
        } else {
            Some(Self { x, y, z })
        }
    }

    /// Applies the remap to a raw sensor-frame vector, returning the
    /// equivalent body-frame vector.
    #[must_use]
    pub fn remap(&self, v: Vector3<f32>) -> Vector3<f32> {
        Vector3::new(self.x.select(&v), self.y.select(&v), self.z.select(&v))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn identity_is_no_op() {
        let v = Vector3::new(1.0, -2.0, 3.5);
        assert_abs_diff_eq!(AxisRemap::IDENTITY.remap(v), v, epsilon = 1e-6);
    }

    #[test]
    fn new_accepts_a_valid_permutation() {
        assert!(AxisRemap::new(Axis::XPos, Axis::YPos, Axis::ZPos).is_some());
        assert!(AxisRemap::new(Axis::YNeg, Axis::XPos, Axis::ZPos).is_some());
        assert!(AxisRemap::new(Axis::ZPos, Axis::YNeg, Axis::XNeg).is_some());
    }

    #[test]
    fn new_rejects_a_repeated_physical_axis() {
        // x and y both reference the physical X axis.
        assert!(AxisRemap::new(Axis::XPos, Axis::XNeg, Axis::ZPos).is_none());
        // y and z both reference the physical Y axis.
        assert!(AxisRemap::new(Axis::XPos, Axis::YPos, Axis::YNeg).is_none());
        // x and z both reference the physical Z axis.
        assert!(AxisRemap::new(Axis::ZPos, Axis::YPos, Axis::ZNeg).is_none());
    }

    #[test]
    fn remap_selects_and_negates_the_configured_axes() {
        // Sensor mounted rotated 90 degrees about Z relative to the body:
        // body X reads the sensor's Y axis, body Y reads the sensor's
        // negated X axis, body Z is unchanged.
        let remap = AxisRemap::new(Axis::YPos, Axis::XNeg, Axis::ZPos).expect("valid permutation");
        let sensor = Vector3::new(1.0, 2.0, 3.0);
        let body = remap.remap(sensor);
        assert_abs_diff_eq!(body, Vector3::new(2.0, -1.0, 3.0), epsilon = 1e-6);
    }

    #[test]
    fn remap_handles_axis_flip() {
        // Sensor mounted upside down: X unchanged, Y and Z flipped.
        let remap = AxisRemap::new(Axis::XPos, Axis::YNeg, Axis::ZNeg).expect("valid permutation");
        let sensor = Vector3::new(1.0, 2.0, 3.0);
        let body = remap.remap(sensor);
        assert_abs_diff_eq!(body, Vector3::new(1.0, -2.0, -3.0), epsilon = 1e-6);
    }
}
