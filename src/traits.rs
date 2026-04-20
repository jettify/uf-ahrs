use nalgebra::{UnitQuaternion, Vector3};

/// Common interface for AHRS filters that estimate orientation from IMU/MARG data.
pub trait Ahrs {
    /// Returns the current orientation estimate as a unit quaternion.
    fn orientation(&self) -> UnitQuaternion<f32>;

    /// Sets the current orientation estimate.
    ///
    /// Implementations may also reset internal state coupled to orientation,
    /// such as bias estimators or low-pass filter history.
    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) -> ();

    /// Updates the filter with gyroscope, accelerometer, and magnetometer data.
    ///
    /// - `gyroscope` must be in radians per second.
    /// - `accelerometer` and `magnetometer` can be in arbitrary units.
    ///   Their absolute scale is not important because vectors are normalized internally.
    ///   Keep units consistent within each sensor stream over time.
    ///
    /// Returns the updated orientation estimate.
    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;

    /// Updates the filter with gyroscope and accelerometer data only.
    ///
    /// - `gyroscope` must be in radians per second.
    /// - `accelerometer` can be in arbitrary units.
    ///   Its absolute scale is not important because vectors are normalized internally.
    ///   Keep units consistent across samples.
    ///
    /// Returns the updated orientation estimate.
    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;

    /// Updates the filter using only gyroscope integration.
    ///
    /// `gyroscope` must be in radians per second.
    /// Returns the updated orientation estimate.
    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32>;
}
