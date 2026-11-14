use core::time::Duration;

use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

/// Common interface for AHRS filters that estimate orientation from IMU/MARG data.
pub trait Ahrs {
    /// Returns the current orientation estimate as a unit quaternion.
    fn orientation(&self) -> UnitQuaternion<f32>;

    /// Sets the current orientation estimate.
    ///
    /// Implementations may also reset internal state coupled to orientation,
    /// such as bias estimators or low-pass filter history.
    fn set_orientation(&mut self, quat: UnitQuaternion<f32>) -> ();

    /// Converts an external heading in radians into a synthetic magnetometer reading.
    ///
    /// The finite `heading` follows the orientation's positive yaw convention:
    /// at identity orientation, 0 radians produces positive X and $\pi / 2$ radians
    /// produces negative Y. The result is a unit vector in the body frame,
    /// compensated for the current roll and pitch without extracting Euler angles.
    /// This method does not change the filter state.
    ///
    /// Conventional heading is undefined when the body's forward (positive X)
    /// axis is vertical. Returns zero when `heading` is non-finite or when that
    /// axis's horizontal projection has length at most `1e-3` (within
    /// approximately 0.001 radians of vertical). Passing this zero vector to
    /// [`Ahrs::update`] skips magnetometer correction.
    #[must_use]
    fn heading_magnetometer(&self, heading: f32) -> Vector3<f32> {
        if !heading.is_finite() {
            return Vector3::zeros();
        }

        let orientation = self.orientation();
        let forward = orientation.transform_vector(&Vector3::x());
        let Some(horizontal) = Vector3::new(forward.x, forward.y, 0.0).try_normalize(1e-3) else {
            return Vector3::zeros();
        };

        let sin_heading = libm::sinf(heading);
        let cos_heading = libm::cosf(heading);

        // The normalized projection holds cos/sin of the current heading.
        // Rotate world X by current_heading - external_heading, then transform
        // into the body frame to preserve tilt while replacing the heading.
        let reference = Vector3::new(
            horizontal.x * cos_heading + horizontal.y * sin_heading,
            horizontal.y * cos_heading - horizontal.x * sin_heading,
            0.0,
        );
        orientation.inverse_transform_vector(&reference)
    }

    /// Updates the filter with gyroscope, accelerometer, and external heading data.
    ///
    /// `gyroscope` is in radians per second, `accelerometer` is in arbitrary
    /// consistent units, and the finite `heading` is in radians following the
    /// orientation's positive yaw convention.
    ///
    /// The default implementation converts the heading with
    /// [`Ahrs::heading_magnetometer`] and delegates to [`Ahrs::update`], retaining
    /// the filter's correction gains and magnetometer rejection behavior.
    /// Near vertical, only gyro/accelerometer updates apply.
    /// Returns the updated orientation estimate; heading is not set instantly.
    ///
    /// ```
    /// use core::f32::consts::FRAC_PI_2;
    /// use nalgebra::Vector3;
    /// use uf_ahrs::{Ahrs, Mahony};
    ///
    /// let mut ahrs = Mahony::default();
    /// let orientation = ahrs.update_external_heading(
    ///     Vector3::zeros(),
    ///     Vector3::new(0.0, 0.0, 9.81),
    ///     FRAC_PI_2,
    /// );
    /// ```
    fn update_external_heading(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        heading: f32,
    ) -> UnitQuaternion<f32> {
        let magnetometer = self.heading_magnetometer(heading);
        self.update(gyroscope, accelerometer, magnetometer)
    }

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

/// Optional interface for AHRS filters that support a per-update sample period.
///
/// Implementations use the supplied duration for all time-dependent operations
/// in the update, including gyro integration and estimator state updates.
pub trait AhrsWithDt: Ahrs {
    /// Updates the filter with gyroscope, accelerometer, and external heading data.
    ///
    /// Converts the heading with [`Ahrs::heading_magnetometer`] and delegates to
    /// [`AhrsWithDt::update_with_dt`].
    fn update_external_heading_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        heading: f32,
    ) -> UnitQuaternion<f32> {
        let magnetometer = self.heading_magnetometer(heading);
        self.update_with_dt(dt, gyroscope, accelerometer, magnetometer)
    }

    /// Updates the filter with gyroscope, accelerometer, and magnetometer data.
    fn update_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;

    /// Updates the filter with gyroscope and accelerometer data only.
    fn update_imu_with_dt(
        &mut self,
        dt: Duration,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;

    /// Updates the filter using only gyroscope integration.
    fn update_gyro_with_dt(&mut self, dt: Duration, gyroscope: Vector3<f32>)
    -> UnitQuaternion<f32>;
}
