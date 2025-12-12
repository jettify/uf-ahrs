use nalgebra::{UnitQuaternion, Vector3};

pub trait Ahrs {
    fn orientation(&self) -> UnitQuaternion<f32>;
    fn update(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
        magnetometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;
    fn update_imu(
        &mut self,
        gyroscope: Vector3<f32>,
        accelerometer: Vector3<f32>,
    ) -> UnitQuaternion<f32>;
    fn update_gyro(&mut self, gyroscope: Vector3<f32>) -> UnitQuaternion<f32>;
}
