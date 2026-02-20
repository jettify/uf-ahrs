#![cfg(test)]
extern crate std;

use core::f32::consts::{FRAC_PI_2, FRAC_PI_4};
use core::time::Duration;

use approx::{assert_abs_diff_eq, assert_relative_eq};
use nalgebra::{UnitQuaternion, Vector3};
use rstest::{fixture, rstest};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParameters};

type AhrsBox = Box<dyn Ahrs>;

const EPS_GYRO: f32 = 1e-4;
const EPS_IMU: f32 = 1e-2;
const EPS_UPDATE: f32 = 1e-2;
const IMU_STEPS: usize = 1000;
const UPDATE_STEPS: usize = 2000;

const GYRO_PERIOD: Duration = Duration::from_millis(100);
const IMU_PERIOD: Duration = Duration::from_millis(10);

#[fixture]
fn mahony_gyro() -> AhrsBox {
    Box::new(Mahony::new(GYRO_PERIOD, MahonyParams::default()))
}

#[fixture]
fn mahony_imu() -> AhrsBox {
    Box::new(Mahony::new(IMU_PERIOD, MahonyParams::default()))
}

#[fixture]
fn madgwick_gyro() -> AhrsBox {
    Box::new(Madgwick::new(GYRO_PERIOD, MadgwickParams::default()))
}

#[fixture]
fn madgwick_imu() -> AhrsBox {
    Box::new(Madgwick::new(IMU_PERIOD, MadgwickParams::default()))
}

#[fixture]
fn vqf_gyro() -> AhrsBox {
    Box::new(Vqf::new(GYRO_PERIOD, VqfParameters::default()))
}

#[fixture]
fn vqf_imu() -> AhrsBox {
    Box::new(Vqf::new(IMU_PERIOD, VqfParameters::default()))
}

fn assert_euler_close(
    quat: UnitQuaternion<f32>,
    expected_roll: f32,
    expected_pitch: f32,
    expected_yaw: f32,
    epsilon: f32,
) {
    let (roll, pitch, yaw) = quat.euler_angles();
    assert_relative_eq!(roll, expected_roll, epsilon = epsilon);
    assert_relative_eq!(pitch, expected_pitch, epsilon = epsilon);
    assert_relative_eq!(yaw, expected_yaw, epsilon = epsilon);
}

fn run_imu_steps(ahrs: &mut dyn Ahrs, steps: usize, gyro: Vector3<f32>, accel: Vector3<f32>) {
    for _ in 0..steps {
        ahrs.update_imu(gyro, accel);
    }
}

fn run_update_steps(
    ahrs: &mut dyn Ahrs,
    steps: usize,
    gyro: Vector3<f32>,
    accel: Vector3<f32>,
    mag: Vector3<f32>,
) {
    for _ in 0..steps {
        ahrs.update(gyro, accel, mag);
    }
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
#[case::vqf(vqf_gyro())]
fn test_zero_rotation(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0f32, 0.0f32, 0.0f32);
    ahrs.update_gyro(gyro);
    let (roll, pitch, yaw) = ahrs.orientation().euler_angles();
    assert_abs_diff_eq!(roll, 0.0);
    assert_abs_diff_eq!(pitch, 0.0);
    assert_abs_diff_eq!(yaw, 0.0);
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
#[case::vqf(vqf_gyro())]
fn test_gyro_roll_estimation(#[case] mut ahrs: AhrsBox) {
    ahrs.update_gyro(Vector3::new(0.5f32, 0.0f32, 0.0f32));
    assert_euler_close(ahrs.orientation(), 0.05, 0.0, 0.0, EPS_GYRO);

    ahrs.update_gyro(Vector3::new(-0.5f32, 0.0f32, 0.0f32));
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.0, EPS_GYRO);
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
#[case::vqf(vqf_gyro())]
fn test_gyro_pitch_estimation(#[case] mut ahrs: AhrsBox) {
    ahrs.update_gyro(Vector3::new(0.0f32, 0.5f32, 0.0f32));
    assert_euler_close(ahrs.orientation(), 0.0, 0.05, 0.0, EPS_GYRO);
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
#[case::vqf(vqf_gyro())]
fn test_gyro_yaw_estimation(#[case] mut ahrs: AhrsBox) {
    ahrs.update_gyro(Vector3::new(0.0f32, 0.0f32, 0.5f32));
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.05, EPS_GYRO);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_imu_level(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, 0.0, 9.81);
    run_imu_steps(&mut *ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.0, EPS_IMU);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_imu_pitch_45(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(-FRAC_PI_4.sin(), 0.0, FRAC_PI_4.cos());
    run_imu_steps(&mut *ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), 0.0, FRAC_PI_4, 0.0, EPS_IMU);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_imu_roll_45(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, FRAC_PI_4.sin(), FRAC_PI_4.cos());
    run_imu_steps(&mut *ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), FRAC_PI_4, 0.0, 0.0, EPS_IMU);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_update_level(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(1.0, 0.0, 0.0);
    run_update_steps(&mut *ahrs, UPDATE_STEPS, gyro, accel, mag);
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.0, EPS_UPDATE);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_update_yaw_90(#[case] mut ahrs: AhrsBox) {
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(0.0, -1.0, 0.0);
    run_update_steps(&mut *ahrs, UPDATE_STEPS, gyro, accel, mag);
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, FRAC_PI_2, EPS_UPDATE);
}

#[rstest]
#[case::mahony(mahony_imu(), mahony_imu())]
#[case::madgwick(madgwick_imu(), madgwick_imu())]
#[case::vqf(vqf_imu(), vqf_imu())]
fn test_update_zero_magnetometer_falls_back_to_imu(
    #[case] mut ahrs_update: AhrsBox,
    #[case] mut ahrs_imu: AhrsBox,
) {
    let gyro = Vector3::new(0.01, 0.02, 0.03);
    let accel = Vector3::new(0.1, 0.2, 9.8);
    let mag = Vector3::new(0.0, 0.0, 0.0);

    ahrs_update.update(gyro, accel, mag);
    ahrs_imu.update_imu(gyro, accel);

    assert_relative_eq!(ahrs_update.orientation(), ahrs_imu.orientation());
}

#[rstest]
#[case::mahony(mahony_gyro(), mahony_gyro())]
#[case::madgwick(madgwick_gyro(), madgwick_gyro())]
#[case::vqf(vqf_gyro(), vqf_gyro())]
fn test_imu_zero_accelerometer_falls_back_to_gyro(
    #[case] mut ahrs_imu: AhrsBox,
    #[case] mut ahrs_gyro: AhrsBox,
) {
    let gyro = Vector3::new(0.1, -0.2, 0.3);
    let accel = Vector3::new(0.0, 0.0, 0.0);

    ahrs_imu.update_imu(gyro, accel);
    ahrs_gyro.update_gyro(gyro);

    assert_relative_eq!(ahrs_imu.orientation(), ahrs_gyro.orientation());
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
#[case::vqf(vqf_gyro())]
fn test_set_orientation_round_trip(#[case] mut ahrs: AhrsBox) {
    let expected = UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3);
    ahrs.set_orientation(expected);
    assert_relative_eq!(ahrs.orientation(), expected);
}

#[test]
fn test_default_constructors_start_identity_orientation() {
    let mahony = Mahony::default();
    let madgwick = Madgwick::default();
    let vqf = Vqf::default();

    assert_relative_eq!(mahony.orientation(), UnitQuaternion::identity());
    assert_relative_eq!(madgwick.orientation(), UnitQuaternion::identity());
    assert_relative_eq!(vqf.orientation(), UnitQuaternion::identity());
}

#[test]
fn test_update_zero_accelerometer_falls_back_to_gyro_for_mahony_and_madgwick() {
    let gyro = Vector3::new(0.12, -0.34, 0.56);
    let accel = Vector3::new(0.0, 0.0, 0.0);
    let mag = Vector3::new(1.0, 0.0, 0.0);

    let mut mahony_update = Mahony::new(IMU_PERIOD, MahonyParams::default());
    let mut mahony_gyro = Mahony::new(IMU_PERIOD, MahonyParams::default());
    mahony_update.update(gyro, accel, mag);
    mahony_gyro.update_gyro(gyro);
    assert_relative_eq!(mahony_update.orientation(), mahony_gyro.orientation());

    let mut madgwick_update = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    let mut madgwick_gyro = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    madgwick_update.update(gyro, accel, mag);
    madgwick_gyro.update_gyro(gyro);
    assert_relative_eq!(madgwick_update.orientation(), madgwick_gyro.orientation());
}

#[test]
fn test_vqf_constructors_with_orientation_apply_given_orientation() {
    let period = Duration::from_millis(10);
    let params = VqfParameters::default();
    let expected = UnitQuaternion::from_euler_angles(0.2, -0.1, 0.3);

    let vqf_with_orientation = Vqf::new_with_orientation(period, params.clone(), expected);
    assert_relative_eq!(vqf_with_orientation.orientation(), expected, epsilon = 1e-6);

    let vqf_with_sensor_rates = Vqf::new_with_sensor_rates_and_orientation(
        period,
        Duration::from_millis(20),
        Duration::from_millis(30),
        params,
        expected,
    );
    assert_relative_eq!(vqf_with_sensor_rates.orientation(), expected, epsilon = 1e-6);
}

#[test]
fn test_vqf_update2_updates_orientation() {
    let mut vqf = Vqf::new(Duration::from_millis(10), VqfParameters::default());
    let initial = vqf.orientation();
    vqf.update2(Vector3::new(0.3, -0.1, 0.2), Vector3::new(0.1, 0.2, 9.8));
    assert!(vqf.orientation().angle_to(&initial) > 0.0);
}
