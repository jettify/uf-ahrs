#![cfg(test)]
extern crate std;

use core::f32::consts::FRAC_PI_2;
use core::f32::consts::FRAC_PI_4;
use core::f32::consts::PI;
use core::time::Duration;

use approx::assert_abs_diff_eq;
use approx::assert_relative_eq;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;
use rstest::fixture;
use rstest::rstest;
use uf_ahrs::Ahrs;
use uf_ahrs::AhrsWithDt;
use uf_ahrs::Madgwick;
use uf_ahrs::MadgwickParams;
use uf_ahrs::Mahony;
use uf_ahrs::MahonyParams;
use uf_ahrs::Vqf;
use uf_ahrs::VqfParams;

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
    Box::new(Vqf::new(GYRO_PERIOD, VqfParams::default()))
}

#[fixture]
fn vqf_imu() -> AhrsBox {
    Box::new(Vqf::new(IMU_PERIOD, VqfParams::default()))
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
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
fn test_gyro_large_step_matches_body_frame_rotation(#[case] mut ahrs: AhrsBox) {
    let initial = UnitQuaternion::from_euler_angles(0.6, -0.4, 0.8);
    let axis = nalgebra::Unit::new_normalize(Vector3::new(1.0, -2.0, 3.0));
    ahrs.set_orientation(initial);
    // A 1.2 radian step exposes the angle error of normalized Euler integration.
    let actual = ahrs.update_gyro(axis.into_inner() * 12.0);
    let expected = initial * UnitQuaternion::from_axis_angle(&axis, 1.2);
    assert!(
        actual.angle_to(&expected) < 1e-6,
        "incorrect body-frame rotation"
    );
}

#[rstest]
#[case::mahony(mahony_gyro())]
#[case::madgwick(madgwick_gyro())]
fn test_gyro_repeated_rotations_preserve_accuracy_and_norm(#[case] mut ahrs: AhrsBox) {
    let axis = nalgebra::Unit::new_normalize(Vector3::new(-2.0, 1.0, 3.0));
    for _ in 0..10_000 {
        ahrs.update_gyro(axis.into_inner() * 0.7);
    }
    let expected = UnitQuaternion::from_axis_angle(&axis, 700.0);
    assert!(
        ahrs.orientation().angle_to(&expected) < 2e-4,
        "accumulated rotation error"
    );
    assert_abs_diff_eq!(ahrs.orientation().quaternion().norm(), 1.0, epsilon = 2e-7);
}

#[rstest]
#[case::mahony(Box::new(Mahony::default()) as Box<dyn AhrsWithDt>)]
#[case::madgwick(Box::new(Madgwick::default()) as Box<dyn AhrsWithDt>)]
fn test_gyro_zero_and_tiny_steps(#[case] mut ahrs: Box<dyn AhrsWithDt>) {
    let initial = UnitQuaternion::from_euler_angles(0.3, -0.2, 0.5);
    ahrs.set_orientation(initial);
    ahrs.update_gyro_with_dt(Duration::ZERO, Vector3::new(10.0, -20.0, 30.0));
    assert_relative_eq!(ahrs.orientation(), initial, epsilon = 1e-7);
    ahrs.update_gyro_with_dt(Duration::from_secs(1), Vector3::zeros());
    assert!(
        ahrs.orientation().angle_to(&initial) < 1e-7,
        "zero gyro changed orientation"
    );

    // Test near zero, including a rate whose squared norm underflows in f32.
    for rate in [1e-7, 1e-25] {
        ahrs.set_orientation(UnitQuaternion::identity());
        let actual = ahrs.update_gyro_with_dt(Duration::from_secs(1), Vector3::x() * rate);
        assert_relative_eq!(
            actual.quaternion().i,
            rate * 0.5,
            max_relative = 1e-6,
            epsilon = 0.0
        );
        assert_abs_diff_eq!(actual.quaternion().w, 1.0, epsilon = 1e-7);
    }
}

#[rstest]
#[case::mahony(Box::new(Mahony::new(IMU_PERIOD, MahonyParams { kp: 0.0, ki: 0.0 })) as Box<dyn AhrsWithDt>)]
#[case::madgwick(Box::new(Madgwick::new(IMU_PERIOD, MadgwickParams { beta: 0.0 })) as Box<dyn AhrsWithDt>)]
fn test_zero_feedback_all_update_paths_match_exponential(#[case] mut ahrs: Box<dyn AhrsWithDt>) {
    let initial = UnitQuaternion::from_euler_angles(0.3, -0.5, 0.7);
    let axis = nalgebra::Unit::new_normalize(Vector3::new(1.0, 2.0, -3.0));
    let gyro = axis.into_inner() * 8.0;
    let accel = Vector3::new(0.2, -0.3, 1.0);
    let mag = Vector3::new(0.5, 0.1, -0.2);
    let dt = Duration::from_millis(200);
    let expected = initial * UnitQuaternion::from_axis_angle(&axis, 1.6);
    for mode in 0..6 {
        ahrs.set_orientation(initial);
        let actual = match mode {
            0 => ahrs.update_gyro_with_dt(dt, gyro),
            1 => ahrs.update_imu_with_dt(dt, gyro, accel),
            2 => ahrs.update_with_dt(dt, gyro, accel, mag),
            3 => ahrs.update_imu_with_dt(dt, gyro, Vector3::zeros()),
            4 => ahrs.update_with_dt(dt, gyro, Vector3::zeros(), mag),
            _ => ahrs.update_with_dt(dt, gyro, accel, Vector3::zeros()),
        };
        assert!(
            actual.angle_to(&expected) < 1e-6,
            "update path {mode} differs from exponential"
        );
    }
}

#[test]
fn test_mahony_exponential_uses_bias_corrected_rate() {
    let mut ahrs = Mahony::new(IMU_PERIOD, MahonyParams { kp: 0.0, ki: 0.0 });
    ahrs.bias = Vector3::new(1.0, -2.0, 0.5);
    let gyro = ahrs.bias + Vector3::z() * 6.0;
    let actual = ahrs.update_imu_with_dt(Duration::from_millis(200), gyro, Vector3::z());
    let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.2);
    assert!(
        actual.angle_to(&expected) < 1e-6,
        "bias was not removed before integration"
    );
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

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_heading_magnetometer_level_headings(#[case] ahrs: AhrsBox) {
    for (heading, expected) in [
        (0.0, Vector3::x()),
        (FRAC_PI_2, -Vector3::y()),
        (PI, -Vector3::x()),
        (3.0 * FRAC_PI_2, Vector3::y()),
        (-FRAC_PI_2, Vector3::y()),
        (5.0 * FRAC_PI_2, -Vector3::y()),
    ] {
        assert_relative_eq!(ahrs.heading_magnetometer(heading), expected, epsilon = 1e-6);
    }
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_heading_magnetometer_matches_tilt_compensated_reference(#[case] mut ahrs: AhrsBox) {
    // Include inverted attitudes and pitch outside the canonical Euler range.
    for (roll, pitch) in [(0.4, -0.7), (-1.2, 0.8), (2.8, -0.3), (0.5, 2.0)] {
        for yaw in [-2.0, 0.0, 1.3] {
            let orientation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            ahrs.set_orientation(orientation);
            let (canonical_roll, canonical_pitch, _) = orientation.euler_angles();
            for heading in [
                -135.0_f32.to_radians(),
                0.0,
                60.0_f32.to_radians(),
                420.0_f32.to_radians(),
            ] {
                let target =
                    UnitQuaternion::from_euler_angles(canonical_roll, canonical_pitch, heading);
                let mag = ahrs.heading_magnetometer(heading);
                assert_relative_eq!(
                    mag,
                    target.inverse_transform_vector(&Vector3::x()),
                    epsilon = 1e-6
                );
                assert_relative_eq!(target * mag, Vector3::x(), epsilon = 1e-6);
                assert_relative_eq!(mag.norm(), 1.0, epsilon = 1e-6);
                assert_relative_eq!(ahrs.orientation(), orientation, epsilon = 1e-6);
            }
        }
    }
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_heading_magnetometer_zero_pitch_matches_original_formula(#[case] mut ahrs: AhrsBox) {
    for roll in [-2.0_f32, -0.5, 0.0, 0.8, 2.5] {
        let heading = 63.0_f32.to_radians();
        let expected = Vector3::new(
            libm::cosf(heading),
            -libm::cosf(roll) * libm::sinf(heading),
            libm::sinf(heading) * libm::sinf(roll),
        );
        for yaw in [-1.5, 0.0, 2.0] {
            ahrs.set_orientation(UnitQuaternion::from_euler_angles(roll, 0.0, yaw));
            assert_relative_eq!(ahrs.heading_magnetometer(heading), expected, epsilon = 1e-6);
        }
    }
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_heading_magnetometer_quaternion_sign_invariance(#[case] mut ahrs: AhrsBox) {
    let orientation = UnitQuaternion::from_euler_angles(0.4, -0.7, 1.2);
    let heading = 75.0_f32.to_radians();
    ahrs.set_orientation(orientation);
    let positive = ahrs.heading_magnetometer(heading);
    ahrs.set_orientation(UnitQuaternion::new_unchecked(-orientation.into_inner()));
    assert_relative_eq!(ahrs.heading_magnetometer(heading), positive, epsilon = 1e-6);
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_heading_magnetometer_vertical_threshold(#[case] mut ahrs: AhrsBox) {
    for sign in [-1.0, 1.0] {
        // Test both sides of vertical and both sides of the exclusion threshold.
        for offset in [-0.002, -0.0011, -0.0009, 0.0, 0.0009, 0.0011, 0.002] {
            ahrs.set_orientation(UnitQuaternion::from_euler_angles(
                0.4,
                sign * FRAC_PI_2 + offset,
                0.8,
            ));
            let mag = ahrs.heading_magnetometer(120.0_f32.to_radians());
            assert!(mag.iter().all(|value| value.is_finite()));
            if libm::fabsf(offset) < 0.001 {
                assert_eq!(mag, Vector3::zeros());
            } else {
                assert_relative_eq!(mag.norm(), 1.0, epsilon = 1e-6);
            }
        }
    }
}

#[rstest]
#[case::mahony(mahony_imu(), mahony_imu())]
#[case::madgwick(madgwick_imu(), madgwick_imu())]
#[case::vqf(vqf_imu(), vqf_imu())]
fn test_update_external_heading_matches_manual_update(
    #[case] mut external: AhrsBox,
    #[case] mut manual: AhrsBox,
) {
    let initial = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
    external.set_orientation(initial);
    manual.set_orientation(initial);
    let gyro = Vector3::new(0.01, -0.02, 0.03);
    let accel = initial.inverse_transform_vector(&Vector3::new(0.0, 0.0, 9.81));
    for heading in [
        30.0_f32.to_radians(),
        60.0_f32.to_radians(),
        -45.0_f32.to_radians(),
        100.0_f32.to_radians(),
    ] {
        let mag = manual.heading_magnetometer(heading);
        let expected = manual.update(gyro, accel, mag);
        let actual = external.update_external_heading(gyro, accel, heading);
        assert_relative_eq!(actual, expected, epsilon = 1e-6);
        assert_relative_eq!(external.orientation(), expected, epsilon = 1e-6);
    }
}

#[rstest]
#[case::mahony(mahony_imu(), mahony_imu())]
#[case::madgwick(madgwick_imu(), madgwick_imu())]
#[case::vqf(vqf_imu(), vqf_imu())]
fn test_update_external_heading_vertical_matches_imu_update(
    #[case] mut external: AhrsBox,
    #[case] mut imu: AhrsBox,
) {
    for pitch in [-FRAC_PI_2, FRAC_PI_2] {
        let initial = UnitQuaternion::from_euler_angles(0.4, pitch, 0.8);
        external.set_orientation(initial);
        imu.set_orientation(initial);
        let gyro = Vector3::new(0.01, -0.02, 0.03);
        let accel = initial.inverse_transform_vector(&Vector3::new(0.0, 0.0, 9.81));
        let expected = imu.update_imu(gyro, accel);
        let actual = external.update_external_heading(gyro, accel, 120.0_f32.to_radians());
        assert_relative_eq!(actual, expected, epsilon = 1e-6);
        assert_relative_eq!(external.orientation(), expected, epsilon = 1e-6);
    }
}

#[rstest]
#[case::mahony(mahony_imu(), mahony_imu())]
#[case::madgwick(madgwick_imu(), madgwick_imu())]
#[case::vqf(vqf_imu(), vqf_imu())]
fn test_nonfinite_external_heading_falls_back_to_imu_update(
    #[case] mut external: AhrsBox,
    #[case] mut imu: AhrsBox,
) {
    let initial = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
    external.set_orientation(initial);
    imu.set_orientation(initial);
    let gyro = Vector3::new(0.01, -0.02, 0.03);
    let accel = initial.inverse_transform_vector(&Vector3::new(0.0, 0.0, 9.81));

    for heading in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(external.heading_magnetometer(heading), Vector3::zeros());
        let expected = imu.update_imu(gyro, accel);
        let actual = external.update_external_heading(gyro, accel, heading);
        assert_relative_eq!(actual, expected, epsilon = 1e-6);
        assert_relative_eq!(external.orientation(), expected, epsilon = 1e-6);
    }
}

#[rstest]
#[case::mahony(mahony_imu())]
#[case::madgwick(madgwick_imu())]
#[case::vqf(vqf_imu())]
fn test_update_external_heading_converges_when_tilted(#[case] mut ahrs: AhrsBox) {
    let expected_roll = 0.3;
    let expected_pitch = -0.2;
    let expected_yaw = FRAC_PI_2;
    let true_attitude =
        UnitQuaternion::from_euler_angles(expected_roll, expected_pitch, expected_yaw);

    let gyro = Vector3::zeros();
    let accel = true_attitude.inverse_transform_vector(&Vector3::new(0.0, 0.0, 9.81));

    for _ in 0..UPDATE_STEPS {
        ahrs.update_external_heading(gyro, accel, expected_yaw);
    }

    assert_euler_close(
        ahrs.orientation(),
        expected_roll,
        expected_pitch,
        expected_yaw,
        EPS_UPDATE,
    );
}

#[test]
fn test_update_external_heading_with_dt_matches_manual_update() {
    let dt = Duration::from_millis(20);
    let initial = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
    let gyro = Vector3::new(0.01, -0.02, 0.03);
    let accel = initial.inverse_transform_vector(&Vector3::new(0.0, 0.0, 9.81));
    let heading = 60.0_f32.to_radians();

    let mut mahony_external =
        Mahony::new_with_orientation(IMU_PERIOD, MahonyParams::default(), initial);
    let mut mahony_manual =
        Mahony::new_with_orientation(IMU_PERIOD, MahonyParams::default(), initial);
    let mag = mahony_manual.heading_magnetometer(heading);
    let expected = mahony_manual.update_with_dt(dt, gyro, accel, mag);
    let actual = mahony_external.update_external_heading_with_dt(dt, gyro, accel, heading);
    assert_relative_eq!(actual, expected, epsilon = 1e-6);

    let mut madgwick_external =
        Madgwick::new_with_orientation(IMU_PERIOD, MadgwickParams::default(), initial);
    let mut madgwick_manual =
        Madgwick::new_with_orientation(IMU_PERIOD, MadgwickParams::default(), initial);
    let mag = madgwick_manual.heading_magnetometer(heading);
    let expected = madgwick_manual.update_with_dt(dt, gyro, accel, mag);
    let actual = madgwick_external.update_external_heading_with_dt(dt, gyro, accel, heading);
    assert_relative_eq!(actual, expected, epsilon = 1e-6);
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
fn test_variable_dt_gyro_matches_fixed_period_for_mahony_and_madgwick() {
    let gyro = Vector3::new(0.12, -0.34, 0.56);

    let mut mahony_fixed = Mahony::new(IMU_PERIOD, MahonyParams::default());
    let mut mahony_variable = Mahony::new(IMU_PERIOD, MahonyParams::default());
    mahony_fixed.update_gyro(gyro);
    mahony_variable.update_gyro_with_dt(IMU_PERIOD, gyro);
    assert_relative_eq!(mahony_fixed.orientation(), mahony_variable.orientation());

    let mut madgwick_fixed = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    let mut madgwick_variable = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    madgwick_fixed.update_gyro(gyro);
    madgwick_variable.update_gyro_with_dt(IMU_PERIOD, gyro);
    assert_relative_eq!(
        madgwick_fixed.orientation(),
        madgwick_variable.orientation()
    );
}

#[test]
fn test_variable_dt_changes_gyro_integration() {
    let gyro = Vector3::new(0.0, 0.0, 1.0);

    let mut mahony = Mahony::new(IMU_PERIOD, MahonyParams::default());
    let mut madgwick = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    mahony.update_gyro_with_dt(Duration::from_millis(20), gyro);
    madgwick.update_gyro_with_dt(Duration::from_millis(20), gyro);

    assert_relative_eq!(mahony.orientation().angle(), 0.02, epsilon = 1e-5);
    assert_relative_eq!(madgwick.orientation().angle(), 0.02, epsilon = 1e-5);
}

#[test]
fn test_mahony_bias_integration_uses_variable_dt() {
    let gyro = Vector3::zeros();
    let accel = Vector3::new(1.0, 0.0, 0.0);

    let mut short = Mahony::new(IMU_PERIOD, MahonyParams::default());
    let mut long = Mahony::new(IMU_PERIOD, MahonyParams::default());
    short.update_imu_with_dt(IMU_PERIOD, gyro, accel);
    long.update_imu_with_dt(Duration::from_millis(20), gyro, accel);

    assert_relative_eq!(long.bias, short.bias * 2.0, epsilon = 1e-6);
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
    let params = VqfParams::default();
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
    assert_relative_eq!(
        vqf_with_sensor_rates.orientation(),
        expected,
        epsilon = 1e-6
    );
}

#[test]
fn test_vqf_update2_updates_orientation() {
    let mut vqf = Vqf::new(Duration::from_millis(10), VqfParams::default());
    let initial = vqf.orientation();
    vqf.update2(Vector3::new(0.3, -0.1, 0.2), Vector3::new(0.1, 0.2, 9.8));
    assert!(vqf.orientation().angle_to(&initial) > 0.0);
}

#[test]
fn test_madgwick_set_params_round_trip() {
    let mut ahrs = Madgwick::new(IMU_PERIOD, MadgwickParams::default());
    let params = MadgwickParams { beta: 0.2 };
    ahrs.set_params(params);
    assert_eq!(ahrs.params(), params);
}

#[test]
fn test_mahony_set_params_round_trip() {
    let mut ahrs = Mahony::new(IMU_PERIOD, MahonyParams::default());
    let params = MahonyParams { kp: 2.0, ki: 0.01 };
    ahrs.set_params(params);
    assert_eq!(ahrs.params(), params);
}

#[test]
fn test_madgwick_set_params_takes_effect() {
    // Start tilted 45 deg in roll while the accelerometer says level.
    let tilted = UnitQuaternion::from_euler_angles(FRAC_PI_4, 0.0, 0.0);
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mut ahrs = Madgwick::new_with_orientation(IMU_PERIOD, MadgwickParams { beta: 0.0 }, tilted);

    // With zero gain the accelerometer is ignored: the tilt stays.
    run_imu_steps(&mut ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), FRAC_PI_4, 0.0, 0.0, EPS_GYRO);

    // Raising the gain mid-run lets it converge to level.
    ahrs.set_params(MadgwickParams { beta: 0.5 });
    run_imu_steps(&mut ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.0, EPS_IMU);
}

#[test]
fn test_mahony_set_params_takes_effect() {
    let tilted = UnitQuaternion::from_euler_angles(FRAC_PI_4, 0.0, 0.0);
    let gyro = Vector3::new(0.0, 0.0, 0.0);
    let accel = Vector3::new(0.0, 0.0, 1.0);
    let mut ahrs =
        Mahony::new_with_orientation(IMU_PERIOD, MahonyParams { kp: 0.0, ki: 0.0 }, tilted);

    run_imu_steps(&mut ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), FRAC_PI_4, 0.0, 0.0, EPS_GYRO);

    ahrs.set_params(MahonyParams::default());
    run_imu_steps(&mut ahrs, IMU_STEPS, gyro, accel);
    assert_euler_close(ahrs.orientation(), 0.0, 0.0, 0.0, EPS_IMU);
}
