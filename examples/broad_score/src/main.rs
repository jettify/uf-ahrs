use core::time::Duration;
use glob::glob;
use polars::prelude::*;
use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::path::PathBuf;
use uf_ahrs::Ahrs;
use uf_ahrs::InertialCalibration;
use uf_ahrs::{GyroBiasCalibration, GyroBiasCalibrationParams, GyroBiasCalibrationStatus};
use uf_ahrs::{Madgwick, MadgwickParams};
use uf_ahrs::{Mahony, MahonyParams};
use uf_ahrs::{Vqf, VqfParams};

use itertools::izip;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

pub fn quat_from_acc_mag(acc: &Vector3<f32>, mag: &Vector3<f32>) -> UnitQuaternion<f32> {
    let z = acc;
    let x_temp = z.cross(&-mag);
    let x = x_temp.cross(z);
    let y = z.cross(&x);
    let mat = Matrix3::from_columns(&[x.normalize(), y.normalize(), z.normalize()]);
    UnitQuaternion::from_matrix(&mat)
}

fn total_angle_error(q_diff: &UnitQuaternion<f32>) -> f32 {
    2.0 * q_diff.w.abs().min(1.0).acos()
}

fn heading_error(q_diff: &UnitQuaternion<f32>) -> f32 {
    2.0 * q_diff.k.abs().atan2(q_diff.w.abs())
}

fn inclination_error(q_diff: &UnitQuaternion<f32>) -> f32 {
    2.0 * (q_diff.w.powi(2) + q_diff.k.powi(2)).sqrt().min(1.0).acos()
}

fn calculate_rmse(errors: &[f32]) -> f32 {
    let count = errors.len();
    if count == 0 {
        return 0.0;
    }

    let sum_of_squares: f32 = errors
        .iter()
        .map(|&error_rad| {
            let error_deg = error_rad.to_degrees();
            error_deg * error_deg
        })
        .sum();

    (sum_of_squares / count as f32).sqrt()
}

#[derive(Debug, Copy, Clone)]
struct RmseScore {
    total: f32,
    heading: f32,
    inclination: f32,
}

const CALIBRATION_DURATION: Duration = Duration::from_millis(1500);

/// Uses only the beginning of a recording, before any scored filter updates.
/// A rejected window fails the recording rather than falling back to zero bias.
fn calibrate_startup(
    df: &DataFrame,
    dt: Duration,
) -> Result<(InertialCalibration, usize), Box<dyn Error>> {
    if dt.is_zero() {
        return Err("sample period must be nonzero".into());
    }
    let mut calibrator = GyroBiasCalibration::new(GyroBiasCalibrationParams {
        duration: CALIBRATION_DURATION,
        ..Default::default()
    })?;
    let x = df.column("imu_gyr_x")?.cast(&DataType::Float32)?;
    let y = df.column("imu_gyr_y")?.cast(&DataType::Float32)?;
    let z = df.column("imu_gyr_z")?.cast(&DataType::Float32)?;
    for (index, (x, y, z)) in izip!(x.f32()?.iter(), y.f32()?.iter(), z.f32()?.iter()).enumerate() {
        let gyro = Vector3::new(
            x.ok_or("missing startup gyro x reading")?,
            y.ok_or("missing startup gyro y reading")?,
            z.ok_or("missing startup gyro z reading")?,
        );
        match calibrator.update(dt, gyro) {
            GyroBiasCalibrationStatus::Collecting => {}
            GyroBiasCalibrationStatus::Complete(offset) => {
                return Ok((
                    InertialCalibration {
                        offset,
                        ..Default::default()
                    },
                    index + 1,
                ));
            }
            GyroBiasCalibrationStatus::Rejected(reason) => {
                return Err(format!("startup gyro calibration rejected: {reason:?}; statistics: {:?}", calibrator.rejected_statistics()).into());
            }
            GyroBiasCalibrationStatus::InvalidSample => {
                return Err("startup gyro calibration encountered invalid data".into());
            }
        }
    }
    Err("recording is too short for startup gyro calibration".into())
}

fn evaluate_dataset(
    df: &DataFrame,
    ahrs: &mut (impl Ahrs + ?Sized),
    gyro_calibration: &InertialCalibration,
) -> Result<DataFrame, Box<dyn Error>> {
    let gyr_x = df.column("imu_gyr_x")?.cast(&DataType::Float32)?;
    let gyr_y = df.column("imu_gyr_y")?.cast(&DataType::Float32)?;
    let gyr_z = df.column("imu_gyr_z")?.cast(&DataType::Float32)?;

    let acc_x = df.column("imu_acc_x")?.cast(&DataType::Float32)?;
    let acc_y = df.column("imu_acc_y")?.cast(&DataType::Float32)?;
    let acc_z = df.column("imu_acc_z")?.cast(&DataType::Float32)?;

    let mag_x = df.column("imu_mag_x")?.cast(&DataType::Float32)?;
    let mag_y = df.column("imu_mag_y")?.cast(&DataType::Float32)?;
    let mag_z = df.column("imu_mag_z")?.cast(&DataType::Float32)?;

    let opt_quat_w = df.column("opt_quat_w")?.cast(&DataType::Float32)?;
    let opt_quat_x = df.column("opt_quat_x")?.cast(&DataType::Float32)?;
    let opt_quat_y = df.column("opt_quat_y")?.cast(&DataType::Float32)?;
    let opt_quat_z = df.column("opt_quat_z")?.cast(&DataType::Float32)?;
    let movement = df.column("movement")?.cast(&DataType::Float32)?;

    let gyr_x = gyr_x.f32()?;
    let gyr_y = gyr_y.f32()?;
    let gyr_z = gyr_z.f32()?;

    let acc_x = acc_x.f32()?;
    let acc_y = acc_y.f32()?;
    let acc_z = acc_z.f32()?;

    let mag_x = mag_x.f32()?;
    let mag_y = mag_y.f32()?;
    let mag_z = mag_z.f32()?;

    let mut quat_w_values = Vec::new();
    let mut quat_x_values = Vec::new();
    let mut quat_y_values = Vec::new();
    let mut quat_z_values = Vec::new();

    let init_acc = Vector3::new(
        acc_x
            .get(0)
            .ok_or("missing initial accelerometer x reading")?,
        acc_y
            .get(0)
            .ok_or("missing initial accelerometer y reading")?,
        acc_z
            .get(0)
            .ok_or("missing initial accelerometer z reading")?,
    );
    let init_mag = Vector3::new(
        mag_x
            .get(0)
            .ok_or("missing initial magnetometer x reading")?,
        mag_y
            .get(0)
            .ok_or("missing initial magnetometer y reading")?,
        mag_z
            .get(0)
            .ok_or("missing initial magnetometer z reading")?,
    );

    let init_q = quat_from_acc_mag(&init_acc, &init_mag);
    ahrs.set_orientation(init_q);

    for (
        gyr_x_val,
        gyr_y_val,
        gyr_z_val,
        acc_x_val,
        acc_y_val,
        acc_z_val,
        mag_x_val,
        mag_y_val,
        mag_z_val,
    ) in izip!(
        gyr_x.iter(),
        gyr_y.iter(),
        gyr_z.iter(),
        acc_x.iter(),
        acc_y.iter(),
        acc_z.iter(),
        mag_x.iter(),
        mag_y.iter(),
        mag_z.iter(),
    ) {
        if let (
            Some(gx),
            Some(gy),
            Some(gz),
            Some(ax),
            Some(ay),
            Some(az),
            Some(mx),
            Some(my),
            Some(mz),
        ) = (
            gyr_x_val, gyr_y_val, gyr_z_val, acc_x_val, acc_y_val, acc_z_val, mag_x_val, mag_y_val,
            mag_z_val,
        ) {
            let gyro = gyro_calibration.calibrate(Vector3::new(gx, gy, gz));
            let acc = Vector3::new(ax, ay, az);
            let mag = Vector3::new(mx, my, mz);

            ahrs.update(gyro, acc, mag);
            let quaternion = ahrs.orientation();

            quat_w_values.push(quaternion.w);
            quat_x_values.push(quaternion.i);
            quat_y_values.push(quaternion.j);
            quat_z_values.push(quaternion.k);
        } else {
            return Err("missing IMU reading in evaluation samples".into());
        }
    }

    let s_comp_w = Series::new("computed_quat_w".into(), quat_w_values);
    let s_comp_x = Series::new("computed_quat_x".into(), quat_x_values);
    let s_comp_y = Series::new("computed_quat_y".into(), quat_y_values);
    let s_comp_z = Series::new("computed_quat_z".into(), quat_z_values);

    let result_df = DataFrame::new(
        s_comp_w.len(),
        vec![
            s_comp_w.into(),
            s_comp_x.into(),
            s_comp_y.into(),
            s_comp_z.into(),
            opt_quat_w.clone(),
            opt_quat_x.clone(),
            opt_quat_y.clone(),
            opt_quat_z.clone(),
            movement.clone(),
        ],
    )?;

    Ok(result_df)
}

fn analyze_results(df: &DataFrame) -> Result<RmseScore, Box<dyn Error>> {
    let mut diff_angle = Vec::new();
    let mut heading_errors = Vec::new();
    let mut inclination_errors = Vec::new();

    let to_enu = UnitQuaternion::new_normalize(Quaternion::new(
        1.0f32 / f32::sqrt(2.0f32),
        0.0f32,
        0.0f32,
        1.0f32 / f32::sqrt(2.0f32),
    ));

    let computed_w = df.column("computed_quat_w")?.f32()?;
    let computed_x = df.column("computed_quat_x")?.f32()?;
    let computed_y = df.column("computed_quat_y")?.f32()?;
    let computed_z = df.column("computed_quat_z")?.f32()?;

    let opt_w = df.column("opt_quat_w")?.f32()?;
    let opt_x = df.column("opt_quat_x")?.f32()?;
    let opt_y = df.column("opt_quat_y")?.f32()?;
    let opt_z = df.column("opt_quat_z")?.f32()?;
    let movement = df.column("movement")?.f32()?;

    for (comp_w, comp_x, comp_y, comp_z, opt_w, opt_x, opt_y, opt_z, mov) in izip!(
        computed_w.iter(),
        computed_x.iter(),
        computed_y.iter(),
        computed_z.iter(),
        opt_w.iter(),
        opt_x.iter(),
        opt_y.iter(),
        opt_z.iter(),
        movement.iter()
    ) {
        if let (
            Some(comp_w),
            Some(comp_x),
            Some(comp_y),
            Some(comp_z),
            Some(opt_w),
            Some(opt_x),
            Some(opt_y),
            Some(opt_z),
            Some(mov),
        ) = (
            comp_w, comp_x, comp_y, comp_z, opt_w, opt_x, opt_y, opt_z, mov,
        ) && mov > 0.5
        {
            let imu_quat_ned =
                UnitQuaternion::new_normalize(Quaternion::new(comp_w, comp_x, comp_y, comp_z));
            let imu_quat_enu = to_enu * imu_quat_ned;

            let opt_quat =
                UnitQuaternion::new_normalize(Quaternion::new(opt_w, opt_x, opt_y, opt_z));

            let q_diff_earth = imu_quat_enu * opt_quat.inverse();
            diff_angle.push(total_angle_error(&q_diff_earth));
            heading_errors.push(heading_error(&q_diff_earth));
            inclination_errors.push(inclination_error(&q_diff_earth));
        }
    }

    let rmse = calculate_rmse(&diff_angle);
    let rmse_heading = calculate_rmse(&heading_errors);
    let rmse_inclination = calculate_rmse(&inclination_errors);

    Ok(RmseScore {
        total: rmse,
        heading: rmse_heading,
        inclination: rmse_inclination,
    })
}

#[derive(Debug, Copy, Clone)]
enum Algorithm {
    Madgwick,
    Mahony,
    Vqf,
}

fn make_ahrs(algorithm: Algorithm, sample_period: Duration) -> Box<dyn Ahrs> {
    match algorithm {
        Algorithm::Mahony => Box::new(Mahony::new(sample_period, MahonyParams::default())),
        Algorithm::Madgwick => Box::new(Madgwick::new(sample_period, MadgwickParams::default())),
        Algorithm::Vqf => {
            let params = VqfParams::default();
            Box::new(Vqf::new(sample_period, params))
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut paths: Vec<PathBuf> = glob("data/*.parquet")?.filter_map(Result::ok).collect();
    paths.sort();

    let dt = Duration::from_secs_f32(0.0035);

    let algorithms_to_test = [
        ("Mahony", Algorithm::Mahony),
        ("Madgwick", Algorithm::Madgwick),
        ("Vqf", Algorithm::Vqf),
    ];

    let mut all_scores: HashMap<(&str, bool), Vec<RmseScore>> = HashMap::new();

    if paths.is_empty() {
        return Err(
            "no BROAD recordings found in data/*.parquet; run from examples/broad_score".into(),
        );
    }
    for entry in &paths {
        let df = ParquetReader::new(File::open(entry)?).finish()?;
        let (gyro_calibration, startup_samples) =
            calibrate_startup(&df, dt).map_err(|error| format!("{}: {error}", entry.display()))?;
        if startup_samples >= df.height() {
            return Err(format!("{}: no samples remain after calibration", entry.display()).into());
        }
        // Slice sensor readings and reference quaternions together so that
        // calibration samples never contribute to scored errors.
        let start = i64::try_from(startup_samples)?;
        let evaluation_df = df.slice(start, df.height() - startup_samples);
        println!(
            "{}: calibrated {} samples ({:.4} s), bias (rad/s) = {:?}",
            entry.display(),
            startup_samples,
            startup_samples as f64 * dt.as_secs_f64(),
            gyro_calibration.offset,
        );
        for (name, algo) in algorithms_to_test {
            for calibration_enabled in [false, true] {
                let correction = if calibration_enabled {
                    gyro_calibration
                } else {
                    InertialCalibration::default()
                };
                // Both modes use the same post-startup rows and initial
                // orientation, with all filter state reset between runs.
                let mut ahrs = make_ahrs(algo, dt);
                let result_df = evaluate_dataset(&evaluation_df, ahrs.as_mut(), &correction)?;
                let score = analyze_results(&result_df)?;
                all_scores
                    .entry((name, calibration_enabled))
                    .or_default()
                    .push(score);
            }
        }
    }

    for (name, _) in algorithms_to_test {
        for calibration_enabled in [false, true] {
            let scores = all_scores
                .get(&(name, calibration_enabled))
                .ok_or("missing comparison scores")?;
            let count = scores.len() as f32;
            let mean_total = scores.iter().map(|s| s.total).sum::<f32>() / count;
            let mean_heading = scores.iter().map(|s| s.heading).sum::<f32>() / count;
            let mean_inclination = scores.iter().map(|s| s.inclination).sum::<f32>() / count;
            let mode = if calibration_enabled { "on" } else { "off" };
            println!(
                "Algorithm: {name} (startup calibration {mode}, {} recordings)",
                scores.len()
            );
            println!("  AVG Total RMSE:       {mean_total:.4} deg");
            println!("  AVG Heading RMSE:     {mean_heading:.4} deg");
            println!("  AVG Inclination RMSE: {mean_inclination:.4} deg");
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recording(gyro: Vec<Vector3<f32>>) -> DataFrame {
        let n = gyro.len();
        DataFrame::new(
            n,
            vec![
                Series::new(
                    "imu_gyr_x".into(),
                    gyro.iter().map(|v| v.x).collect::<Vec<_>>(),
                )
                .into(),
                Series::new(
                    "imu_gyr_y".into(),
                    gyro.iter().map(|v| v.y).collect::<Vec<_>>(),
                )
                .into(),
                Series::new(
                    "imu_gyr_z".into(),
                    gyro.iter().map(|v| v.z).collect::<Vec<_>>(),
                )
                .into(),
                Series::new("imu_acc_x".into(), vec![0.0_f32; n]).into(),
                Series::new("imu_acc_y".into(), vec![0.0_f32; n]).into(),
                Series::new("imu_acc_z".into(), vec![9.81_f32; n]).into(),
                Series::new("imu_mag_x".into(), vec![0.0_f32; n]).into(),
                Series::new("imu_mag_y".into(), vec![1.0_f32; n]).into(),
                Series::new("imu_mag_z".into(), vec![0.0_f32; n]).into(),
                // Distinct references let the test detect accidental row shifts.
                Series::new(
                    "opt_quat_w".into(),
                    (0..n).map(|i| i as f32).collect::<Vec<_>>(),
                )
                .into(),
                Series::new("opt_quat_x".into(), vec![0.0_f32; n]).into(),
                Series::new("opt_quat_y".into(), vec![0.0_f32; n]).into(),
                Series::new("opt_quat_z".into(), vec![0.0_f32; n]).into(),
                Series::new("movement".into(), vec![1.0_f32; n]).into(),
            ],
        )
        .unwrap()
    }

    #[derive(Default)]
    struct RecordingAhrs {
        orientation: UnitQuaternion<f32>,
        gyro: Vec<Vector3<f32>>,
    }

    impl Ahrs for RecordingAhrs {
        fn orientation(&self) -> UnitQuaternion<f32> {
            self.orientation
        }
        fn set_orientation(&mut self, quat: UnitQuaternion<f32>) {
            self.orientation = quat;
        }
        fn update(
            &mut self,
            gyro: Vector3<f32>,
            _acc: Vector3<f32>,
            _mag: Vector3<f32>,
        ) -> UnitQuaternion<f32> {
            self.gyro.push(gyro);
            self.orientation
        }
        fn update_imu(&mut self, gyro: Vector3<f32>, acc: Vector3<f32>) -> UnitQuaternion<f32> {
            self.update(gyro, acc, Vector3::zeros())
        }
        fn update_gyro(&mut self, gyro: Vector3<f32>) -> UnitQuaternion<f32> {
            self.update(gyro, Vector3::zeros(), Vector3::zeros())
        }
    }

    #[test]
    fn startup_bias_uses_only_initial_window_and_corrects_following_samples() {
        let bias = Vector3::new(0.02, -0.01, 0.03);
        let rotation = Vector3::new(0.25, -0.5, 1.0);
        let mut gyro = vec![bias; 150];
        gyro.extend([bias + rotation, bias - rotation]);
        let df = recording(gyro);
        let (calibration, start) = calibrate_startup(&df, Duration::from_millis(10)).unwrap();
        assert_eq!(start, 150);
        assert_eq!(calibration.offset, bias);
        let evaluation = df.slice(start as i64, df.height() - start);
        let mut filter = RecordingAhrs::default();
        let result = evaluate_dataset(&evaluation, &mut filter, &calibration).unwrap();
        assert_eq!(filter.gyro, [rotation, -rotation]);
        let mut uncalibrated = RecordingAhrs::default();
        let uncalibrated_result = evaluate_dataset(
            &evaluation,
            &mut uncalibrated,
            &InertialCalibration::default(),
        )
        .unwrap();
        assert_eq!(uncalibrated.gyro, [bias + rotation, bias - rotation]);
        assert_eq!(
            uncalibrated_result.column("opt_quat_w").unwrap(),
            result.column("opt_quat_w").unwrap()
        );
        assert_eq!(result.height(), 2);
        assert_eq!(
            result.column("opt_quat_w").unwrap().f32().unwrap().get(0),
            Some(150.0)
        );
        assert_eq!(
            result.column("opt_quat_w").unwrap().f32().unwrap().get(1),
            Some(151.0)
        );
    }

    #[test]
    fn startup_calibration_rejects_short_invalid_or_moving_recordings() {
        let dt = Duration::from_millis(10);
        let mut gyro = vec![Vector3::zeros(); 149];
        assert!(calibrate_startup(&recording(gyro.clone()), dt).is_err());
        gyro.push(Vector3::zeros());
        assert!(calibrate_startup(&recording(gyro.clone()), Duration::ZERO).is_err());
        gyro[10] = Vector3::repeat(0.1);
        assert!(calibrate_startup(&recording(gyro.clone()), dt).is_err());
        gyro[10] = Vector3::repeat(f32::NAN);
        assert!(calibrate_startup(&recording(gyro), dt).is_err());
    }
}
