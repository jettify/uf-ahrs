use core::f32;
use core::time::Duration;
use glob::glob;
use polars::prelude::*;
use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::path::PathBuf;
use uf_ahrs::Ahrs;
use uf_ahrs::{Madgwick, MadgwickParams};
use uf_ahrs::{Mahony, MahonyParams};
use uf_ahrs::{Vqf, VqfParameters};

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

fn evaluate_dataset(
    df: &DataFrame,
    ahrs: &mut (impl Ahrs + ?Sized),
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
        acc_x.get(0).unwrap(),
        acc_y.get(0).unwrap(),
        acc_z.get(0).unwrap(),
    );
    let init_mag = Vector3::new(
        mag_x.get(0).unwrap(),
        mag_y.get(0).unwrap(),
        mag_z.get(0).unwrap(),
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
        gyr_x.into_iter(),
        gyr_y.into_iter(),
        gyr_z.into_iter(),
        acc_x.into_iter(),
        acc_y.into_iter(),
        acc_z.into_iter(),
        mag_x.into_iter(),
        mag_y.into_iter(),
        mag_z.into_iter(),
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
            let gyro = Vector3::new(gx, gy, gz);
            let acc = Vector3::new(ax, ay, az);
            let mag = Vector3::new(mx, my, mz);

            ahrs.update(gyro, acc, mag);
            //ahrs.update_imu(gyro, acc);
            let quaternion = ahrs.orientation();

            quat_w_values.push(quaternion.w);
            quat_x_values.push(quaternion.i);
            quat_y_values.push(quaternion.j);
            quat_z_values.push(quaternion.k);
        }
    }

    let s_comp_w = Series::new("computed_quat_w".into(), quat_w_values);
    let s_comp_x = Series::new("computed_quat_x".into(), quat_x_values);
    let s_comp_y = Series::new("computed_quat_y".into(), quat_y_values);
    let s_comp_z = Series::new("computed_quat_z".into(), quat_z_values);

    let result_df = DataFrame::new(vec![
        s_comp_w.into(),
        s_comp_x.into(),
        s_comp_y.into(),
        s_comp_z.into(),
        opt_quat_w.clone(),
        opt_quat_x.clone(),
        opt_quat_y.clone(),
        opt_quat_z.clone(),
        movement.clone(),
    ])?;

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
        computed_w.into_iter(),
        computed_x.into_iter(),
        computed_y.into_iter(),
        computed_z.into_iter(),
        opt_w.into_iter(),
        opt_x.into_iter(),
        opt_y.into_iter(),
        opt_z.into_iter(),
        movement.into_iter()
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
            let params = VqfParameters::default();
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

    let mut all_scores: HashMap<&str, Vec<RmseScore>> = HashMap::new();

    for (name, algo) in algorithms_to_test {
        println!("Evaluating {}...", name);
        let mut ahrs = make_ahrs(algo, dt);
        let mut scores = Vec::new();
        for entry in &paths {
            let file = File::open(entry)?;
            let df = ParquetReader::new(file).finish()?;
            let result_df = evaluate_dataset(&df, ahrs.as_mut())?;
            let score = analyze_results(&result_df)?;
            scores.push(score);
        }
        all_scores.insert(name, scores);
    }

    for (name, scores) in &all_scores {
        let count = scores.len();
        if count == 0 {
            continue;
        }
        let mean_total: f32 = scores.iter().map(|s| s.total).sum::<f32>() / (count as f32);
        let mean_heading: f32 = scores.iter().map(|s| s.heading).sum::<f32>() / (count as f32);
        let mean_inclination: f32 =
            scores.iter().map(|s| s.inclination).sum::<f32>() / (count as f32);

        println!("Algorithm: {}", name);
        println!("  AVG Total RMSE:       {:.4} deg", mean_total);
        println!("  AVG Heading RMSE:     {:.4} deg", mean_heading);
        println!("  AVG Inclination RMSE: {:.4} deg", mean_inclination);
    }

    Ok(())
}
