use core::f32;
use glob::glob;
use polars::prelude::*;
use std::error::Error;
use std::fs::File;
use std::path::Path;
use uf_ahrs::Ahrs;
use uf_ahrs::Mahony;
use uf_ahrs::quat_from_acc_mag;

use itertools::izip;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn evaluate_dataset(path: impl AsRef<Path>) -> Result<f32, Box<dyn Error>> {
    let file = File::open(path).unwrap();
    let df = ParquetReader::new(file).finish()?;

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
    let dt = df.column("sampling_rate")?.cast(&DataType::Float32)?;
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

    let opt_quat_w = opt_quat_w.f32()?;
    let opt_quat_x = opt_quat_x.f32()?;
    let opt_quat_y = opt_quat_y.f32()?;
    let opt_quat_z = opt_quat_z.f32()?;
    let dt = dt.f32()?;
    let movement = movement.f32()?;

    let mut quat_w_values = Vec::new();
    let mut quat_x_values = Vec::new();
    let mut quat_y_values = Vec::new();
    let mut quat_z_values = Vec::new();
    let mut diff_angle = Vec::new();
    let mut heading_errors = Vec::new();
    let mut inclination_errors = Vec::new();

    let sample_rate = 1.0 / dt.get(0).unwrap();

    let to_enu = UnitQuaternion::new_normalize(Quaternion::new(
        1.0f32 / f32::sqrt(2.0f32),
        0.0f32,
        0.0f32,
        1.0f32 / f32::sqrt(2.0f32),
    ));

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
    let mut ahrs = Mahony::new_with_quaternion(sample_rate, 1.95, 0.004, init_q);
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
        opt_quat_w_val,
        opt_quat_x_val,
        opt_quat_y_val,
        opt_quat_z_val,
        movement_val,
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
        opt_quat_w.into_iter(),
        opt_quat_x.into_iter(),
        opt_quat_y.into_iter(),
        opt_quat_z.into_iter(),
        movement.into_iter()
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
            Some(mov),
        ) = (
            gyr_x_val,
            gyr_y_val,
            gyr_z_val,
            acc_x_val,
            acc_y_val,
            acc_z_val,
            mag_x_val,
            mag_y_val,
            mag_z_val,
            movement_val,
        ) {
            let gyro = Vector3::new(gx, gy, gz);
            let acc = Vector3::new(ax, ay, az);
            let mag = Vector3::new(mx, my, mz);

            let opt_quat = to_enu.inverse()
                * UnitQuaternion::new_normalize(Quaternion::new(
                    opt_quat_w_val.unwrap_or(1.0),
                    opt_quat_x_val.unwrap_or(0.0),
                    opt_quat_y_val.unwrap_or(0.0),
                    opt_quat_z_val.unwrap_or(0.0),
                ));

            ahrs.update(&gyro, &acc, &mag);
            let quaternion = ahrs.quaternion;

            if mov > 0.5 {
                let diff = opt_quat.angle_to(&quaternion);
                diff_angle.push(diff);

                let q_diff = opt_quat.inverse() * quaternion;
                let q_diff_earth = to_enu * q_diff * to_enu.inverse();

                let heading = 2.0 * q_diff_earth.k.abs().atan2(q_diff_earth.w.abs());
                heading_errors.push(heading);

                let inclination = 2.0
                    * (q_diff_earth.w.powi(2) + q_diff_earth.k.powi(2))
                        .sqrt()
                        .min(1.0)
                        .acos();
                inclination_errors.push(inclination);
            }

            quat_w_values.push(quaternion.w);
            quat_x_values.push(quaternion.i);
            quat_y_values.push(quaternion.j);
            quat_z_values.push(quaternion.k);
        }
    }

    let s = diff_angle.len();
    let result_df = df!(
        "computed_quat_w" => quat_w_values,
        "computed_quat_x" => quat_x_values,
        "computed_quat_y" => quat_y_values,
        "computed_quat_z" => quat_z_values,
        "opt_quat_w" => opt_quat_w.to_vec(),
        "opt_quat_x" => opt_quat_x.to_vec(),
        "opt_quat_y" => opt_quat_y.to_vec(),
        "opt_quat_z" => opt_quat_z.to_vec(),
    )?;
    // TODO: save result_df to csv
    let rmse = f32::sqrt(
        diff_angle
            .iter()
            .map(|v| v.to_degrees())
            .map(|x| x * x)
            .sum::<f32>()
            / (s as f32),
    );

    let s_heading = heading_errors.len();
    let rmse_heading = f32::sqrt(
        heading_errors
            .iter()
            .map(|v| v.to_degrees())
            .map(|x| x * x)
            .sum::<f32>()
            / (s_heading as f32),
    );

    let s_inclination = inclination_errors.len();
    let rmse_inclination = f32::sqrt(
        inclination_errors
            .iter()
            .map(|v| v.to_degrees())
            .map(|x| x * x)
            .sum::<f32>()
            / (s_inclination as f32),
    );

    println!("\nRMSE Total: {} deg", rmse);
    println!("RMSE Heading: {} deg", rmse_heading);
    println!("RMSE Inclination: {} deg", rmse_inclination);

    Ok(rmse)
}

fn main() -> Result<(), Box<dyn Error>> {
    for entry in glob("data/*.parquet").unwrap() {
        match entry {
            Ok(path) => {
                print!("Evaluating {:?}:", &path.display());
                evaluate_dataset(&path).unwrap();
            }
            Err(e) => println!("{:?}", e),
        }
    }
    Ok(())
}
