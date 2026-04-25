# uf-ahrs

[![CI](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-ahrs/graph/badge.svg?token=NFUQBCTUXF)](https://codecov.io/gh/jettify/uf-ahrs)
[![crates.io](https://img.shields.io/crates/v/uf-ahrs)](https://crates.io/crates/uf-ahrs)
[![docs.rs](https://img.shields.io/docsrs/uf-ahrs)](https://docs.rs/uf-ahrs/latest/uf_ahrs/)

`uf-ahrs` rust `no_std` library for orientation estimation using gyroscope, accelerometer and magnetometer.

## Demo

<video src="https://github.com/user-attachments/assets/d1891b70-1a44-4a4e-acee-5aacc891a5e4"></video>

[Direct video link](https://raw.githubusercontent.com/jettify/uf-ahrs/master/docs/clip.mp4)

## Features

* `no_std` and allocator-free for embedded systems.
* IO and MCU agnostic.
* Implements `Mahony`, `Madgwick` and `VQF` filters.
* Filters evaluated using `BROAD` dataset.

## Note

Library is under active development and testing, API might change at any time.

## Installation

Add `uf-ahrs` to your `Cargo.toml`:

```toml
[dependencies]
uf-ahrs = "*" # replace * by the latest version of the crate.
```

Or use the command line:

```bash
cargo add uf-ahrs
```

## Usage

Here is a simple example showing how to initialize and use `Mahony`, `Madgwick`, and `VQF` filters.

> **Note on units**
>
> - Gyroscope data must be in **rad/s**
> - Accelerometer and magnetometer units do not matter, as long as they are consistent.
>   The filters normalize these vectors internally.

```rust
use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParams};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);

    let mut mahony = Mahony::new(dt, MahonyParams::default());
    let mut madgwick = Madgwick::new(dt, MadgwickParams::default());
    let mut vqf = Vqf::new(dt, VqfParams::default());

    // Sensor data
    let gyr = Vector3::new(0.0, 0.0, 0.0);     // rad/s
    let acc = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(20.0, 0.0, 0.0);

    mahony.update(gyr, acc, mag);
    madgwick.update(gyr, acc, mag);
    vqf.update(gyr, acc, mag);

    // Get orientation as UnitQuaternion
    let q_mahony = mahony.orientation();
    let q_madgwick = madgwick.orientation();
    let q_vqf = vqf.orientation();

    // std::println used  only for example purposes, library itself
    // is fully no_std compatible.
    println!("Mahony:   {:?}", q_mahony.euler_angles());
    println!("Madgwick: {:?}", q_madgwick.euler_angles());
    println!("VQF:      {:?}", q_vqf.euler_angles());
}
```

## Benchmark Scores (BROAD)

The results below are from evaluations on the **Berlin Robust Orientation Estimation Assessment Dataset (BROAD)**.
For context, BROAD contains **39 trials** with synchronized IMU and optical ground-truth orientation:
**23 undisturbed trials** (rotation, translation, and combined motion at slow/fast speeds) and
**16 disturbed trials** (e.g., tapping, vibration, stationary/attached magnets, office environment, and mixed conditions).
Dataset publication: [https://www.mdpi.com/2306-5729/6/7/72](https://www.mdpi.com/2306-5729/6/7/72).

Reported metrics are dataset-averaged orientation errors in degrees:
`AVG Total RMSE`, `AVG Heading RMSE`, and `AVG Inclination RMSE`.

```bash no_run
Algorithm: Vqf
  AVG Total RMSE:       2.0776 deg
  AVG Heading RMSE:     1.9205 deg
  AVG Inclination RMSE: 0.6963 deg
Algorithm: Mahony
  AVG Total RMSE:       7.4884 deg
  AVG Heading RMSE:     6.2993 deg
  AVG Inclination RMSE: 3.6971 deg
Algorithm: Madgwick
  AVG Total RMSE:       5.0831 deg
  AVG Heading RMSE:     4.3588 deg
  AVG Inclination RMSE: 2.3404 deg
```

## Embedded Performance

Measured on an `STM32F411CEUx` Black Pill using the DWT cycle counter. The
benchmark configures the chip to run at `100 MHz`.

| Filter | Update | Cycles/update | Time/update at 100 MHz |
| --- | --- | ---: | ---: |
| Mahony | IMU | 638 | 6.38 us |
| Mahony | IMU + magnetometer | 1029 | 10.29 us |
| Madgwick | IMU | 817 | 8.17 us |
| Madgwick | IMU + magnetometer | 1299 | 12.99 us |
| VQF | IMU | 4413 | 44.13 us |
| VQF | IMU + magnetometer | 10282 | 102.82 us |

Following numbers are from `cargo size --release --no-default-features` with `defmt`
disabled, flash is `text + data`, and the delta is measured relative to `size-base`.
Static RAM is `data + bss` reported by `cargo size`; it does not include the runtime stack.

| Experiment | Flash total | Flash delta vs base | Static RAM |
| --- | ---: | ---: | ---: |
| Base firmware | 2_556 B | - | 4 B |
| Mahony IMU | 3_884 B | 1_328 B | 4 B |
| Mahony IMU + magnetometer | 4_616 B | 2_060 B | 4 B |
| Madgwick IMU | 4_060 B | 1_504 B | 4 B |
| Madgwick IMU + magnetometer | 5_064 B | 2_508 B | 4 B |
| VQF IMU | 28_844 B | 26_288 B | 4 B |
| VQF IMU + magnetometer | 32_356 B | 29_800 B | 4 B |


## License

This project is licensed under the `Apache 2.0`. See the [LICENSE](https://github.com/jettify/uf-ahrs/blob/master/LICENSE) file for details.

## References

This library incorporates ideas and data from the following projects and publications.

### Publications

- _**BROAD - A benchmark for robust inertial orientation estimation**_
  D. Laidig, M. Caruso, A. Cereatti, and T. Seel, *Data*, vol. 6, no. 7, p. 72, 2021.

- _**Nonlinear complementary filters on the special orthogonal group**_
  R. Mahony, T. Hamel, and J.-M. Pflimlin, *IEEE Transactions on Automatic Control*, vol. 53, no. 5, pp. 1203–1218, 2008.

- _**An efficient orientation filter for inertial and inertial/magnetic sensor arrays**_
  S. O. H. Madgwick et al., 2010.

- _**VQF: Highly accurate IMU orientation estimation with bias estimation and magnetic disturbance rejection**_
  D. Laidig and T. Seel, *Information Fusion*, vol. 91, pp. 187–204, 2023.

### Related Projects

- **ahrs-rs**: alternative Mahony and Madgwick filter implementation in Rust.
  [https://github.com/jmagnuson/ahrs-rs](https://github.com/jmagnuson/ahrs-rs)

- **vqf**: Original VQF implementation in Matlab and C++.
  [https://github.com/dlaidig/vqf](https://github.com/dlaidig/vqf)

- **vqf-rs**: alternative VQF filter implementation in Rust.
  [https://github.com/vgskye/vqf-rs](https://github.com/vgskye/vqf-rs)

- **vqf**: another VQF filter implementation in Rust.
  [https://github.com/oxkitsune/vqf](https://github.com/oxkitsune/vqf)

- **broad**: Berlin Robust Orientation Estimation Assessment Dataset (BROAD).
  [https://github.com/dlaidig/broad](https://github.com/dlaidig/broad)
