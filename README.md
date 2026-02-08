# uf-ahrs

[![CI](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-ahrs/graph/badge.svg?token=2N16CN1OZX)](https://codecov.io/gh/jettify/uf-ahrs)

`uf-ahrs` rust `no_std` library for orientation estimation using gyroscope, accelerometer and magnetometer.

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

```rust
use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParameters};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);

    let mut mahony = Mahony::new(dt, MahonyParams::default());
    let mut madgwick = Madgwick::new(dt, MadgwickParams::default());
    let mut vqf = Vqf::new(dt, VqfParameters::default());

    // Sensor data
    let gyr = Vector3::new(0.0, 0.0, 0.0);
    let acc = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(20.0, 0.0, 0.0);

    mahony.update(gyr, acc, mag);
    madgwick.update(gyr, acc, mag);
    vqf.update(gyr, acc, mag);

    // Get orientation as UnitQuaternion
    let q_mahony = mahony.orientation();
    let q_madgwick = madgwick.orientation();
    let q_vqf = vqf.orientation();

    println!("Mahony:   {:?}", q_mahony.euler_angles());
    println!("Madgwick: {:?}", q_madgwick.euler_angles());
    println!("VQF:      {:?}", q_vqf.euler_angles());
}
```

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

## License

This project is licensed under the `Apache 2.0`. See the [LICENSE](https://github.com/jettify/uf-crsf/blob/master/LICENSE) file for details.

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

- **vqf-rs**: alternative VQF filter implementation in Rust.
  [https://github.com/vgskye/vqf-rs](https://github.com/vgskye/vqf-rs)

- **vqf**: another VQF filter implementation in Rust.
  [https://github.com/oxkitsune/vqf](https://github.com/oxkitsune/vqf)

- **broad**: Berlin Robust Orientation Estimation Assessment Dataset (BROAD).
  [https://github.com/dlaidig/broad](https://github.com/dlaidig/broad)
