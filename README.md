# uf-ahrs

[![CI](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-ahrs/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-ahrs/graph/badge.svg?token=NFUQBCTUXF)](https://codecov.io/gh/jettify/uf-ahrs)
[![crates.io](https://img.shields.io/crates/v/uf-ahrs)](https://crates.io/crates/uf-ahrs)
[![docs.rs](https://img.shields.io/docsrs/uf-ahrs)](https://docs.rs/uf-ahrs/latest/uf_ahrs/)

`uf-ahrs` is an allocator-free, `no_std` Rust library for attitude and heading estimation (AHRS) in embedded systems, robotics, and flight controllers. It provides Mahony, Madgwick, and VQF sensor fusion filters with a shared API and `nalgebra` quaternion output, supporting gyroscope and accelerometer data with optional magnetometer or external heading input.

## Demo

<video src="https://github.com/user-attachments/assets/d1891b70-1a44-4a4e-acee-5aacc891a5e4"></video>

[Direct video link](https://raw.githubusercontent.com/jettify/uf-ahrs/master/docs/clip.mp4)

## Features

* `no_std` and allocator-free, with no dependency on a specific MCU or sensor driver.
* `Mahony`, `Madgwick`, and `Vqf` filters behind a shared `Ahrs` interface.
* `Mahony`, `Madgwick` support `Vqf` style accelerometer filtering that improve RMSE 2x.
* Gyroscope-only, IMU (gyroscope + accelerometer), and IMU + magnetometer updates.
* External heading input for heading correction without a magnetometer.
* VQF gyroscope bias estimation during motion and rest, plus magnetic disturbance rejection.
* Sensor axis remapping, inertial and magnetometer calibration, and stationary gyroscope bias calibration.
* Accuracy evaluated on the BROAD dataset and embedded performance measured on STM32F411.

## API stability

The library is under active development. The API may change between releases.

## Installation

Add `uf-ahrs` to your `Cargo.toml`:

```toml
[dependencies]
uf-ahrs = "*" # Replace * with the version you want to use.
```

Or use the command line:

```bash
cargo add uf-ahrs
```

## Quick start

Create a filter with a sample period, then call `update` for each sensor sample.
This example uses the Mahony filter at 100 Hz and returns orientation as a `UnitQuaternion<f32>`.

> **Note on units**
>
> - Gyroscope data must be in **rad/s**.
> - Accelerometer and magnetometer units do not matter, as long as they are consistent.
>   The filters normalize these vectors internally.

```rust
use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{Ahrs, Mahony, MahonyParams};

fn main() {
    let dt = Duration::from_secs_f32(1.0 / 100.0);

    let mut filter = Mahony::new(dt, MahonyParams::default());

    let gyro = Vector3::new(0.0, 0.0, 0.0); // rad/s
    let accel = Vector3::new(0.0, 0.0, 9.81);
    let mag = Vector3::new(20.0, 0.0, 0.0);

    let orientation = filter.update(gyro, accel, mag);

    // Printing requires std; the library itself is no_std.
    println!("Roll, pitch, yaw (radians): {:?}", orientation.euler_angles());
}
```

To use another filter, replace the import and constructor:

* Madgwick: import `Madgwick` and `MadgwickParams`, then use `Madgwick::new(dt, MadgwickParams::default())`.
* VQF: import `Vqf` and `VqfParams`, then use `Vqf::new(dt, VqfParams::default())`.

All three filters implement `Ahrs`, so the update call stays the same.
Without a magnetometer, call `filter.update_imu(gyro, accel)`.

## Sensor Axis Remap

Use `AxisRemap` when sensor axes differ from body axes, such as when an IMU is mounted
rotated or upside down. For each body axis, specify the sensor axis and its sign.
Apply the remap before feeding readings into the filter. Sensors with the same axis
alignment can share a remap.

```rust
use nalgebra::Vector3;
use uf_ahrs::{Axis, AxisRemap};

fn main() {
    // 90-degree mounting rotation about Z: body X = sensor Y,
    // body Y = -sensor X, body Z = sensor Z.
    let remap = AxisRemap::new(Axis::YPos, Axis::XNeg, Axis::ZPos).expect("valid permutation");

    let raw_acc = Vector3::new(0.0, 0.0, 9.81);

    // Remap into the body frame before feeding the filter.
    let acc = remap.remap(raw_acc);

    println!("acc: {acc:?}");
}
```

## Sensor Calibration

Accurate filter output requires calibrated sensor measurements. `InertialCalibration` applies
misalignment, sensitivity, and offset correction to gyroscope or accelerometer readings, and
`MagnetometerCalibration` applies soft-iron and hard-iron correction to magnetometer readings.
Both default to an identity model (no correction), so they can be wired into a sensor pipeline
unconditionally even before calibration coefficients are known.

`GyroBiasCalibration` can determine a stationary gyroscope offset. Other coefficients must come
from an external calibration procedure (datasheet, factory calibration, or a calibration routine
run once per device).

```rust
use nalgebra::{Matrix3, Vector3};
use uf_ahrs::{InertialCalibration, MagnetometerCalibration};

fn main() {
    let accel_cal = InertialCalibration {
        misalignment: Matrix3::identity(),
        sensitivity: Vector3::new(1.002, 0.998, 1.001),
        offset: Vector3::new(0.01, -0.02, 0.03),
    };
    let mag_cal = MagnetometerCalibration {
        soft_iron: Matrix3::identity(),
        hard_iron: Vector3::new(2.0, -1.5, 0.5),
    };

    let raw_acc = Vector3::new(0.0, 0.0, 9.80);
    let raw_mag = Vector3::new(22.0, -1.5, 0.5);

    // Apply calibration before feeding the filter.
    let acc = accel_cal.calibrate(raw_acc);
    let mag = mag_cal.calibrate(raw_mag);

    println!("acc: {acc:?}, mag: {mag:?}");
}

```

### Stationary gyroscope bias calibration

Feed stationary gyroscope readings in **rad/s** to `GyroBiasCalibration::update(dt, gyro)`.
Once the configured window duration and minimum sample count are reached without exceeding
the standard deviation, drift rate, or peak-to-peak limits, calibration completes and
returns `GyroBiasCalibrationStatus::Complete(bias)`. Rejected windows automatically restart
collection on the next sample.

```rust
use core::time::Duration;
use nalgebra::Vector3;
use uf_ahrs::{GyroBiasCalibration, InertialCalibration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dt = Duration::from_millis(10);
    let mut calibration = GyroBiasCalibration::default();

    // Simulated stationary sensor readings in rad/s.
    for _ in 0..125 {
        let _status = calibration.update(dt, Vector3::new(0.02, -0.01, 0.03));
    }

    if let Some(offset) = calibration.bias() {
        let gyro_calibration = InertialCalibration {
            offset,
            ..Default::default()
        };
        let corrected = gyro_calibration.calibrate(Vector3::new(0.02, -0.01, 0.03));
        // Feed corrected readings into any AHRS filter.
        assert!(corrected.norm() < 1e-6);
    }
    Ok(())
}
```

Use `GyroBiasCalibration::new(params)` to customize thresholds or `GyroBiasCalibrationParams::from_noise_density`
to estimate starting thresholds from a datasheet noise spectral density (in `rad/s/√Hz`):

```rust
use core::time::Duration;
use uf_ahrs::GyroBiasCalibrationParams;

// 0.0028 deg/s/√Hz is approximately 4.8869e-5 rad/s/√Hz.
let params = GyroBiasCalibrationParams::from_noise_density(
    Duration::from_millis(1250),
    Duration::from_millis(10),
    4.8869e-5,
)?;
# Ok::<(), uf_ahrs::GyroBiasCalibrationError>(())
```

Use `statistics()` to inspect the current or completed window (`minimum`, `maximum`, `mean`,
`standard_deviation`, `drift_rate`, and `peak_to_peak()`), and `rejected_statistics()` to inspect
the last rejected window. Call `reset()` to clear the state and recalibrate. See
`examples/gyro_bias_calibration.rs` for a complete example.

## Benchmark Scores (BROAD)

The results below are from evaluations on the **Berlin Robust Orientation Estimation Assessment Dataset (BROAD)**.

The `examples/broad_score` runner calibrates startup gyroscope bias over the first 1.5 seconds of
each recording and compares filter accuracy with and without startup bias correction (excluding the
calibration window from scoring). Run `just eval` to reproduce.

For context, BROAD contains **39 trials** with synchronized IMU and optical ground-truth orientation:
**23 undisturbed trials** (rotation, translation, and combined motion at slow/fast speeds) and
**16 disturbed trials** (e.g., tapping, vibration, stationary/attached magnets, office environment, and mixed conditions).
See the [BROAD dataset publication](https://www.mdpi.com/2306-5729/6/7/72) for details.

Reported metrics are dataset-averaged orientation errors in degrees:
```text
BROAD: 39 recordings, sample period 3.5ms
First 1.5 s excluded in both calibration modes; movement samples only.
Filtered Mahony: kp=0.74, ki=0.0012, tau=750 ms
Filtered Madgwick: beta=0.02, tau=2 s
Scoring Mahony default...
Scoring Mahony filtered...
Scoring Madgwick default...
Scoring Madgwick filtered...
Scoring VQF default...

Mean per-recording RMSE (degrees; lower is better)
                     |     Startup calibration OFF      |      Startup calibration ON
Algorithm            |    Total    Heading  Inclination |    Total    Heading  Inclination
---------------------+----------------------------------+---------------------------------
Mahony default       |   7.4511     6.2592       3.6905 |   6.2364     5.0054       3.4820
Mahony filtered      |   5.3538     4.9571       1.7972 |   3.4642     3.0406       1.4571
Madgwick default     |   5.0682     4.3411       2.3391 |   3.6718     2.8428       2.0579
Madgwick filtered    |   2.7939     2.4684       1.2309 |   2.4255     2.1472       0.9974
VQF default          |   2.3003     2.1525       0.6963 |   2.3018     2.1543       0.6962
```

## Embedded Performance

Measured on an `STM32F411CEUx` Black Pill using the DWT cycle counter. The
benchmark configures the chip to run at `100 MHz`.

### Execution time

| Filter | Update | Cycles/update | Time/update at 100 MHz |
| --- | --- | ---: | ---: |
| Mahony | IMU | 638 | 6.38 µs |
| Mahony | IMU + magnetometer | 1029 | 10.29 µs |
| Madgwick | IMU | 817 | 8.17 µs |
| Madgwick | IMU + magnetometer | 1299 | 12.99 µs |
| VQF | IMU | 4413 | 44.13 µs |
| VQF | IMU + magnetometer | 10282 | 102.82 µs |

### Memory usage

These measurements use release builds with default features and `defmt` disabled.
Reproduce them from the repository root:

```sh
cd examples/stm32-blackpill-bench
just size
# Show individual sections:
just size-sections
```

Flash total is `text + data`. Flash delta is relative to the base firmware.
Static RAM is `data + bss`; it excludes the runtime stack, including filter state stored there.
All sizes below are in bytes.

| Filter | Update | Flash total | Flash delta vs base | Static RAM |
| --- | --- | ---: | ---: | ---: |
| Base firmware | — | 2,556 | — | 4 |
| Mahony | IMU | 18,860 | +16,304 | 4 |
| Mahony filtered | IMU | 18,916 | +16,360 | 4 |
| Mahony | IMU + magnetometer | 18,972 | +16,416 | 4 |
| Mahony filtered | IMU + magnetometer | 19,020 | +16,464 | 4 |
| Madgwick | IMU | 17,756 | +15,200 | 4 |
| Madgwick filtered | IMU | 17,796 | +15,240 | 4 |
| Madgwick | IMU + magnetometer | 18,980 | +16,424 | 4 |
| Madgwick filtered | IMU + magnetometer | 19,020 | +16,464 | 4 |
| VQF | IMU | 28,844 | +26,288 | 4 |
| VQF | IMU + magnetometer | 32,356 | +29,800 | 4 |

Filtered Mahony uses `kp=0.74`, `ki=0.0012`, and `tau=750 ms`; filtered Madgwick
uses `beta=0.02` and `tau=2 s`. Unfiltered rows use library defaults.

## License

Licensed under Apache 2.0. See [LICENSE](https://github.com/jettify/uf-ahrs/blob/master/LICENSE) for details.

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
