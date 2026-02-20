# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.1](https://github.com/jettify/uf-ahrs/compare/v0.1.0...v0.1.1) - 2026-02-20

### Fixed

- Handle zero tau correctly. ([#18](https://github.com/jettify/uf-ahrs/pull/18))

### Other

- Add more test for coverage. ([#25](https://github.com/jettify/uf-ahrs/pull/25))
- Strict lints. ([#24](https://github.com/jettify/uf-ahrs/pull/24))
- Do not use indexes in VQF to avoid panics. ([#23](https://github.com/jettify/uf-ahrs/pull/23))
- Avoid using indexes in Madgwick to minimize risk of panics. ([#22](https://github.com/jettify/uf-ahrs/pull/22))
- Avoid indexing for better safety in Mahony algo. ([#21](https://github.com/jettify/uf-ahrs/pull/21))
- Upgrade idsp library. ([#20](https://github.com/jettify/uf-ahrs/pull/20))
- Avoid using indexes in madgwick ([#19](https://github.com/jettify/uf-ahrs/pull/19))
- More numericaly stable Kalman implementation.
- Upgrade idsp library.
- Share tests across algos to increase coverage. ([#17](https://github.com/jettify/uf-ahrs/pull/17))
- Correct REAMDE links add docs reference.
- release v0.1.0 ([#15](https://github.com/jettify/uf-ahrs/pull/15))

## [0.1.0](https://github.com/jettify/uf-ahrs/releases/tag/v0.1.0) - 2026-02-08

### Added

- Setup release proccess. ([#14](https://github.com/jettify/uf-ahrs/pull/14))
- Score all algorithms against dataset. ([#7](https://github.com/jettify/uf-ahrs/pull/7))
- Rework filters constructors. ([#6](https://github.com/jettify/uf-ahrs/pull/6))
- Use Duration type for dt.
- VQF filter ([#5](https://github.com/jettify/uf-ahrs/pull/5))
- Rework evaluation code for algorithms.
- Add set orientation to the interface for easier testing.
- Add orientation to ahrs trait.
- Do not borrow stack allocated items.
- Experiment with madgwick
- Experiment with madgwick
- Add new ahrs trait.
- Function to estimate initial orientation.
- Add magnetometer correction.
- First iterration for implementation.
- Add justfile
- Basic project setup
- Initial commit

### Fixed

- Fix bias correction in vqf. ([#13](https://github.com/jettify/uf-ahrs/pull/13))
- Align quat_from_acc_mag function wit known data points.
- Fixed initial estimation of quaternion.

### Other

- Updae cargo toml and readme. ([#12](https://github.com/jettify/uf-ahrs/pull/12))
- Add license file. ([#11](https://github.com/jettify/uf-ahrs/pull/11))
- Add simple example, and minor cleanup. ([#10](https://github.com/jettify/uf-ahrs/pull/10))
- Update reame with references. ([#9](https://github.com/jettify/uf-ahrs/pull/9))
- Make LPF more readable. ([#8](https://github.com/jettify/uf-ahrs/pull/8))
- refactor VQF to use duration for sample rate.
- Cleanup rebase issues.
- Better naming for default parameters in filters.
- Clean filters implementation.
- Add cargo audio CI job
- Improve justfile and evaluation logic.
- Add script to convert evaluation data to parquet.
- Move quat_from_acc_mag to example.
- Address lint and improve try_normalize usage.
- Add basic github CI
- Mahony tests cleanup
- Add basic  description to readme
- Cover with more test casess.
