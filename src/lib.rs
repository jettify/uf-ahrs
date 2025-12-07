#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]
pub mod madgwick;
mod mahony;
mod traits;

pub use mahony::Mahony;
pub use mahony::quat_from_acc_mag;
pub use traits::Ahrs;
