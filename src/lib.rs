#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]
mod mahony;
mod traits;

pub use mahony::Mahony;
pub use traits::Ahrs;
