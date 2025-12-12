#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod madgwick;
mod mahony;
mod traits;

pub use madgwick::Madgwick;
pub use mahony::Mahony;
pub use traits::Ahrs;
