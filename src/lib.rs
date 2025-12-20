#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod madgwick;
mod mahony;
pub(crate) mod mean_init_lfp;
mod traits;
mod vqf;

pub use traits::Ahrs;

pub use madgwick::Madgwick;
pub use madgwick::MadgwickParams;

pub use mahony::Mahony;
pub use mahony::MahonyParams;

pub use vqf::Vqf;
pub use vqf::VqfParameters;
