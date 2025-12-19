#![no_std]
#![allow(clippy::needless_doctest_main)]
#![doc = include_str!("../README.md")]

mod madgwick;
mod mahony;
pub(crate) mod mean_init_lfp;
mod traits;
mod vqf;

pub use madgwick::Madgwick;
pub use mahony::Mahony;
pub use traits::Ahrs;
pub use vqf::Vqf;
pub use vqf::VqfParameters;
