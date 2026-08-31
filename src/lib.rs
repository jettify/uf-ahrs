#![no_std]
#![allow(
    clippy::needless_doctest_main,
    reason = "This is readme example, not doctest"
)]
#![doc = include_str!("../README.md")]

mod madgwick;
mod mahony;
pub(crate) mod mean_init_lfp;
mod remap;
mod traits;
mod vqf;

pub use traits::{Ahrs, AhrsWithDt};

pub use remap::Axis;
pub use remap::AxisRemap;

pub use madgwick::Madgwick;
pub use madgwick::MadgwickParams;

pub use mahony::Mahony;
pub use mahony::MahonyParams;

pub use vqf::Vqf;
pub use vqf::VqfParams;
