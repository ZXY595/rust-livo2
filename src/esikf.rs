//! Implementation of Error-State Iterated Kalman Filter

use crate::utils::jacobian::Covariance;
use nalgebra::{IsometryMatrix3};

pub struct State {
    /// estimated isometry, from imu frame to world frame
    isometry: IsometryMatrix3<f64>,
    /// states covariance
    covariance: Covariance<f64, { Self::COV_DIMENSION }>,
}

impl State {
    pub const COV_DIMENSION: usize = 19;
}

impl State {}
