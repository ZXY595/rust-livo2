//! Implementation of Error-State Iterated Kalman Filter

use crate::utils::jacobian::{Covariance, Covariance3View};
use nalgebra::{IsometryMatrix3, Rotation3};

pub struct State {
    /// estimated isometry, from imu frame to world frame
    pub isometry: IsometryMatrix3<f64>,
    /// states covariance
    covariance: Covariance<f64, { Self::COV_DIMENSION }>,
}

impl State {
    pub const COV_DIMENSION: usize = 19;

    pub fn rotation(&self) -> Rotation3<f64> {
        self.isometry.rotation
    }

    pub fn rotation_covariance(&self) -> Covariance3View<'_, f64, 1, { Self::COV_DIMENSION }> {
        self.covariance.fixed_view::<3, 3>(0, 0).into()
    }

    pub fn translation_covariance(&self) -> Covariance3View<'_, f64, 1, { Self::COV_DIMENSION }> {
        self.covariance.fixed_view::<3, 3>(3, 3).into()
    }
}
