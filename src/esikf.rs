//! Implementation of Error-State Iterated Kalman Filter

use crate::utils::uncertain::{Uncertainty, UncertaintyView};
use nalgebra::{IsometryMatrix3, Rotation3, Vector3, Vector6, stack};

pub struct Config {
    max_iterations: u32,
}

pub struct State {
    /// estimated isometry, from imu frame to world frame
    pub isometry: IsometryMatrix3<f64>,
    /// states covariance
    covariance: Uncertainty<f64, { Self::COV_DIMENSION }>,
}

impl State {
    pub const COV_DIMENSION: usize = 19;

    pub fn rotation(&self) -> Rotation3<f64> {
        self.isometry.rotation
    }

    pub fn rotation_covariance(&self) -> UncertaintyView<'_, f64, 3, 1, { Self::COV_DIMENSION }> {
        self.covariance.fixed_view::<3, 3>(0, 0).into()
    }

    pub fn translation_covariance(&self) -> UncertaintyView<'_, f64, 3, 1, { Self::COV_DIMENSION }> {
        self.covariance.fixed_view::<3, 3>(3, 3).into()
    }

    pub fn diff_vector(&self, other: &Self) -> Vector6<f64> {
        let rotation = other.isometry.rotation.transpose() * self.isometry.rotation;
        let translation = self.isometry.translation.vector - other.isometry.translation.vector;
        let rotation = rotation
            .axis_angle()
            .map(|(axis, angle)| axis.into_inner() * angle * angle / angle.sin())
            .unwrap_or(Vector3::zeros());

        #[expect(clippy::toplevel_ref_arg)]
        {
            stack![rotation; translation]
        }
    }
}

pub trait KalmanFilterIterator: Iterator<Item = State> {}
