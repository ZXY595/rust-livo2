//! Implementation of Error-State Iterated Kalman Filter

use std::ops::Deref;

use crate::{
    frame::{Framed, Imu, World},
    uncertain::Uncertainty3,
};
use nalgebra::{IsometryMatrix3, Rotation3, Vector3, Vector6, stack};
use rust_livo2_macros::uncertainties;

pub struct Config {
    max_iterations: u32,
}

pub struct UncertainOdometer {
    /// estimated isometry, from imu frame to world frame
    pub isometry: Framed<IsometryMatrix3<f64>, fn(Imu) -> World>,
    /// odometer covariance
    pub covariance: OdometerUncertainties,
}

#[uncertainties]
#[derive(Debug)]
pub struct OdometerUncertainties {
    pub rotation: RotationUncertainty,
    pub translation: TranslationUncertainty,
}

type RotationUncertainty = Uncertainty3<f64>;
type TranslationUncertainty = Uncertainty3<f64>;

impl UncertainOdometer {
    pub fn rotation(&self) -> Rotation3<f64> {
        self.isometry.rotation
    }

    pub fn diff_vector(&self, other: &Self) -> Vector6<f64> {
        let rotation = other.isometry.rotation.transpose() * self.isometry.rotation;
        let translation = self.isometry.translation.vector - other.isometry.translation.vector;
        let rotation = rotation
            .axis_angle()
            .map(|(axis, angle)| axis.deref() * angle * angle / angle.sin())
            .unwrap_or(Vector3::zeros());

        #[expect(clippy::toplevel_ref_arg)]
        {
            stack![rotation; translation]
        }
    }
}

pub trait KalmanFilterIterator: Iterator<Item = UncertainOdometer> {}
