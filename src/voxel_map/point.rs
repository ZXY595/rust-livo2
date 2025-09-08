use super::Config;
use crate::{
    esikf,
    utils::{
        frame::{Body, BodyPoint, FramedPoint, World},
        uncertain::{Uncertainty1, Uncertainty2, Uncertainty3},
    },
};
use nalgebra::{IsometryMatrix3, Matrix2, Matrix3x2, Point3, vector};
use num_traits::Zero;
use std::ops::{Deref, DerefMut};

/// A uncertain point in F frame.
#[derive(Debug, Clone)]
pub struct UncertainPoint<F> {
    /// from imu frame to world frame
    pub point: FramedPoint<f64, F>,
    pub covariance: Uncertainty3<f64>,
}

impl<F> Deref for UncertainPoint<F> {
    type Target = FramedPoint<f64, F>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

impl<F> DerefMut for UncertainPoint<F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.point
    }
}

impl UncertainPoint<Body> {
    pub fn new_body_point(point: BodyPoint<f64>, config: &Config) -> Self {
        let point_coords = &point.coords;
        let range = point_coords.norm();
        let point_direction = point_coords.normalize();

        let base1 = vector![
            1.,
            1.,
            -(point_direction.x + point_direction.y)
                / if point_direction.z.is_zero() {
                    0.0001
                } else {
                    point_direction.z
                },
        ]
        .normalize();
        let base2 = base1.cross(&point_direction).normalize();

        let point_base_coords =
            range * point_direction.cross_matrix() * Matrix3x2::from_columns(&[base1, base2]);

        let range_covariance = Uncertainty1::from_element(config.beam_err.powi(2));

        let direction_covariance: Uncertainty2<f64> =
            Matrix2::from_diagonal_element(config.dept_err.to_radians().sin().powi(2)).into();

        let covariance = range_covariance.forward(point_direction)
            + direction_covariance.forward(point_base_coords);

        Self { point, covariance }
    }
}

impl UncertainPoint<World> {
    pub fn from_body_point(
        body_point: UncertainPoint<Body>,
        current_pose: &esikf::State,
        body_to_imu: &IsometryMatrix3<f64>,
    ) -> Self {
        let rotation_covariance = current_pose.rotation_covariance();
        let translation_covariance = current_pose.translation_covariance();

        let body_to_world_rotation = current_pose.rotation() * body_to_imu.rotation.matrix();

        let covariance = body_point.covariance.forward(body_to_world_rotation)
            + rotation_covariance.forward(
                body_to_world_rotation * /* ignored '-' */ body_point.coords.cross_matrix(),
            )
            + translation_covariance;

        let world_point = body_point
            .to_imu_point(body_to_imu)
            .to_world_point(&current_pose.isometry);

        Self {
            point: world_point,
            covariance,
        }
    }

    pub fn from_body_point_without_pose_error(
        body_point: UncertainPoint<Body>,
        current_pose: &esikf::State,
        body_to_imu: &IsometryMatrix3<f64>,
    ) -> Self {
        let body_to_world_rotation = current_pose.rotation() * body_to_imu.rotation.matrix();

        let covariance = body_point.covariance.forward(body_to_world_rotation);

        let world_point = body_point
            .to_imu_point(body_to_imu)
            .to_world_point(&current_pose.isometry);

        Self {
            point: world_point,
            covariance,
        }
    }
}

impl<S> UncertainPoint<S> {
    pub(crate) fn point(&self) -> &Point3<f64> {
        &self.point.point
    }
}
