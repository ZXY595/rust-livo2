use super::Config;
use crate::{
    esikf::UncertainOdometer,
    frame::{self, Body, BodyPoint, Framed, Imu, World},
    uncertain::{Uncertain, UncertainForward, Uncertained, Uncertainty1, Uncertainty2},
};
use nalgebra::{IsometryMatrix3, Matrix2, Matrix3x2, Point3, vector};
use num_traits::Zero;
use rust_livo2_macros::uncertainties;
use std::{borrow::Borrow, ops::Deref};

type FramedPoint<F> = frame::FramedPoint<f64, F>;

/// A uncertain point in F frame.
pub type UncertainPoint<F> = Uncertained<FramedPoint<F>>;

#[uncertainties]
#[derive(Debug, Clone)]
pub struct BodyPointUncertainties {
    pub distance: DistanceUncertainty,
    pub direction: DirectionUncertainty,
}

type DistanceUncertainty = Uncertainty1<f64>;
type DirectionUncertainty = Uncertainty2<f64>;
pub type WorldPointUncertainties = BodyPointUncertaintiesMatrix;

impl Uncertain for FramedPoint<Body> {
    type Uncertainty = BodyPointUncertainties;
}

impl Uncertain for FramedPoint<World> {
    type Uncertainty = WorldPointUncertainties;
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
                }
        ]
        .normalize();
        let base2 = base1.cross(&point_direction).normalize();

        let point_base_coords =
            range * point_direction.cross_matrix() * Matrix3x2::from_columns(&[base1, base2]);

        let distance_covariance = DistanceUncertainty::from_element(config.beam_err.powi(2));

        let direction_covariance: DirectionUncertainty =
            Matrix2::from_diagonal_element(config.dept_err.to_radians().sin().powi(2));

        let covariance_matrix = distance_covariance.forward(point_direction)
            + direction_covariance.forward(point_base_coords);

        Self::new_uncertained(point, covariance_matrix.into())
    }
}

impl UncertainPoint<World> {
    pub fn from_body_point(
        body_point: UncertainPoint<Body>,
        current_odom: &UncertainOdometer,
        body_to_imu: &Framed<IsometryMatrix3<f64>, fn(Body) -> Imu>,
    ) -> Self {
        let odom_covariance = &current_odom.covariance;
        let rotation_covariance = odom_covariance.view_rotation();
        let translation_covariance = odom_covariance.view_translation();

        let body_to_world_rotation = current_odom.rotation() * body_to_imu.rotation.matrix();

        let covariance_matrix = body_point.covariance.forward(body_to_world_rotation)
            + rotation_covariance.forward(
                body_to_world_rotation * /* ignored '-' */ body_point.coords.cross_matrix(),
            )
            + translation_covariance;

        let world_point = body_point
            .to_imu_point(body_to_imu)
            .to_world_point(&current_odom.isometry);

        Self::new_uncertained(world_point, covariance_matrix)
    }

    #[expect(unused)]
    pub fn from_body_point_without_pose_error(
        body_point: UncertainPoint<Body>,
        current_pose: &UncertainOdometer,
        body_to_imu: &Framed<IsometryMatrix3<f64>, fn(Body) -> Imu>,
    ) -> Self {
        let body_to_world_rotation = current_pose.rotation() * body_to_imu.rotation.matrix();

        let covariance_matrix = body_point.covariance.forward(body_to_world_rotation);

        let world_point = body_point
            .to_imu_point(body_to_imu)
            .to_world_point(&current_pose.isometry);

        Self::new_uncertained(world_point, covariance_matrix)
    }
}

impl<F> UncertainPoint<F>
where
    FramedPoint<F>: Uncertain,
{
    pub(crate) fn point(&self) -> &Point3<f64> {
        let framed: &FramedPoint<F> = self.borrow();
        framed.deref()
    }
}
