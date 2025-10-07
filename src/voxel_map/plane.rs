use std::borrow::Borrow;

use super::Config;
use crate::{
    frame::{World, WorldPoint},
    uncertain::{Uncertain, UncertainForward, Uncertained, Uncertainty3},
    utils::VectorSquareSum,
    voxel_map::point::UncertainPoint,
};
use nalgebra::{DVector, Matrix3, Matrix6, RowVector3, SymmetricEigen, Vector3, stack};
use rust_livo2_macros::uncertainties;

pub struct Plane {
    normal: Vector3<f64>,
    center: WorldPoint<f64>,
    points_count: usize,
    radius: f64,
    distance_to_origin: f64,
}

impl Uncertain for Plane {
    type Uncertainty = PlaneUncertainties;
}

pub type UncertainPlane = Uncertained<Plane>;

#[uncertainties]
#[derive(Debug)]
pub struct PlaneUncertainties {
    normal: NormalUncertainty,
    center: CenterUncertainty,
}

type NormalUncertainty = Uncertainty3<f64>;
type CenterUncertainty = Uncertainty3<f64>;

impl UncertainPlane {
    pub fn new_plane(
        plane_points: &DVector<UncertainPoint<World>>,
        config: &Config,
    ) -> Option<Self> {
        let sum = plane_points
            .iter()
            .map(UncertainPoint::point)
            .map(|point| &point.coords)
            .sum::<VectorSquareSum>();

        let points_count = sum.count();
        let (center, covariance) = sum.mean();

        let SymmetricEigen {
            eigenvectors,
            eigenvalues,
        } = covariance.symmetric_eigen();

        let (min_eigen_index, min_eigen_value) = eigenvalues.argmin();

        if min_eigen_value < config.planer_threshold {
            return None;
        }

        let covariance_matrix = plane_points
            .iter()
            .map(|uncertain_point| {
                let points_count = points_count as f64;

                let rows: [RowVector3<f64>; 3] = std::array::from_fn(|i| {
                    if i == min_eigen_index {
                        return RowVector3::zeros();
                    }
                    let i_eigenvector = eigenvectors.column(i);
                    let min_eigenvector = eigenvectors.column(min_eigen_index);

                    (uncertain_point.point() - center).coords.transpose()
                        / (points_count * (min_eigen_value - eigenvalues[i]))
                        * (i_eigenvector * min_eigenvector.transpose()
                            + min_eigenvector * i_eigenvector.transpose())
                });
                let normal_error = eigenvectors * Matrix3::from_rows(&rows);
                let position_error = Matrix3::from_diagonal_element(points_count.recip());

                #[expect(clippy::toplevel_ref_arg)]
                let error_matrix = stack![normal_error; position_error];

                uncertain_point.covariance.forward(error_matrix)
            })
            .sum::<Matrix6<f64>>();

        let normal = eigenvectors.column(min_eigen_index);

        let distance_to_origin = normal.dot(&center);

        Some(Self::new(
            Plane {
                normal: normal.into(),
                center: center.into(),
                points_count,
                radius: eigenvalues.max().sqrt(),
                distance_to_origin,
            },
            covariance_matrix.into(),
        ))
    }

    pub fn sigma_to(&self, world_point: UncertainPoint<World>) -> f64 {
        let distance_error = world_point.coords - self.center.coords;
        let normal_error = -self.borrow().normal;

        #[expect(clippy::toplevel_ref_arg)]
        let error_matrix = stack![distance_error; normal_error];

        let sigma =
            self.covariance.backward(error_matrix) + world_point.covariance.backward(self.normal);
        sigma.to_scalar()
    }

    pub fn distance_to(&self, world_point: WorldPoint<f64>) -> f64 {
        self.normal.dot(&world_point.coords) - self.distance_to_origin
    }
}
