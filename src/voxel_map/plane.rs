use super::Config;
use crate::{
    utils::{
        PointCovSum,
        frame::{World, WorldPoint},
        uncertain::Uncertainty6,
    },
    voxel_map::point::UncertainPoint,
};
use nalgebra::{DVector, Matrix3, RowVector3, SymmetricEigen, Vector3, stack};
use std::ops::Deref;

pub struct UncertainPlane {
    normal: Vector3<f64>,
    center: WorldPoint<f64>,
    covariance: Uncertainty6<f64>,
    point_num: usize,
    radius: f64,
    distance_to_origin: f64,
    // eigenvalues: Vector3<f64>,
    // eigenvectors: Matrix3<f64>,
}

impl UncertainPlane {
    pub fn new(plane_points: &DVector<UncertainPoint<World>>, config: &Config) -> Option<Self> {
        let points_sum = plane_points
            .iter()
            .map(UncertainPoint::point)
            .sum::<PointCovSum>();
        let point_num = points_sum.num as f64;
        let center = points_sum.mean / point_num;
        let covariance = points_sum.covariance / point_num - center * center.transpose();

        let SymmetricEigen {
            eigenvectors,
            eigenvalues,
        } = covariance.symmetric_eigen();

        let (min_eigen_index, min_eigen_value) = eigenvalues.argmin();

        if min_eigen_value < config.planer_threshold {
            return None;
        }

        let covariance = plane_points
            .iter()
            .map(|uncertain_point| {
                let rows: [RowVector3<f64>; 3] = std::array::from_fn(|i| {
                    if i == min_eigen_index {
                        return RowVector3::zeros();
                    }
                    let i_eigenvector = eigenvectors.column(i);
                    let min_eigenvector = eigenvectors.column(min_eigen_index);

                    (uncertain_point.point.deref() - center).coords.transpose()
                        / (point_num * (min_eigen_value - eigenvalues[i]))
                        * (i_eigenvector * min_eigenvector.transpose()
                            + min_eigenvector * i_eigenvector.transpose())
                });
                let normal_error = eigenvectors * Matrix3::from_rows(&rows);
                let position_error = Matrix3::from_diagonal_element(point_num.recip());

                #[expect(clippy::toplevel_ref_arg)]
                let error_matrix = stack![normal_error; position_error];

                uncertain_point.covariance.forward(error_matrix)
            })
            .sum::<Uncertainty6<f64>>();

        let normal = eigenvectors.column(min_eigen_index);

        let distance_to_origin = normal.dot(&center);

        Some(Self {
            normal: normal.into(),
            center: center.into(),
            covariance,
            point_num: points_sum.num,
            radius: eigenvalues.max().sqrt(),
            distance_to_origin,
            // eigenvectors,
        })
    }

    pub fn sigma_to(&self, world_point: UncertainPoint<World>) -> f64 {
        let distance_error = world_point.coords - self.center.coords;
        let normal_error = -self.normal;

        #[expect(clippy::toplevel_ref_arg)]
        let error_matrix = stack![distance_error; normal_error];

        self.covariance.back(error_matrix) + world_point.covariance.back(self.normal)
    }

    pub fn distance_to(&self, world_point: WorldPoint<f64>) -> f64 {
        self.normal.dot(&world_point.coords) - self.distance_to_origin
    }
}
