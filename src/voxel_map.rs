use museair::HashMap;

use nalgebra::{
    DVector, Matrix2, Matrix3, Matrix3x2, Point3, SymmetricEigen, Vector3, stack, vector,
};
use num_traits::Zero;

use crate::utils::{
    PointCovSum,
    jacobian::{Covariance1, Covariance2, Covariance3, Covariance6},
};

pub struct Config {
    beam_err: f64,
    dept_err: f64,
    sigma_num: f64,
    planer_threshold: f64,
    max_points_num: usize,
    layer_init_threshold: &'static [usize],
}

impl Config {
    pub const fn max_layer(&self) -> usize {
        self.layer_init_threshold.len()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct GaussianPoint {
    /// from imu frame to world frame
    point: Point3<f64>,
    covariance: Covariance3<f64>,
}

impl GaussianPoint {
    /// point must be in world frame
    pub fn new(point: Point3<f64>, config: &Config) -> Self {
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

        let range_covariance = Covariance1::from_element(config.beam_err.powi(2));

        let direction_covariance: Covariance2<f64> =
            Matrix2::from_diagonal_element(config.dept_err.to_radians().sin().powi(2)).into();

        let covariance = range_covariance.propagate_error(&point_direction)
            + direction_covariance.propagate_error(&point_base_coords);

        Self { point, covariance }
    }

    fn point(&self) -> &Point3<f64> {
        &self.point
    }
}

pub struct Plane {
    normal: Vector3<f64>,
    center: Point3<f64>,
    covariance: Covariance6<f64>,
    point_num: usize,
    radius: f64,

    // eigenvalues: Vector3<f64>,
    eigenvectors: Matrix3<f64>,
}

impl Plane {
    pub fn new(plane_points: &DVector<GaussianPoint>, config: &Config) -> Option<Self> {
        let points_sum = plane_points
            .iter()
            .map(GaussianPoint::point)
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
            .map(|point| {
                let rows = std::array::from_fn::<_, 3, _>(|i| {
                    if i == min_eigen_index {
                        return Vector3::zeros().transpose();
                    }
                    let i_eigenvector = eigenvectors.column(i);
                    let min_eigenvector = eigenvectors.column(min_eigen_index);

                    (point.point - center).coords.transpose()
                        / (point_num * (min_eigen_value - eigenvalues[i]))
                        * (i_eigenvector * min_eigenvector.transpose()
                            + min_eigenvector * i_eigenvector.transpose())
                });
                let center_error = eigenvectors * Matrix3::from_rows(&rows);
                let normal_error = Matrix3::from_diagonal_element(point_num.recip());

                #[expect(clippy::toplevel_ref_arg)]
                let error_matrix = stack![center_error; normal_error];

                point.covariance.propagate_error(&error_matrix)
            })
            .sum::<Covariance6<f64>>();

        Some(Self {
            normal: eigenvectors.column(min_eigen_index).into(),
            center: center.into(),
            covariance,
            point_num: points_sum.num,
            radius: eigenvalues.max().sqrt(),
            eigenvectors,
        })
    }
}

pub struct Octree {
    points: DVector<GaussianPoint>,
}

pub struct VoxelMap {
    map: HashMap<Point3<u32>, Octree>
}
