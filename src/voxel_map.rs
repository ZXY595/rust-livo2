//! more infomation see [`https://arxiv.org/pdf/2109.07082`]

use std::ops::Deref;

use museair::HashMap;

use nalgebra::{
    DVector, Matrix2, Matrix3, Matrix3x2, Point3, Rotation3, SymmetricEigen, Vector3, stack, vector,
};
use num_traits::Zero;

use crate::{
    esikf, imu,
    utils::{
        PointCovSum,
        jacobian::{Covariance1, Covariance2, Covariance3, Covariance6},
        space::{BodyPoint, BodySpace, SpacePoint, WorldPoint, WorldSpace},
    },
};

pub struct Config {
    beam_err: f64,
    dept_err: f64,
    sigma_num: f64,
    planer_threshold: f64,
    max_points_num: usize,
    layer_init_threshold: &'static [usize],
    voxel_size: f64,
}

impl Config {
    pub const fn max_layer(&self) -> usize {
        self.layer_init_threshold.len()
    }
}

#[derive(Debug, Clone)]
pub struct GaussianPoint<S> {
    /// from imu frame to world frame
    point: SpacePoint<f64, S>,
    covariance: Covariance3<f64>,
}

impl<S> Deref for GaussianPoint<S> {
    type Target = SpacePoint<f64, S>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

impl GaussianPoint<BodySpace> {
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

        let range_covariance = Covariance1::from_element(config.beam_err.powi(2));

        let direction_covariance: Covariance2<f64> =
            Matrix2::from_diagonal_element(config.dept_err.to_radians().sin().powi(2)).into();

        let covariance = range_covariance.propagate_error(point_direction)
            + direction_covariance.propagate_error(point_base_coords);

        Self { point, covariance }
    }
}

impl GaussianPoint<WorldSpace> {
    pub fn from_body_point(
        body_point: GaussianPoint<BodySpace>,
        current_pose: &esikf::State,
        imu_config: &imu::Config,
    ) -> Self {
        let rotation_covariance = current_pose.rotation_covariance();
        let translation_covariance = current_pose.translation_covariance();

        let body_to_world_rotation =
            current_pose.rotation() * imu_config.body_to_imu.rotation.matrix();

        let covariance = body_point
            .covariance
            .propagate_error(body_to_world_rotation)
            + rotation_covariance.propagate_error(
                body_to_world_rotation * /* ignored '-' */ body_point.coords.cross_matrix(),
            )
            + translation_covariance;

        let world_point = body_point
            .to_imu_point(imu_config)
            .to_world_point(current_pose);

        Self {
            point: world_point,
            covariance,
        }
    }

    pub fn from_body_point_without_pose_error(
        body_point: GaussianPoint<BodySpace>,
        current_pose: &esikf::State,
        imu_config: &imu::Config,
    ) -> Self {
        let body_to_world_rotation =
            current_pose.rotation() * imu_config.body_to_imu.rotation.matrix();

        let covariance = body_point
            .covariance
            .propagate_error(body_to_world_rotation);

        let world_point = body_point
            .to_imu_point(imu_config)
            .to_world_point(current_pose);

        Self {
            point: world_point,
            covariance,
        }
    }
}

impl<S> GaussianPoint<S> {
    fn point(&self) -> &Point3<f64> {
        &self.point.point
    }
}

pub struct Plane {
    normal: Vector3<f64>,
    center: WorldPoint<f64>,
    covariance: Covariance6<f64>,
    point_num: usize,
    radius: f64,
    distance_to_origin: f64,
    // eigenvalues: Vector3<f64>,
    // eigenvectors: Matrix3<f64>,
}

impl Plane {
    pub fn new(plane_points: &DVector<GaussianPoint<WorldSpace>>, config: &Config) -> Option<Self> {
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
            .map(|gaussian_point| {
                let rows = std::array::from_fn::<_, 3, _>(|i| {
                    if i == min_eigen_index {
                        return Vector3::zeros().transpose();
                    }
                    let i_eigenvector = eigenvectors.column(i);
                    let min_eigenvector = eigenvectors.column(min_eigen_index);

                    (gaussian_point.point.deref() - center).coords.transpose()
                        / (point_num * (min_eigen_value - eigenvalues[i]))
                        * (i_eigenvector * min_eigenvector.transpose()
                            + min_eigenvector * i_eigenvector.transpose())
                });
                let normal_error = eigenvectors * Matrix3::from_rows(&rows);
                let position_error = Matrix3::from_diagonal_element(point_num.recip());

                #[expect(clippy::toplevel_ref_arg)]
                let error_matrix = stack![normal_error; position_error];

                gaussian_point.covariance.propagate_error(error_matrix)
            })
            .sum::<Covariance6<f64>>();

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

    pub fn sigma_to(&self, world_point: GaussianPoint<WorldSpace>) -> f64 {
        let distance_error = world_point.coords - self.center.coords;
        let normal_error = -self.normal;

        #[expect(clippy::toplevel_ref_arg)]
        let error_matrix = stack![distance_error; normal_error];

        self.covariance.project_on(error_matrix) + world_point.covariance.project_on(self.normal)
    }

    pub fn distance_to(&self, world_point: WorldPoint<f64>) -> f64 {
        self.normal.dot(&world_point.coords) - self.distance_to_origin
    }
}

pub struct VoxelIndex(WorldPoint<i64>);

impl Deref for VoxelIndex {
    type Target = WorldPoint<i64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl VoxelIndex {
    pub fn from_point(point: WorldPoint<f64>, voxel_size: f64) -> Self {
        let point: WorldPoint<_> = point
            .map(|x| x / voxel_size)
            .map(f64::floor)
            .map(|x| x as i64)
            .into();
        point.into()
    }
}

impl From<WorldPoint<i64>> for VoxelIndex {
    fn from(value: WorldPoint<i64>) -> Self {
        Self(value)
    }
}

pub struct Octree {
    points: DVector<GaussianPoint<WorldSpace>>,
}

pub struct VoxelMap {
    trees: HashMap<VoxelIndex, Octree>,
}

impl Extend<WorldPoint<f64>> for VoxelMap {
    fn extend<T>(&mut self, iter: T)
    where
        T: IntoIterator<Item = WorldPoint<f64>>,
    {
        todo!()
    }
}

pub trait PointCloudMap: Extend<WorldPoint<f64>> {
    fn build_residual(&self, config: &Config);
}
