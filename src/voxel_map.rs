//! more infomation see [`https://arxiv.org/pdf/2109.07082`] and ['https://arxiv.org/pdf/2103.01627']
mod plane;
mod point;
mod point_to_plane;

use std::ops::{Deref, DerefMut};

use museair::HashMap;

use nalgebra::DVector;

use crate::utils::frame::{World, WorldPoint};

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

pub struct VoxelIndex(WorldPoint<i64>);

impl Deref for VoxelIndex {
    type Target = WorldPoint<i64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for VoxelIndex {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
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
    points: DVector<point::UncertainPoint<World>>,
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
impl VoxelMap {
    pub fn build_residual(&self, config: &Config) {
        todo!()
    }
}
