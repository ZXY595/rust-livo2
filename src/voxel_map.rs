//! more infomation see [`https://arxiv.org/pdf/2109.07082`] and ['https://arxiv.org/pdf/2103.01627']
mod plane;
mod point;
mod point_to_plane;

use std::{
    hash::Hash,
    ops::{Deref, DerefMut, Index, IndexMut},
};

use nohash_hasher::IntMap;

use nalgebra::{DVector, dvector};

use crate::{frame::{World, WorldPoint}, voxel_map::point::UncertainPoint};
use plane::UncertainPlane;

pub struct Config {
    beam_err: f64,
    dept_err: f64,
    sigma_num: f64,
    planer_threshold: f64,
    max_points_num: usize,
    max_layer: usize,
    layer_init_threshold: &'static [usize],
    voxel_size: f64,
}

impl Config {
    pub const fn max_layer(&self) -> usize {
        self.layer_init_threshold.len()
    }
}

pub struct VoxelIndex(WorldPoint<i64>);

impl Hash for VoxelIndex {
    fn hash<H>(&self, hasher: &mut H)
    where
        H: std::hash::Hasher,
    {
        const HASH_P: i64 = 116101;
        const MAX_N: i64 = 10000000000;

        let index = [self.y, self.x]
            .into_iter()
            .fold(self.z, |acc, cur| (acc * HASH_P) % MAX_N + cur);

        hasher.write_i64(index);
    }
}

/// Hasher methods is invoked exactly once
impl nohash_hasher::IsEnabled for VoxelIndex {}

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

struct Leafs([[[Option<Box<Octree>>; 2]; 2]; 2]);

impl Leafs {
    fn empty() -> Self {
        Self([[[None, None], [None, None]], [[None, None], [None, None]]])
    }
    fn iter(&self) -> impl Iterator<Item = &Box<Octree>> {
        self.0
            .iter()
            .flat_map(|z| z.iter().flat_map(|y| y.iter().flat_map(|x| x.iter())))
    }
}

impl Index<&WorldPoint<bool>> for Leafs {
    type Output = Option<Box<Octree>>;

    fn index(&self, index: &WorldPoint<bool>) -> &Self::Output {
        let index = index.map(|is_positive| is_positive as usize);
        &self.0[index.z][index.y][index.x]
    }
}

impl IndexMut<&WorldPoint<bool>> for Leafs {
    fn index_mut(&mut self, index: &WorldPoint<bool>) -> &mut Self::Output {
        let index = index.map(|is_positive| is_positive as usize);
        &mut self.0[index.z][index.y][index.x]
    }
}

pub struct Octree {
    leafs: Leafs,
    points: Vec<WorldPoint<f64>>,
    center: WorldPoint<f64>,
    tree_size: f64,
    plane: Option<UncertainPlane>,
}

impl Octree {
    pub fn new() -> Self {
        todo!()
    }

    pub fn create_plane(&mut self, plane_min_points: usize, planer_threshold: f64) {
        if self.points.len() < plane_min_points {
            return;
        }
        todo!("create uncertain points");
        todo!("create plane from points")
        // self.points.iter().map(UncertainPoint::new_body_point)
        // UncertainPlane::new(&self.points, planer_threshold);
    }
    pub fn cut(&mut self, max_layer: usize) {
        self.points.iter().cloned().for_each(|point| {
            let leaf_index = (point.deref() - self.center.deref())
                .map(|x| x.is_sign_positive())
                .into();

            let leaf = &mut self.leafs[&leaf_index];

            if let Some(leaf) = leaf {
                leaf.points.push(point);
            } else {
                *leaf = Some(Box::new(Octree {
                    points: vec![point],
                    center: leaf_index.framed_map(|point| {
                        point.map(|x| if x { 1.0 } else { -1.0 }) * self.tree_size
                            + self.center.coords
                    }),
                    tree_size: self.tree_size / 2.0,
                    leafs: Leafs::empty(),
                    plane: None,
                }));
            }
        });
        todo!("try to create plane")
    }
}

pub struct VoxelMap {
    trees: IntMap<VoxelIndex, Octree>,
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
