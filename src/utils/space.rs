use std::{marker::PhantomData, ops::Deref};

use nalgebra::{Point3, Scalar, Vector3};
use num_traits::Zero;

use crate::{esikf, imu};

pub enum WorldSpace {}

pub enum ImuSpace {}

/// also known as the space your collect point cloud
pub enum BodySpace {}

pub type WorldPoint<T> = SpacePoint<T, WorldSpace>;
pub type ImuPoint<T> = SpacePoint<T, ImuSpace>;
pub type BodyPoint<T> = SpacePoint<T, BodySpace>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpacePoint<T, S>
where
    T: Scalar,
{
    pub point: Point3<T>,
    space: PhantomData<S>,
}

impl<T, S> SpacePoint<T, S>
where
    T: Scalar,
{
    pub fn new(point: Point3<T>) -> Self {
        Self {
            point,
            space: PhantomData,
        }
    }
}

impl<T, S> From<Point3<T>> for SpacePoint<T, S>
where
    T: Scalar,
{
    fn from(value: Point3<T>) -> Self {
        Self::new(value)
    }
}

impl<T, S> From<Vector3<T>> for SpacePoint<T, S>
where
    T: Scalar,
{
    fn from(value: Vector3<T>) -> Self {
        Self::new(Point3::from(value))
    }
}

impl<T, S> Default for SpacePoint<T, S>
where
    T: Scalar + Default + Zero,
{
    fn default() -> Self {
        Self {
            point: Default::default(),
            space: Default::default(),
        }
    }
}

impl<T, S> Deref for SpacePoint<T, S>
where
    T: Scalar,
{
    type Target = Point3<T>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

impl BodyPoint<f64> {
    pub fn to_imu_point(&self, imu_config: &imu::Config) -> ImuPoint<f64> {
        (imu_config.body_to_imu * self.point).into()
    }
}

impl ImuPoint<f64> {
    pub fn to_world_point(&self, current_state: &esikf::State) -> WorldPoint<f64> {
        (current_state.isometry * self.point).into()
    }
}
