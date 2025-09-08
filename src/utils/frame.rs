use std::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use nalgebra::{IsometryMatrix3, Point3, Scalar, Vector3};
use num_traits::Zero;

use crate::{esikf, imu};

pub struct World {}
pub struct Imu {}
/// also known as the space your collect point cloud
pub struct Body {}

pub type WorldPoint<T> = FramedPoint<T, World>;
pub type ImuPoint<T> = FramedPoint<T, Imu>;
pub type BodyPoint<T> = FramedPoint<T, Body>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FramedPoint<T, F>
where
    T: Scalar,
{
    pub point: Point3<T>,
    frame: PhantomData<F>,
}

impl<T, F> FramedPoint<T, F>
where
    T: Scalar,
{
    pub fn new(point: Point3<T>) -> Self {
        Self {
            point,
            frame: PhantomData,
        }
    }
    pub fn new_with_frame(point: Point3<T>, frame: F) -> Self {
        let _ = frame;
        Self {
            point,
            frame: PhantomData,
        }
    }
}

impl<T, S> From<Point3<T>> for FramedPoint<T, S>
where
    T: Scalar,
{
    fn from(value: Point3<T>) -> Self {
        Self::new(value)
    }
}

impl<T, S> From<Vector3<T>> for FramedPoint<T, S>
where
    T: Scalar,
{
    fn from(value: Vector3<T>) -> Self {
        Self::new(Point3::from(value))
    }
}

impl<T, S> Default for FramedPoint<T, S>
where
    T: Scalar + Default + Zero,
{
    fn default() -> Self {
        Self {
            point: Default::default(),
            frame: Default::default(),
        }
    }
}

impl<T, S> Deref for FramedPoint<T, S>
where
    T: Scalar,
{
    type Target = Point3<T>;

    fn deref(&self) -> &Self::Target {
        &self.point
    }
}

impl<T, F> DerefMut for FramedPoint<T, F>
where
    T: Scalar,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.point
    }
}

impl BodyPoint<f64> {
    pub fn to_imu_point(&self, body_to_imu: &IsometryMatrix3<f64>) -> ImuPoint<f64> {
        (body_to_imu * self.point).into()
    }
}

impl ImuPoint<f64> {
    pub fn to_world_point(&self, imu_to_world: &IsometryMatrix3<f64>) -> WorldPoint<f64> {
        (imu_to_world * self.point).into()
    }
}
