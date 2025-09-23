use std::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use nalgebra::{IsometryMatrix3, Point3, Scalar, SimdRealField, Vector3};
use num_traits::Zero;

pub struct World {}
pub struct Imu {}
/// also known as the frame where you collect point cloud
pub struct Body {}

pub type WorldPoint<T> = FramedPoint<T, World>;
pub type ImuPoint<T> = FramedPoint<T, Imu>;
pub type BodyPoint<T> = FramedPoint<T, Body>;

pub type FramedPoint<T, F> = Framed<Point3<T>, F>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Framed<T, F> {
    pub inner: T,
    frame: PhantomData<F>,
}

impl<T, F> Deref for Framed<T, F> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T, F> DerefMut for Framed<T, F> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T, F> Framed<T, F> {
    pub const fn new(inner: T) -> Self {
        Self {
            inner,
            frame: PhantomData,
        }
    }
    pub fn new_with_frame(inner: T, frame: F) -> Self {
        let _ = frame;
        Self::new(inner)
    }
}

impl<T, F> From<Point3<T>> for FramedPoint<T, F>
where
    T: Scalar,
{
    fn from(value: Point3<T>) -> Self {
        Self::new(value)
    }
}

impl<T, F> From<Vector3<T>> for FramedPoint<T, F>
where
    T: Scalar,
{
    fn from(value: Vector3<T>) -> Self {
        Self::new(Point3::from(value))
    }
}

impl<T, F, To> From<IsometryMatrix3<T>> for Framed<IsometryMatrix3<T>, fn(F) -> To>
where
    T: Scalar,
{
    fn from(value: IsometryMatrix3<T>) -> Self {
        Self::new(value)
    }
}

impl<T, F> Default for FramedPoint<T, F>
where
    T: Scalar + Default + Zero,
{
    fn default() -> Self {
        Self {
            inner: Default::default(),
            frame: Default::default(),
        }
    }
}

impl<T, F> FramedPoint<T, F>
where
    T: SimdRealField,
    T::Element: SimdRealField,
{
    pub fn transform_with_isometry<To>(
        &self,
        tf: &Framed<IsometryMatrix3<T>, fn(F) -> To>,
    ) -> FramedPoint<T, To> {
        (tf.deref() * self.deref()).into()
    }
}

impl BodyPoint<f64> {
    pub fn to_imu_point(
        &self,
        body_to_imu: &Framed<IsometryMatrix3<f64>, fn(Body) -> Imu>,
    ) -> ImuPoint<f64> {
        (body_to_imu.deref() * self.deref()).into()
    }
}

impl ImuPoint<f64> {
    pub fn to_world_point(
        &self,
        imu_to_world: &Framed<IsometryMatrix3<f64>, fn(Imu) -> World>,
    ) -> WorldPoint<f64> {
        (imu_to_world.deref() * self.deref()).into()
    }
}
