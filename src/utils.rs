use std::iter::Sum;

use nalgebra::{Matrix3, Point3, Vector3};

pub mod jacobian;

#[derive(Default)]
pub struct PointCovSum {
    pub num: usize,
    pub mean: Vector3<f64>,
    pub covariance: Matrix3<f64>,
}

impl<'a> Sum<&'a Point3<f64>> for PointCovSum {
    fn sum<I: Iterator<Item = &'a Point3<f64>>>(iter: I) -> Self {
        iter.fold(Self::default(), |mut acc, current| {
            let current = current.coords;
            acc.num += 1;
            acc.mean += current;
            acc.covariance += current * current.transpose();
            acc
        })
    }
}
