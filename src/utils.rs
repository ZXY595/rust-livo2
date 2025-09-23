use std::iter::Sum;

use nalgebra::{Matrix3, Vector3};

#[derive(Default)]
pub struct VectorSquareSum {
    count: usize,
    sum: Vector3<f64>,
    square_sum: Matrix3<f64>,
}

impl VectorSquareSum {
    pub fn mean(&self) -> (Vector3<f64>, Matrix3<f64>) {
        let count = self.count as f64;
        let mean = self.sum / count;
        (mean, self.square_sum / count - mean * mean.transpose())
    }
    pub fn count(&self) -> usize {
        self.count
    }
}

impl<'a> Sum<&'a Vector3<f64>> for VectorSquareSum {
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = &'a Vector3<f64>>,
    {
        iter.fold(Self::default(), |mut acc, current| {
            acc.count += 1;
            acc.sum += current;
            acc.square_sum += current * current.transpose();
            acc
        })
    }
}
