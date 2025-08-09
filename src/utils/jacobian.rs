use std::{
    iter::Sum,
    ops::{Add, AddAssign, Deref},
};

use nalgebra::{ClosedAddAssign, ClosedMulAssign, SMatrix, Scalar};
use num_traits::{One, Zero};

pub type Covariance1<T> = Covariance<T, 1>;
pub type Covariance2<T> = Covariance<T, 2>;
pub type Covariance3<T> = Covariance<T, 3>;
pub type Covariance6<T> = Covariance<T, 6>;

#[derive(Debug, Clone, PartialEq)]
pub struct Covariance<T, const D: usize>(pub SMatrix<T, D, D>);

impl<T, const D: usize> Covariance<T, D>
where
    T: Scalar,
{
    pub fn from_element(cov: T) -> Self {
        Self(SMatrix::<T, D, D>::from_element(cov))
    }
}

impl<T, const D: usize> Deref for Covariance<T, D> {
    type Target = SMatrix<T, D, D>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const D: usize> Covariance<T, D>
where
    T: Scalar + Zero + One + ClosedAddAssign + ClosedMulAssign,
{
    pub fn propagate_error<const R: usize>(&self, error: &SMatrix<T, R, D>) -> Covariance<T, R> {
        Covariance(error * &self.0 * error.transpose())
    }
}

impl<T, const D: usize> Add for Covariance<T, D>
where
    T: Scalar + ClosedAddAssign,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

// impl<T, const D: usize> AddAssign<SMatrix<T, D, D>> for Covariance<T, D>
// where
//     T: Scalar + ClosedAddAssign,
// {
//     fn add_assign(&mut self, rhs: SMatrix<T, D, D>) {
//         self.0 += rhs;
//     }
// }

impl<T, const D: usize> From<SMatrix<T, D, D>> for Covariance<T, D> {
    fn from(value: SMatrix<T, D, D>) -> Self {
        Self(value)
    }
}

impl<T, const D: usize> Zero for Covariance<T, D>
where
    T: Scalar + ClosedAddAssign + Zero,
{
    fn zero() -> Self {
        Self(SMatrix::<T, D, D>::zero())
    }

    fn is_zero(&self) -> bool {
        self.0.is_zero()
    }
}

impl<T, const D: usize> Sum for Covariance<T, D>
where
    T: Scalar + ClosedAddAssign,
{
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.reduce(|acc, item| acc + item)
            .expect("Cannot compute `sum` of empty iterator.")
    }
}
