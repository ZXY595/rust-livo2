use std::{
    borrow::Borrow,
    iter::Sum,
    ops::{Add, Deref, DerefMut},
};

use nalgebra::{
    ArrayStorage, ClosedAddAssign, ClosedMulAssign, Const, Matrix, SMatrix, Scalar,
    Storage, ViewStorage,
};
use num_traits::{One, Zero};

pub type Uncertainty1<T> = Uncertainty<T, 1>;
pub type Uncertainty2<T> = Uncertainty<T, 2>;
pub type Uncertainty3<T> = Uncertainty<T, 3>;
pub type Uncertainty6<T> = Uncertainty<T, 6>;
pub type Uncertainty9<T> = Uncertainty<T, 9>;
pub type UncertaintyView<'a, T, const D: usize, const RS: usize = 1, const CS: usize = D> =
    Uncertainty<T, D, ViewStorage<'a, T, Const<D>, Const<D>, Const<RS>, Const<CS>>>;

/// Representing the relative uncertainties
pub trait Uncertain {
    type Uncertainties;
}

#[derive(Debug, Clone)]
pub struct Uncertainty<T, const D: usize, S = ArrayStorage<T, D, D>>(
    Matrix<T, Const<D>, Const<D>, S>,
);

// impl<T, S> Uncertainty<T, 1, S>
// where
//     T: Scalar + Clone,
//     S: Storage<T, U1, U1> + IsContiguous,
// {
//     pub fn inner(&self) -> T {
//         self.0.x.clone()
//     }
// }

impl<T, const D: usize> Uncertainty<T, D>
where
    T: Scalar,
{
    pub fn from_element(cov: T) -> Self {
        Self(SMatrix::<T, D, D>::from_element(cov))
    }

    pub fn from_element_with_len(cov: T, len: Const<D>) -> Self {
        let _ = len;
        Self(SMatrix::<T, D, D>::from_element(cov))
    }
}

impl<T, const D: usize, S> Deref for Uncertainty<T, D, S> {
    type Target = Matrix<T, Const<D>, Const<D>, S>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const D: usize, S> DerefMut for Uncertainty<T, D, S> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const D: usize, S> Uncertainty<T, D, S>
where
    T: Scalar + Zero + One + ClosedAddAssign + ClosedMulAssign,
    S: Storage<T, Const<D>, Const<D>>,
{
    #[inline]
    pub fn forward<const R: usize>(
        &self,
        error: impl Borrow<SMatrix<T, R, D>>,
    ) -> Uncertainty<T, R> {
        let error: &SMatrix<T, R, D> = error.borrow();
        Uncertainty(error * &self.0 * error.transpose())
    }

    pub fn back(&self, error: impl Borrow<SMatrix<T, D, 1>>) -> T {
        let error: &SMatrix<T, D, 1> = error.borrow();
        (error.transpose() * &self.0 * error).x.clone()
    }
}

impl<T, const D: usize, S1, S2> Add<Uncertainty<T, D, S2>> for Uncertainty<T, D, S1>
where
    T: Scalar + ClosedAddAssign,
    S1: Storage<T, Const<D>, Const<D>>,
    S2: Storage<T, Const<D>, Const<D>>,
{
    type Output = Uncertainty<T, D, ArrayStorage<T, D, D>>;

    fn add(self, rhs: Uncertainty<T, D, S2>) -> Self::Output {
        Uncertainty(self.0 + rhs.0)
    }
}

impl<T, const D: usize, S> From<Matrix<T, Const<D>, Const<D>, S>> for Uncertainty<T, D, S> {
    fn from(value: Matrix<T, Const<D>, Const<D>, S>) -> Self {
        Self(value)
    }
}

impl<T, const D: usize> Zero for Uncertainty<T, D>
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

impl<T, const D: usize> Sum for Uncertainty<T, D>
where
    T: Scalar + ClosedAddAssign + Zero,
{
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.reduce(|acc, item| acc + item)
            .unwrap_or_else(Self::zero)
    }
}
