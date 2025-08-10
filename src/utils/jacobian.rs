use std::{
    borrow::Borrow,
    iter::Sum,
    ops::{Add, Deref},
};

use nalgebra::{
    ArrayStorage, ClosedAddAssign, ClosedMulAssign, Const, IsContiguous, Matrix, RawStorage,
    SMatrix, Scalar, Storage, U1, U3, ViewStorage,
};
use num_traits::{One, Zero};

pub type Covariance1<T> = Covariance<T, 1>;
pub type Covariance2<T> = Covariance<T, 2>;
pub type Covariance3<T> = Covariance<T, 3>;
pub type Covariance6<T> = Covariance<T, 6>;
pub type Covariance3View<'a, T, const R: usize, const C: usize> =
    Covariance<T, 3, ViewStorage<'a, T, U3, U3, Const<R>, Const<C>>>;

#[derive(Debug, Clone)]
pub struct Covariance<T, const D: usize, S = ArrayStorage<T, D, D>>(
    pub Matrix<T, Const<D>, Const<D>, S>,
);

impl<T, S> Covariance<T, 1, S>
where
    T: Scalar + Clone,
    S: RawStorage<T, U1, U1> + IsContiguous,
{
    pub fn inner(&self) -> T {
        self.0.x.clone()
    }
}

impl<T, const D: usize> Covariance<T, D>
where
    T: Scalar,
{
    pub fn from_element(cov: T) -> Self {
        Self(SMatrix::<T, D, D>::from_element(cov))
    }
}

impl<T, const D: usize, S> Deref for Covariance<T, D, S> {
    type Target = Matrix<T, Const<D>, Const<D>, S>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const D: usize, S> Covariance<T, D, S>
where
    T: Scalar + Zero + One + ClosedAddAssign + ClosedMulAssign,
    S: Storage<T, Const<D>, Const<D>>,
{
    #[inline]
    pub fn propagate_error<const R: usize>(
        &self,
        error: impl Borrow<SMatrix<T, R, D>>,
    ) -> Covariance<T, R> {
        let error: &SMatrix<T, R, D> = error.borrow();
        Covariance(error * &self.0 * error.transpose())
    }

    #[inline]
    pub fn project_on(&self, error: impl Borrow<SMatrix<T, D, 1>>) -> T {
        let error: &SMatrix<T, D, 1> = error.borrow();
        (error.transpose() * &self.0 * error).x.clone()
    }
}

impl<T, const D: usize, S1, S2> Add<Covariance<T, D, S2>> for Covariance<T, D, S1>
where
    T: Scalar + ClosedAddAssign,
    S1: Storage<T, Const<D>, Const<D>>,
    S2: Storage<T, Const<D>, Const<D>>,
{
    type Output = Covariance<T, D, ArrayStorage<T, D, D>>;

    fn add(self, rhs: Covariance<T, D, S2>) -> Self::Output {
        Covariance::from(self.0 + rhs.0)
    }
}

impl<T, const D: usize, S> From<Matrix<T, Const<D>, Const<D>, S>> for Covariance<T, D, S> {
    fn from(value: Matrix<T, Const<D>, Const<D>, S>) -> Self {
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
