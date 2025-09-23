use std::borrow::Borrow;

use nalgebra::{
    ClosedAddAssign, ClosedMulAssign, DefaultAllocator, Dim, DimAdd, DimName, DimSum, Matrix,
    OMatrix, SMatrix, Scalar, SquareMatrix, Storage, allocator::Allocator,
};
use num_traits::{One, Zero};

pub trait Uncertainty {
    type Element: Scalar;
    type Dim: DimName;
}

pub trait UncertainForward<R>:
    Uncertainty<Element: Zero + One + ClosedAddAssign + ClosedMulAssign>
where
    R: Dim,
    DefaultAllocator: Allocator<R, Self::Dim> + Allocator<Self::Dim, R> + Allocator<R, R>,
{
    fn forward<S>(
        &self,
        error: impl Borrow<Matrix<Self::Element, R, Self::Dim, S>>,
    ) -> OMatrix<Self::Element, R, R>
    where
        S: Storage<Self::Element, R, Self::Dim>;

    fn backward<S>(
        &self,
        error: impl Borrow<Matrix<Self::Element, Self::Dim, R, S>>,
    ) -> OMatrix<Self::Element, R, R>
    where
        S: Storage<Self::Element, Self::Dim, R>;
}

impl<T, D, S> Uncertainty for SquareMatrix<T, D, S>
where
    T: Scalar,
    D: DimName,
{
    type Element = T;
    type Dim = D;
}

impl<T, D, R, S1> UncertainForward<R> for SquareMatrix<T, D, S1>
where
    T: Scalar + Zero + One + ClosedAddAssign + ClosedMulAssign,
    D: DimName,
    R: Dim,
    S1: Storage<T, D, D>,
    DefaultAllocator: Allocator<R, Self::Dim> + Allocator<Self::Dim, R> + Allocator<R, R>,
{
    fn forward<S2>(
        &self,
        error: impl Borrow<Matrix<Self::Element, R, Self::Dim, S2>>,
    ) -> OMatrix<Self::Element, R, R>
    where
        S2: Storage<Self::Element, R, Self::Dim>,
    {
        let error = error.borrow();
        error * self * error.transpose()
    }
    fn backward<S2>(
        &self,
        error: impl Borrow<Matrix<Self::Element, Self::Dim, R, S2>>,
    ) -> OMatrix<Self::Element, R, R>
    where
        S2: Storage<Self::Element, Self::Dim, R>,
    {
        let error = error.borrow();
        error.transpose() * self * error
    }
}

pub trait UncertaintyAdd<T> {
    type Output;
}

impl<L, R> UncertaintyAdd<L> for R
where
    L: Uncertainty<Dim: DimAdd<R::Dim>>,
    R: Uncertainty<Element = L::Element>,
    DefaultAllocator: Allocator<DimSum<L::Dim, R::Dim>, DimSum<L::Dim, R::Dim>>,
{
    type Output = OMatrix<L::Element, DimSum<L::Dim, R::Dim>, DimSum<L::Dim, R::Dim>>;
}

pub type Uncertainty1<T> = SMatrix<T, 1, 1>;
pub type Uncertainty2<T> = SMatrix<T, 2, 2>;
pub type Uncertainty3<T> = SMatrix<T, 3, 3>;
