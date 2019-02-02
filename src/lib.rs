use nalgebra as na;

use std::fmt::Debug;
use std::ops::Neg;

pub mod gjk;
pub mod epa;
pub mod simplex;
pub mod shapes;

pub type Vector<T> = na::Vector2<T>;

pub type Point<T> = na::Point2<T>;

pub trait Perp {
    fn perpendicular(&self) -> Self;
}

impl<F> Perp for Vector<F>
where
    F: Copy + PartialEq + Neg<Output = F> + Debug + 'static,
{
    fn perpendicular(&self) -> Self {
        Vector::new(-self.y, self.x)
    }
}
