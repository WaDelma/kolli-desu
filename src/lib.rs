#[macro_use]
extern crate serde_derive;

use nalgebra as na;

use std::fmt::Debug;
use std::ops::Neg;

pub mod epa;
pub mod gjk;
pub mod shapes;
pub mod simplex;

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

fn det(a: Vector<f32>, b: Vector<f32>, p: Vector<f32>) -> f32 {
    (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x)
}
