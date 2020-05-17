use crate::shapes::support;
use crate::shapes::Shape;
use crate::simplex::Simplex;
use crate::simplex::Winding;
use crate::{Point, Vector};

use nalgebra::zero;

const TOLERANCE: f32 = 0.00001;

struct Edge {
    distance: f32,
    normal: Vector<f32>,
    index: usize,
}

pub fn solve(
    a: (&impl Shape, Point<f32>),
    b: (&impl Shape, Point<f32>),
    simplex: Simplex,
) -> (Vector<f32>, f32) {
    let (v, d, _) = solve_internal(a, b, simplex);
    (v, d)
}

pub fn solve_internal(
    a: (&impl Shape, Point<f32>),
    b: (&impl Shape, Point<f32>),
    mut simplex: Simplex,
) -> (Vector<f32>, f32, Vec<Vector<f32>>) {
    if let Simplex::Line(from, to) = simplex {
        let support = support(a, b, perp(to - from, Winding::Left));
        simplex.add(support);
    }
    let winding = simplex.winding();
    let mut simplex: Vec<_> = simplex.into_iter().collect();
    loop {
        let edge = find_closest_edge(&simplex, winding);
        let support = support(a, b, edge.normal);
        let depth = support.dot(&edge.normal);
        if depth - edge.distance < TOLERANCE {
            return (edge.normal, depth, simplex);
        } else {
            simplex.insert(edge.index, support);
        }
    }
}

fn find_closest_edge(simplex: &[Vector<f32>], winding: Winding) -> Edge {
    let mut closest = Edge {
        distance: ::std::f32::MAX,
        normal: zero(),
        index: 0,
    };
    for i in 0..simplex.len() {
        let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
        let a = simplex[i];
        let b = simplex[j];
        let edge = b - a;
        let normal = perp(edge, winding).normalize();
        let dist = normal.dot(&a);
        if dist < closest.distance {
            closest.distance = dist;
            closest.normal = normal;
            closest.index = j;
        }
    }
    closest
}

fn perp(vector: Vector<f32>, winding: Winding) -> Vector<f32> {
    if let Winding::Right = winding {
        Vector::new(-vector.y, vector.x)
    } else {
        Vector::new(vector.y, -vector.x)
    }
}
