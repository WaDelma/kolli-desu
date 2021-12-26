use crate::shapes::support;
use crate::shapes::Shape;
use crate::simplex::Simplex;
use crate::simplex::Winding;
use crate::{det, Point, Vector};

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
        let support = support(a, b, perp(to - from, Winding::Right));
        simplex.add(support);
    }
    let winding = simplex.winding();
    let mut simplex: Vec<_> = simplex.into_iter().collect();
    loop {
        let edge = find_closest_edge(&simplex, winding, a, b);
        let sup = support(a, b, edge.normal);
        let depth = sup.dot(&edge.normal);
        if depth - edge.distance < TOLERANCE {
            return (edge.normal, depth, simplex);
        } else {
            simplex.insert(edge.index, sup);
            debug_assert!(is_convex(&simplex), "Simplex should be always convex");
        }
    }
}

fn is_convex(simplex: &[Vector<f32>]) -> bool {
    let len = simplex.len();
    let mut sign = None;
    for i in 0..len {
        let j = (i + 1) % len;
        let k = (i + 2) % len;
        let det = det(simplex[i], simplex[j], simplex[k]);
        let sign = *sign.get_or_insert(det);
        if sign * det < 0. {
            return false;
        }
    }
    true
}

fn find_closest_edge(
    simplex: &[Vector<f32>],
    winding: Winding,
    aa: (&impl Shape, Point<f32>),
    bb: (&impl Shape, Point<f32>),
) -> Edge {
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

        // TODO: Here we check that the support for the normal is between vectors.
        // Without this there are infinite loops... However even with this `is_convex` still fails.
        let s = support(aa, bb, normal);
        let before = det(zero(), s, a);
        let after = det(zero(), s, b);
        if before.signum() != after.signum() {
            if dist < closest.distance {
                closest.distance = dist;
                closest.normal = normal;
                closest.index = j;
            }
        }
    }
    closest
}

fn perp(vector: Vector<f32>, winding: Winding) -> Vector<f32> {
    if let Winding::Right = winding {
        Vector::new(vector.y, -vector.x)
    } else {
        Vector::new(-vector.y, vector.x)
    }
}
