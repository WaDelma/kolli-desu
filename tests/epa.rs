use kolli_desu::Perp;
use std::fmt::Debug;

use nalgebra::Isometry2;

use kolli_desu::epa::solve;
use kolli_desu::gjk::collides_internal;
use kolli_desu::shapes::{Aabb, Circle, ConvexPolygon, Shape};
use kolli_desu::{Point, Vector};

const TAU: f32 = 2. * ::std::f32::consts::PI;

#[test]
fn circle_circle_penetration_multi() {
    let circle = Circle::new(Point::new(0., 0.), 0.50001);
    let steps = 360;
    for n in 0..steps {
        let other = Circle::new(
            Isometry2::new(Vector::new(0., 0.), n as f32 * TAU / steps as f32) * Point::new(1., 0.),
            0.5,
        );
        let (_, simplex) =
            collides_internal((&circle, Point::new(0., 0.)), (&other, Point::new(0., 0.)));
        let (vector, depth) = solve(
            (&circle, Point::new(0., 0.)),
            (&other, Point::new(0., 0.)),
            simplex,
        );
        assert!(
            (depth - 0.00001).abs() < 0.00001,
            "Depth wasn't close enough to 0.00001: {}",
            depth
        );
        let diff = (vector - other.center.coords).norm();
        assert!(
            diff < 0.005,
            "Penetration vector {} wasn't close enough to {}: {}",
            other.center,
            vector,
            diff
        );
    }
}

#[test]
fn same_circle() {
    let circle = Circle::new(Point::new(0., 0.), 0.5);
    let (_, simplex) =
        collides_internal((&circle, Point::new(0., 0.)), (&circle, Point::new(0., 0.)));
    let (_, depth) = solve(
        (&circle, Point::new(0., 0.)),
        (&circle, Point::new(0., 0.)),
        simplex,
    );
    assert!(
        (depth - 1.) < 0.00001,
        "Depth wasn't close enough to 1: {}",
        depth
    );
}

#[test]
fn small_circle_penetrates_bigger_one() {
    let circle1 = Circle::new(Point::new(0., 0.), 0.5);
    let steps = 100;
    for x in 0..=steps {
        let percentage = (x as f32) / steps as f32;
        let circle2 = Circle::new(Point::new(-0.5 + percentage, 0.), 0.1);
        let (_, simplex) = collides_internal(
            (&circle1, Point::new(0., 0.)),
            (&circle2, Point::new(0., 0.)),
        );
        let (_, depth) = solve(
            (&circle1, Point::new(0., 0.)),
            (&circle2, Point::new(0., 0.)),
            simplex,
        );
        let correct = 0.5 - (percentage - 0.5).abs() + 0.1;
        assert!(
            (depth - correct).abs() < 0.00001,
            "Depth wasn't close enough to {}: {}",
            correct,
            depth
        );
    }
}
