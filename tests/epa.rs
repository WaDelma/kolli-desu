
use kolli_desu::Perp;
use std::fmt::Debug;

use nalgebra::Isometry2;

use kolli_desu::{Point, Vector};
use kolli_desu::gjk::{collides_internal};
use kolli_desu::epa::{solve};
use kolli_desu::shapes::{Circle, ConvexPolygon, Aabb, Shape};

const TAU: f32 = 2. * ::std::f32::consts::PI;

#[test]
fn circle_circle_penetration_multi() {
    let circle = Circle::new(Point::new(0., 0.), 0.55);
    let steps = 360;
    for n in 0..steps {
        let other = Circle::new(Isometry2::new(Vector::new(0., 0.), n as f32 * TAU / steps as f32) * Point::new(1., 0.), 0.5);
        let (_, simplex) = collides_internal((&circle, Point::new(0., 0.)), (&other, Point::new(0., 0.)));
        let (vector, depth) = solve((&circle, Point::new(0., 0.)), (&other, Point::new(0., 0.)), simplex);
        assert!((depth - 0.05).abs() < 0.001, "Depth wasn't close enough to 0.05: {}", depth);
        let diff = (vector - other.center.coords).norm();
        assert!(diff < 0.01, "Penetration vector {} wasn't close enough to {}: {}", other.center, vector, diff);
    }
}