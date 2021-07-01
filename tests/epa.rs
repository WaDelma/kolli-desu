use kolli_desu::simplex::Simplex;

use nalgebra::Isometry2;

use kolli_desu::epa::solve;
use kolli_desu::gjk::collides_internal;
use kolli_desu::shapes::{Aabb, Circle};
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

#[test]
fn reproduction_of_infinite_loop_original() {
    let h1 = Circle::new(Point::new(0.0, 0.0), 0.25);
    let p1 = Point::new(734.13696, 750.62823);
    let h2 = Aabb::new(Point::new(-0.75, -0.75), Point::new(0.75, 0.75));
    let p2 = Point::new(735.0, 751.5);

    let (_, s) = collides_internal((&h1, p1), (&h2, p2));
    solve((&h1, p1), (&h2, p2), s);
}

#[test]
fn reproduction_of_infinite_loop_centered() {
    let m = Vector::new(1., 1.) * 0.;
    let h1 = Circle::new(Point::new(256.136, 272.628) - m, 0.25);
    let p1 = Point::new(0., 0.);
    let h2 = Aabb::new(
        Point::new(257.0 - 0.75, 273.5 - 0.75) - m,
        Point::new(257.0 + 0.75, 273.5 + 0.75) - m,
    );
    let p2 = Point::new(0., 0.);

    let (_, s) = collides_internal((&h1, p1), (&h2, p2));
    solve((&h1, p1), (&h2, p2), s);
}

#[test]
fn reproduction_of_support_was_not_between_adjanced_vectors() {
    let h1 = Aabb::new(Point::new(-0.75, -0.75), Point::new(0.75, 0.75));
    let p1 = Point::new(739.5, 814.5);
    let h2 = Circle::new(Point::new(0.0, 0.0), 0.25);
    let p2 = Point::new(739.597, 813.5169);
    let s = Simplex::Triangle(
        Vector::new(0.67755127, -0.015686035),
        Vector::new(-1.0445557, 0.07989502),
        Vector::new(-0.86083984, -0.016540527),
    );
    solve((&h1, p1), (&h2, p2), s);
}
