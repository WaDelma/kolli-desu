extern crate kolli_desu;
extern crate nalgebra;

// use kolli_desu::Hitbox;
// use kolli_desu::Perp;
// use kolli_desu::{line_line_intersection_point, LineIntersectError};
use std::fmt::Debug;

use nalgebra::Isometry2;

use kolli_desu::{Point, Vector};
use kolli_desu::gjk::{collides};
use kolli_desu::shapes::{Circle, Aabb, Shape};

const TAU: f32 = 2. * ::std::f32::consts::PI;

fn zero() -> Point<f32> {
    Point::from_coordinates(::nalgebra::zero())
}

fn assert_collides<S1, S2>(hitbox1: &S1, hitbox2: &S2) 
    where
        S1: Debug+Shape,
        S2: Debug+Shape,
{
    let lamppa = |pos| {
        assert!(collides((hitbox1, pos), (hitbox2, pos)), "{:?} == {:?}, {}", hitbox1, hitbox2, pos);
        assert!(collides((hitbox2, pos), (hitbox1, pos)), "{:?} == {:?}, {}", hitbox2, hitbox1, pos);
        assert!(collides((hitbox1, pos), (hitbox1, pos)), "{:?} == {:?}, {}", hitbox1, hitbox1, pos);
        assert!(collides((hitbox2, pos), (hitbox2, pos)), "{:?} == {:?}, {}", hitbox2, hitbox2, pos);
    };
    lamppa(zero());
    lamppa(Point::new(1000., 1000.));
}

fn assert_not_collides<S1, S2>(hitbox1: &S1, hitbox2: &S2)
    where
        S1: Debug+Shape,
        S2: Debug+Shape,
{
    let lamppa = |pos| {
        assert!(!collides((hitbox1, pos), (hitbox2, pos)), "{:?} != {:?}, {}", hitbox1, hitbox2, pos);
        assert!(!collides((hitbox2, pos), (hitbox1, pos)), "{:?} != {:?}, {}", hitbox2, hitbox1, pos);
    };
    lamppa(zero());
    lamppa(Point::new(1000., 1000.));
}

#[test]
fn circle_circle_collides_multi() {
    let circle = Circle::new(Vector::new(0., 0.), 0.55);
    let mut other = Circle::new(Vector::new(1., 0.), 0.5);
    let steps = 360;
    for _ in 0..steps {
        other.center = Isometry2::new(Vector::new(0., 0.), TAU / steps as f32) * other.center;
        assert_collides(&circle, &other);
    }
}

#[test]
fn circle_circle_doesnt_collide_multi() {
    let circle = Circle::new(Vector::new(0., 0.), 0.45);
    let mut other = Circle::new(Vector::new(1., 0.), 0.5);
    let steps = 360;
    for _ in 0..steps {
        other.center = Isometry2::new(Vector::new(0., 0.), TAU / steps as f32) * other.center;
        assert_not_collides(&circle, &other);
    }
}


#[test]
fn circle_aabb_collides_multi() {
    let aabb = Aabb::new(Vector::new(-0.5, -0.5), Vector::new(0.5, 0.5));
    let mut other = Circle::new(Vector::new(1., 0.), 0.55);
    let steps = 360;
    for _ in 0..steps {
        other.center = Isometry2::new(Vector::new(0., 0.), TAU / steps as f32) * other.center;
        assert_collides(&aabb, &other);
    }
}

#[test]
fn circlee_aabb_doesnt_collide_multi() {
    let aabb = Aabb::new(Vector::new(-0.5, -0.5), Vector::new(0.5, 0.5));
    let mut other = Circle::new(Vector::new(1., 0.), 0.28);
    let steps = 360;
    for _ in 0..steps {
        other.center = Isometry2::new(Vector::new(0., 0.), TAU / steps as f32) * other.center;
        assert_not_collides(&aabb, &other);
    }
}

// #[test]
// fn enclosing_aabb_with_circle() {
//     let circle = Hitbox::circle(Vector::new(0.5, 0.5), 0.5);
//     let enclosing_aabb = Hitbox::Aabb(circle.enclosing_aabb());
//     assert_eq!(Hitbox::aabb(Vector::new(0.5, 0.5), 1., 1.), enclosing_aabb);
// }

// #[test]
// fn enclosing_aabb_with_aabb() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 2., 2.);
//     let enclosing_aabb = Hitbox::Aabb(aabb.enclosing_aabb());
//     assert_eq!(aabb, enclosing_aabb);
// }

// #[test]
// fn enclosing_aabb_with_line_segment() {
//     let line_segment = Hitbox::line_segment(Vector::new(0., 0.), Vector::new(1., 1.));
//     let enclosing_aabb = Hitbox::Aabb(line_segment.enclosing_aabb());
//     assert_eq!(Hitbox::aabb(Vector::new(0.5, 0.5), 1., 1.), enclosing_aabb);
// }

// #[test]
// fn enclosing_aabb_with_rectangle() {
//     use std::f32::consts::FRAC_1_SQRT_2;
//     let line_segment = Hitbox::rectangle(Vector::new(0.5, 0.), Vector::new(1., 0.5), FRAC_1_SQRT_2);
//     let enclosing_aabb = Hitbox::Aabb(line_segment.enclosing_aabb());
//     assert_eq!(Hitbox::aabb(Vector::new(0.5, 0.5), 1., 1.), enclosing_aabb);
// }

#[test]
fn circle_circle_offset_non_collision() {
    let circle = Circle::new(Vector::new(0., 0.), 1.);
    assert!(!collides(
        (&circle, zero()),
        (&circle, Point::new(1000., 1000.))
    ));
}

// #[test]
// fn circle_circle_collision() {
//     assert!(collides(
//         (Circle::new(Vector::new(1., 0.), 1.1), Point::new(0., 0.)),
//         (Circle::new(Vector::new(-1., 0.), 1.1), Point::new(0., 0.)),
//     ));
// }

#[test]
fn circle_circle_non_collision() {
    assert!(!collides(
        (&Circle::new(Vector::new(-1., 0.), 0.9), Point::new(0., 0.)),
        (&Circle::new(Vector::new(1., 0.), 0.9), Point::new(0., 0.)),
    ));
}

#[test]
fn circle_aabb_offset_non_collision() {
    let circle = Circle::new(Vector::new(0., 0.), 1.);
    let aabb = Aabb::new(Vector::new(-0.5, -0.5), Vector::new(0.5, 0.5));
    assert!(!collides(
        (&circle, zero()),
        (&aabb, Point::new(1000., 1000.))
    ));
    assert!(!collides(
        (&circle, Point::new(1000., 1000.)),
        (&aabb, zero())
    ));
}

#[test]
fn aabb_aabb_offset_non_collision() {
    let aabb = Aabb::new(Vector::new(-0.5, -0.5), Vector::new(0.5, 0.5));
    assert!(!collides(
        (&aabb, zero()),
        (&aabb, Point::new(1000., 1000.))
    ));
}

#[test]
fn rectangle_rectangle_offset_non_collision() {
    let rectangle = Rectangle::new(Vector::new(0., 0.), Vector::new(1., 0.), 1.);
    assert!(!collides(
        (&rectangle, zero()),
        (&rectangle, Point::new(1000., 1000.))
    ));
}

// #[test]
// fn aabb_rectangle_offset_non_collision() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 1.);
//     assert!(!Hitbox::collides(
//         (&rectangle, zero()),
//         (&aabb, Point::new(1000., 1000.))
//     ));
//     assert!(!Hitbox::collides(
//         (&aabb, zero()),
//         (&rectangle, Point::new(1000., 1000.))
//     ));
// }

// #[test]
// fn rectangle_dot_offset_non_collision() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 1.);
//     let point = Hitbox::dot(Vector::new(0., 0.5));
//     assert!(!Hitbox::collides(
//         (&rectangle, zero()),
//         (&point, Point::new(1000., 1000.))
//     ));
//     assert!(!Hitbox::collides(
//         (&point, zero()),otation2
//         (&rectangle, Point::new(1000., 1000.))
//     ));
// }

// #[test]
// fn line_segment_line_segment_non_collision() {
//     let ls1 = Hitbox::line_segment(Vector::new(-1., -1.), Vector::new(1., 1.));
//     let ls2 = Hitbox::line_segment(Vector::new(1., -1.), Vector::new(-1., 1.));
//     assert!(!Hitbox::collides(
//         (&ls1, zero()),
//         (&ls2, Point::new(1000., 1000.))
//     ));
//     assert!(!Hitbox::collides(
//         (&ls2, zero()),
//         (&ls1, Point::new(1000., 1000.))
//     ));
// }

// #[test]
// fn aabb_center() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     assert_eq!(aabb.center(), Point::new(0., 0.));
// }

// #[test]
// fn rectangle_center() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     assert!((rectangle.center() - Point::new(0., 1.)).norm() < 0.0000001);
// }

// #[test]
// fn line_segment_center() {
//     let line_segment = Hitbox::line_segment(Vector::new(0., 0.), Vector::new(1., 1.));
//     assert!((line_segment.center() - Point::new(0.5, 0.5)).norm() < 0.0000001);
// }

// #[test]
// fn dot_center() {
//     let dot = Hitbox::dot(Vector::new(100., 100.));
//     assert_eq!(dot.center(), Point::new(100., 100.));
// }

#[test]
fn circle_circle_collision() {
    let circle1 = Circle::new(Vector::new(0., 0.), 1.);
    let circle2 = Circle::new(Vector::new(1.9, 0.), 1.);
    let circle3 = Circle::new(Vector::new(2., 2.), 1.);

    assert_collides(&circle1, &circle2);

    assert_not_collides(&circle1, &circle3);
    assert_not_collides(&circle2, &circle3);
}

// #[test]
// fn aabb_aabb_collision() {
//     let aabb1 = Hitbox::aabb(Vector::new(0.5, 0.5), 1., 1.);
//     let aabb2 = Hitbox::aabb(Vector::new(-0.5, -0.5), 1., 1.);
//     let aabb3 = Hitbox::aabb(Vector::new(1.6, 2.6), 1., 2.);
//     let aabb4 = Hitbox::aabb(Vector::new(50., 50.), 100., 150.);

//     assert_collides(&aabb1, &aabb2);
//     assert_collides(&aabb1, &aabb4);
//     assert_collides(&aabb2, &aabb4);
//     assert_collides(&aabb3, &aabb4);

//     assert_not_collides(&aabb1, &aabb3);
//     assert_not_collides(&aabb2, &aabb3);
// }

// #[test]
// fn crossing_aabb_aabb_collision() {
//     let aabb1 = Hitbox::aabb(Vector::new(0.0, 0.0), 10., 1.);
//     let aabb2 = Hitbox::aabb(Vector::new(0.0, 0.0), 1., 10.);
//     assert_collides(&aabb1, &aabb2);
// }

// #[test]
// fn circle_aabb_collision() {
//     let aabb1 = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let circle1 = Hitbox::circle(Vector::new(0., 0.), 1.);
//     let aabb2 = Hitbox::aabb(Vector::new(2., 2.), 1., 1.);
//     let circle2 = Hitbox::circle(Vector::new(2., 2.), 1.);

//     assert_collides(&aabb1, &circle1);
//     assert_collides(&aabb2, &circle2);

//     assert_not_collides(&aabb1, &aabb2);
//     assert_not_collides(&aabb2, &circle1);
//     assert_not_collides(&circle2, &aabb1);
//     assert_not_collides(&circle2, &circle1);
// }

// #[test]
// fn rectangle_circle_inside() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(0.5, 0.5), 0.5);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_really_distant() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(100., 100.), 1.);

//     assert_not_collides(&rectangle, &circle);
// }
// // Due to how floating points work, these tests have circle radi set to slightly larger than in other
// // tests.
// #[test]
// fn rectangle_circle_right_side_touching() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(2., 1.), 1.0001);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_left_side_touching() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(-2., 1.), 1.0001);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_up_side_touching() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(0., 3.), 1.0001);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_lower_side_touching() {
//     let rectangle = Hitbox::rectangle(Vector::new(0., 0.), Vector::new(1., 1.), 2f32.sqrt());
//     let circle = Hitbox::circle(Vector::new(0., -1.), 1.);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_close_to_side() {
//     let rectangle = Hitbox::rectangle(Vector::new(1., -1.), Vector::new(2., -1.), 2.);
//     let circle = Hitbox::circle(Vector::new(0., 0.), 1.);

//     assert_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_circle_close_to_side_not_touching() {
//     let rectangle = Hitbox::rectangle(Vector::new(1.001, -1.), Vector::new(2., -1.), 2.);
//     let circle = Hitbox::circle(Vector::new(0., 0.), 1.);

//     assert_not_collides(&rectangle, &circle);
// }

// #[test]
// fn rectangle_aabb_partial_overlap() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let rectangle = Hitbox::rectangle(Vector::new(-1., -1.), Vector::new(0.5, 0.), 1.);

//     assert_collides(&rectangle, &aabb);
// }

// #[test]
// fn rectangle_aabb_left() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let rectangle = Hitbox::rectangle(Vector::new(-1., -1.), Vector::new(-0.4, 0.), 1.);

//     assert_not_collides(&rectangle, &aabb);
// }

// #[test]
// fn rectangle_aabb_above_left() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let rectangle = Hitbox::rectangle(Vector::new(-1., -1.), Vector::new(0., 3.), 1.);

//     assert_not_collides(&rectangle, &aabb);
// }

// #[test]
// fn rectangle_aabb_below() {
//     let aabb = Hitbox::aabb(Vector::new(0., 0.), 1., 1.);
//     let rectangle = Hitbox::rectangle(Vector::new(-4., -4.), Vector::new(1., -4.), 0.2);

//     assert_not_collides(&rectangle, &aabb);
// }

// #[test]
// fn lines_intersecting() {
//     let p1 = Point::new(-1., -1.);
//     let v1 = Vector::new(1., 1.);
//     let p2 = Point::new(1., -1.);
//     let v2 = Vector::new(-1., 1.);
//     assert_eq!(
//         line_line_intersection_point(p1, v1, p2, v2),
//         Ok(Point::new(0., 0.))
//     );
//     assert_eq!(
//         line_line_intersection_point(p2, v2, p1, v1),
//         Ok(Point::new(0., 0.))
//     );
//     let line1 = Hitbox::line(p1.coords, v1);
//     let line2 = Hitbox::line(p2.coords, v2);
//     assert_collides(&line1, &line2);
// }

// #[test]
// fn lines_intersecting_weird() {
//     let p1 = Point::new(-1., -1.);
//     let v1 = Vector::new(1., 1.);
//     let p2 = Point::new(1., -1.);
//     let v2 = Vector::new(-2.8459832, 2.87654942);
//     match line_line_intersection_point(p1, v1, p2, v2) {
//         Ok(_) => {}
//         _ => panic!("lines didn't intersect even thought they should"),
//     }
//     match line_line_intersection_point(p2, v2, p1, v1) {
//         Ok(_) => {}
//         _ => panic!("lines didn't intersect even thought they should"),
//     }
//     let line1 = Hitbox::line(p1.coords, v1);
//     let line2 = Hitbox::line(p2.coords, v2);
//     assert_collides(&line1, &line2);
// }

// #[test]
// fn lines_not_intersecting() {
//     let p1 = Point::new(-1., -1.);
//     let v1 = Vector::new(1., 1.);
//     let p2 = Point::new(-1., 1.);
//     assert_eq!(
//         line_line_intersection_point(p1, v1, p2, v1),
//         Err(LineIntersectError::NoCollision)
//     );
//     assert_eq!(
//         line_line_intersection_point(p2, v1, p1, v1),
//         Err(LineIntersectError::NoCollision)
//     );
//     let line1 = Hitbox::line(p1.coords, v1.clone());
//     let line2 = Hitbox::line(p2.coords, v1);
//     assert_not_collides(&line1, &line2);
// }

// #[test]
// fn lines_equal() {
//     let p1 = Point::new(0., 0.);
//     let v1 = Vector::new(1., 1.);
//     let p2 = Point::new(2., 2.);
//     let v2 = Vector::new(3., 3.);
//     assert_eq!(
//         line_line_intersection_point(p1, v1, p2, v2),
//         Err(LineIntersectError::Infinite)
//     );
//     assert_eq!(
//         line_line_intersection_point(p2, v2, p1, v1),
//         Err(LineIntersectError::Infinite)
//     );
//     let line1 = Hitbox::line(p1.coords, v1);
//     let line2 = Hitbox::line(p2.coords, v2);
//     assert_collides(&line1, &line2);
// }
// #[test]
// fn line_segment_intersection() {
//     let ls1 = Hitbox::line_segment(Vector::new(-1., 0.), Vector::new(1., 0.));
//     let ls2 = Hitbox::line_segment(Vector::new(0., -1.), Vector::new(0., 1.));
//     let ls1_2 = Hitbox::line_segment(Vector::new(1., 0.), Vector::new(-1., 0.));
//     let ls2_2 = Hitbox::line_segment(Vector::new(0., 1.), Vector::new(0., -1.));

//     assert_collides(&ls1, &ls2);
//     assert_collides(&ls1, &ls1_2);
//     assert_collides(&ls1, &ls2_2);
//     assert_collides(&ls2, &ls1_2);
//     assert_collides(&ls2, &ls2_2);
//     assert_collides(&ls1_2, &ls2_2);
// }

// #[test]
// fn line_segment_no_intersections_above_and_below() {
//     let ls1 = Hitbox::line_segment(Vector::new(-1., 0.25), Vector::new(1., 0.));
//     let ls2 = Hitbox::line_segment(Vector::new(1., 0.), Vector::new(-1., 0.25));
//     let lsabove = Hitbox::line_segment(Vector::new(0., 2.), Vector::new(0., 1.));
//     let lsbelow = Hitbox::line_segment(Vector::new(0.5, -2.), Vector::new(0., -1.));
//     let lsabove_2 = Hitbox::line_segment(Vector::new(0., 1.), Vector::new(0., 2.));
//     let lsbelow_2 = Hitbox::line_segment(Vector::new(0., -1.), Vector::new(0.5, -2.));

//     assert_not_collides(&ls1, &lsabove);
//     assert_not_collides(&ls1, &lsbelow);
//     assert_not_collides(&ls1, &lsabove_2);
//     assert_not_collides(&ls1, &lsbelow_2);

//     assert_not_collides(&ls2, &lsabove);
//     assert_not_collides(&ls2, &lsbelow);
//     assert_not_collides(&ls2, &lsabove_2);
//     assert_not_collides(&ls2, &lsbelow_2);
// }

// #[test]
// fn line_segment_no_intersections_left_and_right() {
//     let ls1 = Hitbox::line_segment(Vector::new(-1., 0.25), Vector::new(1., 0.));
//     let ls2 = Hitbox::line_segment(Vector::new(1., 0.), Vector::new(-1., 0.25));
//     let lsright = Hitbox::line_segment(Vector::new(2., 0.), Vector::new(1.5, 0.25));
//     let lsleft = Hitbox::line_segment(Vector::new(-2., 0.), Vector::new(-1.5, 0.25));
//     let lsright_2 = Hitbox::line_segment(Vector::new(1.5, 0.25), Vector::new(2., 0.));
//     let lsleft_2 = Hitbox::line_segment(Vector::new(-1.5, 0.25), Vector::new(-2., 0.));

//     assert_not_collides(&ls1, &lsright);
//     assert_not_collides(&ls1, &lsleft);
//     assert_not_collides(&ls1, &lsright_2);
//     assert_not_collides(&ls1, &lsleft_2);

//     assert_not_collides(&ls2, &lsright);
//     assert_not_collides(&ls2, &lsleft);
//     assert_not_collides(&ls2, &lsright_2);
//     assert_not_collides(&ls2, &lsleft_2);
// }

// #[test]
// fn rectangle_point_collisions() {
//     let point1 = Hitbox::dot(Vector::new(0., 0.));
//     let point2 = Hitbox::dot(Vector::new(1., 1.));
//     let point3 = Hitbox::dot(Vector::new(-1., -1.));
//     let point4 = Hitbox::dot(Vector::new(1., -1.));
//     let point5 = Hitbox::dot(Vector::new(-1., 1.));

//     let rectangle = Hitbox::rectangle(Vector::new(-1., -1.), Vector::new(1., -1.), 2.);

//     assert_collides(&rectangle, &point1);
//     assert_collides(&rectangle, &point2);
//     assert_collides(&rectangle, &point3);
//     assert_collides(&rectangle, &point4);
//     assert_collides(&rectangle, &point5);
// }

// #[test]
// fn rectangle_point_no_collisions() {
//     let point1 = Hitbox::dot(Vector::new(1., 0.));
//     let point2 = Hitbox::dot(Vector::new(-1., 0.));
//     let point3 = Hitbox::dot(Vector::new(0., 1.));
//     let point4 = Hitbox::dot(Vector::new(0., -1.));

//     let rectangle = Hitbox::rectangle(Vector::new(-0.5, -0.5), Vector::new(0.5, -0.5), 1.);

//     assert_not_collides(&rectangle, &point1);
//     assert_not_collides(&rectangle, &point2);
//     assert_not_collides(&rectangle, &point3);
//     assert_not_collides(&rectangle, &point4);
// }
// #[test]
// fn rectangle_collides_with_its_points() {
//     let spos = Vector::new(-1., -1.);
//     let epos = Vector::new(-0.5, 0.);
//     let thickness = 1.;

//     let rectangle = Hitbox::rectangle(spos, epos, thickness);

//     let perp = (epos - spos).perpendicular().normalize() * thickness;
//     let a1 = spos;
//     let b1 = a1 + perp;
//     let c1 = epos;
//     let d1 = c1 + perp;

//     assert_collides(&rectangle, &Hitbox::dot(a1));
//     assert_collides(&rectangle, &Hitbox::dot(b1));
//     assert_collides(&rectangle, &Hitbox::dot(c1));
//     assert_collides(&rectangle, &Hitbox::dot(d1));
// }

// #[test]
// fn rectangle_rectangle_cross_collides() {
//     let rectangle1 = Hitbox::rectangle(Vector::new(2., -2.), Vector::new(-2., 2.), 0.2);
//     let rectangle2 = Hitbox::rectangle(Vector::new(2., 2.), Vector::new(-2., -2.), 0.2);

//     assert_collides(&rectangle1, &rectangle2);
// }

// #[test]
// fn rectangle_rectangle_corner_cross_collides() {
//     let rectangle1 = Hitbox::rectangle(Vector::new(-1., 0.), Vector::new(1., 0.), 2.);

//     let rectangle2 = Hitbox::rectangle(Vector::new(-1.3, 1.), Vector::new(0., 2.3), 0.2);
//     let rectangle3 = Hitbox::rectangle(Vector::new(0., 2.3), Vector::new(1.3, 1.), 0.2);
//     let rectangle4 = Hitbox::rectangle(Vector::new(0., -0.3), Vector::new(1.3, 1.), 0.2);
//     let rectangle5 = Hitbox::rectangle(Vector::new(-1.3, 1.), Vector::new(0., -0.3), 0.2);

//     assert_collides(&rectangle1, &rectangle2);
//     assert_collides(&rectangle1, &rectangle3);
//     assert_collides(&rectangle1, &rectangle4);
//     assert_collides(&rectangle1, &rectangle5);
// }
