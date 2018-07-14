extern crate nalgebra as na;
extern crate alga;

use std::fmt::Debug;
use std::ops::Neg;

use alga::linear::Transformation;

use na::{Rotation2, zero, dot};

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

#[derive(Debug)]
pub enum Hitbox {
    Circle {
        displacement: Vector<f32>,
        radius: f32,
    },
    ///First vector denotes the center of the AABB and the second vector denotes the dimensions(width, height) of the AABB
    Aabb {
        displacement: Vector<f32>,
        width: f32,
        height: f32,
    },
    Rectangle(Point<f32>, Point<f32>, f32),
    Line(Point<f32>, Vector<f32>),
    LineSegment(Point<f32>, Point<f32>),
    Dot(Point<f32>),
}
impl Hitbox {
    pub fn enclosing_aabb(&self, pos: &Point<f32>) -> (Point<f32>, Point<f32>) {
        use self::Hitbox::*;
        // let pos = pos + self.center();
        // match *self {
        //     Circle(_, radius) => (
        //         pos - Vector::new(radius, radius),
        //         pos + Vector::new(radius, radius),
        //     ),
        //     Aabb(_, side) => (pos - side.abs() / 2., pos + side.abs() / 2.),
        // }
        unimplemented!()
    }

    pub fn center(&self) -> Point<f32> {
        use self::Hitbox::*;
        // match *self {
        //     Circle(ref center, _) => Point::from_coordinates(*center),
        //     _ => unimplemented!(),
        // }
        unimplemented!()
    }

    pub fn collides<'a>((hitbox1, pos1): (&'a Hitbox, Point<f32>), (hitbox2, pos2): (&'a Hitbox, Point<f32>)) -> bool {
        use self::Hitbox::*;
        match (hitbox1, hitbox2) {
            (Circle { displacement: a_lpos, radius: a_radius }, Circle { displacement: b_lpos, radius: b_radius }) => {
                ((*a_lpos) - (*b_lpos)).norm_squared() <= (*a_radius + *b_radius).powi(2)
            }
            (Circle { displacement: c_lpos, radius: c_radius }, Aabb { displacement: a_lpos, width, height })
            | (Aabb { displacement: a_lpos, width, height }, Circle { displacement: c_lpos, radius: c_radius }) => {
                let width = width.abs() / 2.;
                let height = height.abs() / 2.;
                let aabb_center = *a_lpos;
                let circle_center = *c_lpos;
                let mut ca = aabb_center - circle_center;
                if ca != zero() {
                    ca = ca.normalize();
                }
                let outer = circle_center + *c_radius * ca;
                point_in_aabb(
                    Point::from_coordinates(outer),
                    (Point::from_coordinates(aabb_center), width, height),
                )
            }
            (
                Circle { displacement: c_lpos, radius: c_radius },
                &Rectangle(ref r_spos, ref r_epos, ref r_height),
            )
            | (
                &Rectangle(ref r_spos, ref r_epos, ref r_height),
                Circle { displacement: c_lpos, radius: c_radius },
            ) => {
                let rotation =
                    Rotation2::rotation_between(&(r_epos - r_spos), &Vector::new(1., 0.));
                let rot_circle = rotation.transform_vector(&(c_lpos - r_spos.coords));
                let aabb_width = (r_epos - r_spos).norm();
                let aabb_center = Vector::new(aabb_width / 2., r_height / 2.);
                let aabb = Aabb {
                    displacement: aabb_center,
                    width: aabb_width, 
                    height: *r_height
                };
                Hitbox::collides((&aabb, pos1), (&Circle { displacement: rot_circle, radius: *c_radius }, pos2))
            }
            (Aabb { displacement: aabb_pos, width, height }, r @ &Rectangle(..))
            | (r @ &Rectangle(..), Aabb { displacement: aabb_pos, width, height }) => {
                let s_pos = Point::from_coordinates(*aabb_pos);
                let e_pos = Point::from_coordinates(aabb_pos + Vector::new(*width, 0.));
                Hitbox::collides((r, pos1), (&Rectangle(s_pos, e_pos, *height), pos2))
            }
            (
                &Rectangle(ref r1_spos, ref r1_epos, ref r1_height),
                &Rectangle(ref r2_spos, ref r2_epos, ref r2_height),
            ) => {
                let perp = (r1_epos - r1_spos).perpendicular().normalize() * *r1_height;
                let a1 = r1_spos;
                let b1 = a1 + perp;
                let c1 = r1_epos;
                let d1 = c1 + perp;

                let perp = (r2_epos - r2_spos).perpendicular().normalize() * *r2_height;
                let a2 = r2_spos;
                let b2 = a2 + perp;
                let c2 = r2_epos;
                let d2 = c2 + perp;

                let r1 = &Rectangle(*r1_spos, *r1_epos, *r1_height);
                let r2 = &Rectangle(*r2_spos, *r2_epos, *r2_height);

                let r1_line_ad = &Hitbox::LineSegment(*a1, d1);
                let r1_line_bc = &Hitbox::LineSegment(b1, *c1);
                let r2_line_ad = &Hitbox::LineSegment(*a2, d2);
                let r2_line_bc = &Hitbox::LineSegment(b2, *c2);

                Hitbox::collides((r1, pos1), (&Dot(*a2), pos2))
                    || Hitbox::collides((r1, pos1), (&Dot(b2), pos2))
                    || Hitbox::collides((r1, pos1), (&Dot(*c2), pos2))
                    || Hitbox::collides((r1, pos1), (&Dot(d2), pos2))
                    || Hitbox::collides((r2, pos2), (&Dot(*a1), pos1))
                    || Hitbox::collides((r2, pos2), (&Dot(b1), pos1))
                    || Hitbox::collides((r2, pos2), (&Dot(*c1), pos1))
                    || Hitbox::collides((r2, pos2), (&Dot(d1), pos1))
                    || Hitbox::collides((r1_line_ad, pos1), (r2_line_ad, pos2))
                    || Hitbox::collides((r1_line_ad, pos1), (r2_line_bc, pos2))
                    || Hitbox::collides((r1_line_bc, pos1), (r2_line_ad, pos2))
                    || Hitbox::collides((r1_line_bc, pos1), (r2_line_bc, pos2))
            }
            (&Rectangle(ref r_spos, ref r_epos, ref r_height), &Dot(ref p))
            | (&Dot(ref p), &Rectangle(ref r_spos, ref r_epos, ref r_height)) => {
                let perp = (r_epos - r_spos).perpendicular().normalize() * *r_height;
                let a = &r_spos.coords;
                let b = &r_epos.coords;
                let c = &(b + perp);
                let d = &(a + perp);

                let which_side = |(a, b): (&Vector<f32>, &Vector<f32>), c| {
                    let diff = b - a;
                    dot(&(c - a), &diff.perpendicular())
                };

                let p = &p.coords;
                which_side((a, b), p) >= 0. && which_side((b, c), p) >= 0.
                    && which_side((c, d), p) >= 0. && which_side((d, a), p) >= 0.
            }
            (&Dot(ref p1), &Dot(ref p2)) => p1 == p2,
            (Aabb { displacement: a1_lpos, width: a1_width, height: a1_height }, Aabb { displacement: a2_lpos, width: a2_width, height: a2_height }) => {
                let a1_center = *a1_lpos;
                let a2_center = *a2_lpos;
                let a1_width = a1_width.abs() / 2.;
                let a1_height = a1_height.abs() / 2.;
                let a2_width = a2_width.abs() / 2.;
                let a2_height = a2_height.abs() / 2.;
                (a1_center.x - a1_width) <= (a2_center.x + a2_width)
                    && (a1_center.x + a1_width) >= (a2_center.x - a2_width)
                    && (a1_center.y - a1_height) <= (a2_center.y + a2_height)
                    && (a1_center.y + a1_height) >= (a2_center.y - a2_height)
            }

            (&Line(ref p1, ref v1), &Line(ref p2, ref v2)) => {
                match line_line_intersection_point(p1, v1, p2, v2) {
                    Ok(_) => true,
                    Err(LineIntersectError::Infinite) => true,
                    Err(LineIntersectError::NoCollision) => false,
                }
            }
            (&LineSegment(ref a1, ref a2), &LineSegment(ref b1, ref b2)) => {
                match line_line_intersection_point(a1, &(a2 - a1), b1, &(b2 - b1)) {
                    Ok(p) => {
                        p.x >= a1.x.min(a2.x) && p.x <= a1.x.max(a2.x) && p.y >= a1.y.min(a2.y)
                            && p.y <= a1.y.max(a2.y)
                            && p.x >= b1.x.min(b2.x)
                            && p.x <= b1.x.max(b2.x)
                            && p.y >= b1.y.min(b2.y)
                            && p.y <= b1.y.max(b2.y)
                    }
                    Err(LineIntersectError::Infinite) => {
                        b1.x >= a1.x.min(a2.x) && b1.x <= a1.x.max(a2.x) && b1.y >= a1.y.min(a2.y)
                            && b1.y <= a1.y.max(a2.y)
                            || b2.x >= a1.x.min(a2.x) && b2.x <= a1.x.max(a2.x)
                                && b2.y >= a1.y.min(a2.y)
                                && b2.y <= a1.y.max(a2.y)
                            || a1.x >= b1.x.min(b2.x) && a1.x <= b1.x.max(b2.x)
                                && a1.y >= b1.y.min(b2.y)
                                && a1.y <= b1.y.max(b2.y)
                            || a1.x >= b1.x.min(b2.x) && a1.x <= b1.x.max(b2.x)
                                && a1.y >= b1.y.min(b2.y)
                                && a1.y <= b1.y.max(b2.y)
                    }
                    Err(LineIntersectError::NoCollision) => false,
                }
            }
            ref u => unimplemented!("{:?}", u),
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum LineIntersectError {
    Infinite,
    NoCollision,
}

pub fn line_line_intersection_point(
    p1: &Point<f32>,
    v1: &Vector<f32>,
    p2: &Point<f32>,
    v2: &Vector<f32>,
) -> Result<Point<f32>, LineIntersectError> {
    let denominator_det = (v1.x * (-v2.y)) - ((-v2.x) * v1.y);
    let numerator_det = ((p2.x - p1.x) * (-v2.y)) - ((-v2.x) * (p2.y - p1.y));
    if denominator_det == 0. {
        if numerator_det == 0. {
            Err(LineIntersectError::Infinite)
        } else {
            Err(LineIntersectError::NoCollision)
        }
    } else {
        let x = numerator_det / denominator_det;
        Ok(p1 + (v1 * x))
    }
}

fn point_in_aabb(point: Point<f32>, (center, width, height): (Point<f32>, f32, f32)) -> bool {
    point.x >= (center.x - width) && point.x <= (center.x + width) && point.y >= (center.y - height)
        && point.y <= (center.y + height)
}