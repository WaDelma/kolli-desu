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
        center: Vector<f32>,
        radius: f32,
    },
    ///First vector denotes the center of the AABB and the second vector denotes the dimensions(width, height) of the AABB
    Aabb {
        center: Vector<f32>,
        width: f32,
        height: f32,
    },
    Rectangle{
        from: Vector<f32>, 
        to: Vector<f32>, 
        thickness: f32
    },
    Line{
        displacement: Vector<f32>, 
        direction: Vector<f32>
    },
    LineSegment{
        from: Vector<f32>, 
        to: Vector<f32>,
    },
    Dot{
        point: Vector<f32>,
    },
}
impl Hitbox {

    pub fn circle(center: Vector<f32>, radius: f32) -> Hitbox {
        Hitbox::Circle {
            center,
            radius,
        }
    }

    pub fn aabb(center: Vector<f32>, dims: Vector<f32>) -> Hitbox {
        Hitbox::Aabb {
            center,
            width: dims.x,
            height: dims.y,
        }
    }

    pub fn rectangle(from: Vector<f32>, to: Vector<f32>, thickness: f32) -> Hitbox {
        Hitbox::Rectangle {
            from,
            to,
            thickness,
        }
    }

    pub fn line_segment(from: Vector<f32>, to: Vector<f32>) -> Hitbox {
        Hitbox::LineSegment {
            from,
            to,
        }
    }

    pub fn line(displacement: Vector<f32>, direction: Vector<f32>) -> Hitbox {
        Hitbox::Line {
            displacement,
            direction,
        }
    }

    pub fn dot(point: Vector<f32>) -> Hitbox {
        Hitbox::Dot {
            point,
        }
    }

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
            (Circle { center: a_lpos, radius: a_radius }, Circle { center: b_lpos, radius: b_radius }) => {
                ((pos1 + *a_lpos) - (pos2 + *b_lpos)).norm_squared() <= (*a_radius + *b_radius).powi(2)
            }
            (Circle { center: c_lpos, radius: c_radius }, Aabb { center: a_lpos, width, height })
            | (Aabb { center: a_lpos, width, height }, Circle { center: c_lpos, radius: c_radius }) => {
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
                Circle { center: c_lpos, radius: c_radius },
                &Rectangle{
                   from: r_spos,
                   to: r_epos,
                   thickness: r_height,
                },
            )
            | (
                &Rectangle{
                   from: r_spos,
                   to: r_epos,
                   thickness: r_height,
                },
                Circle { center: c_lpos, radius: c_radius },
            ) => {
                let rotation =
                    Rotation2::rotation_between(&(r_epos - r_spos), &Vector::new(1., 0.));
                let rot_circle = rotation.transform_vector(&(c_lpos - r_spos));
                let aabb_width = (r_epos - r_spos).norm();
                let aabb_center = Vector::new(aabb_width / 2., r_height / 2.);
                let aabb = Aabb {
                    center: aabb_center,
                    width: aabb_width, 
                    height: r_height
                };
                Hitbox::collides((&aabb, pos1), (&Circle { center: rot_circle, radius: *c_radius }, pos2))
            }
            //TODO: pos1 and pos2 used wrongly here.
            (Aabb { center: aabb_pos, width, height }, r @ &Rectangle{..})
            | (r @ &Rectangle{..}, Aabb { center: aabb_pos, width, height }) => {
                let s_pos = aabb_pos;
                let e_pos = aabb_pos + Vector::new(*width, 0.);
                Hitbox::collides((r, pos1), (&Hitbox::rectangle(*s_pos, e_pos, *height), pos2))
            }
            (
                &Rectangle{
                   from: r1_spos,
                   to: r1_epos,
                   thickness: r1_height,
                },
                &Rectangle{
                   from: r2_spos,
                   to: r2_epos,
                   thickness: r2_height,
                },
            ) => {
                let perp = (r1_epos - r1_spos).perpendicular().normalize() * r1_height;
                let a1 = r1_spos;
                let b1 = a1 + perp;
                let c1 = r1_epos;
                let d1 = c1 + perp;

                let perp = (r2_epos - r2_spos).perpendicular().normalize() * r2_height;
                let a2 = r2_spos;
                let b2 = a2 + perp;
                let c2 = r2_epos;
                let d2 = c2 + perp;

                let r1 = &Hitbox::rectangle(r1_spos, r1_epos, r1_height);
                let r2 = &Hitbox::rectangle(r2_spos, r2_epos, r2_height);

                let r1_line_ad = &Hitbox::line_segment(a1, d1);
                let r1_line_bc = &Hitbox::line_segment(b1, c1);
                let r2_line_ad = &Hitbox::line_segment(a2, d2);
                let r2_line_bc = &Hitbox::line_segment(b2, c2);

                Hitbox::collides((r1, pos1), (&Self::dot(a2), pos2))
                    || Hitbox::collides((r1, pos1), (&Self::dot(b2), pos2))
                    || Hitbox::collides((r1, pos1), (&Self::dot(c2), pos2))
                    || Hitbox::collides((r1, pos1), (&Self::dot(d2), pos2))
                    || Hitbox::collides((r2, pos2), (&Self::dot(a1), pos1))
                    || Hitbox::collides((r2, pos2), (&Self::dot(b1), pos1))
                    || Hitbox::collides((r2, pos2), (&Self::dot(c1), pos1))
                    || Hitbox::collides((r2, pos2), (&Self::dot(d1), pos1))
                    || Hitbox::collides((r1_line_ad, pos1), (r2_line_ad, pos2))
                    || Hitbox::collides((r1_line_ad, pos1), (r2_line_bc, pos2))
                    || Hitbox::collides((r1_line_bc, pos1), (r2_line_ad, pos2))
                    || Hitbox::collides((r1_line_bc, pos1), (r2_line_bc, pos2))
            }
            (&Rectangle{
                from: r_spos, 
                to: r_epos, 
                thickness: r_height,
            }, &Dot{point: p})
            | (&Dot{point: p}, 
            &Rectangle{
                from: r_spos, 
                to: r_epos, 
                thickness: r_height,
            }) => {
                let perp = (r_epos - r_spos).perpendicular().normalize() * r_height;
                let a = &r_spos;
                let b = &r_epos;
                let c = &(b + perp);
                let d = &(a + perp);

                let which_side = |(a, b): (&Vector<f32>, &Vector<f32>), c| {
                    let diff = b - a;
                    dot(&(c - a), &diff.perpendicular())
                };
                which_side((a, b), p) >= 0. && which_side((b, c), p) >= 0.
                    && which_side((c, d), p) >= 0. && which_side((d, a), p) >= 0.
            }
            (&Dot{point: p1}, &Dot{point: p2}) => p1 == p2,

            (Aabb { center: a1_lpos, width: a1_width, height: a1_height }, Aabb { center: a2_lpos, width: a2_width, height: a2_height }) => {
                let a1_center = pos1 + *a1_lpos;
                let a2_center = pos2 + *a2_lpos;
                let a1_width = a1_width.abs() / 2.;
                let a1_height = a1_height.abs() / 2.;
                let a2_width = a2_width.abs() / 2.;
                let a2_height = a2_height.abs() / 2.;
                (a1_center.x - a1_width) <= (a2_center.x + a2_width)
                    && (a1_center.x + a1_width) >= (a2_center.x - a2_width)
                    && (a1_center.y - a1_height) <= (a2_center.y + a2_height)
                    && (a1_center.y + a1_height) >= (a2_center.y - a2_height)
            }

            (&Line{displacement: p1, direction: v1}, &Line{displacement: p2, direction: v2}) =>
                match line_line_intersection_point(&(pos1 + p1), &v1, &(pos2 + p2), &v2) {
                    Ok(_) => true,
                    Err(LineIntersectError::Infinite) => true,
                    Err(LineIntersectError::NoCollision) => false,
                },
            (&LineSegment{from: a1, to: a2}, &LineSegment{from: b1, to: b2}) => {
                match line_line_intersection_point(&(pos1 + a1), &(a2 - a1), &(pos2 + b1), &(b2 - b1)) {
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