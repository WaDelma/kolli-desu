use crate::na::zero;

use crate::{Point, Vector};
use crate::shapes::Shape;

fn support((a, a_pos): (&impl Shape, Point<f32>), (b, b_pos): (&impl Shape, Point<f32>), dir: Vector<f32>) -> Vector<f32> {
  let p1 = a_pos + a.farthest_in_dir(dir);
  let p2 = b_pos + b.farthest_in_dir(-dir);
  p1 - p2
}

#[derive(Debug)]
enum Simplex {
    Point(Vector<f32>),
    Line(Vector<f32>, Vector<f32>),
    Triangle(Vector<f32>, Vector<f32>, Vector<f32>),
}

impl Simplex {
    fn add(&mut self, p: Vector<f32>) {
        use self::Simplex::*;
        *self = match self {
            Point(p2) => Line(*p2, p),
            Line(p2, p3) => Triangle(*p2, *p3, p),
            _ => panic!(),
        }
    }

    fn last(&self) -> &Vector<f32> {
        use self::Simplex::*;
        match self {
            Point(p) | Line(_, p) | Triangle(_, _, p) => p,
        }
    }
}

pub fn collides(a: (&impl Shape, Point<f32>), b: (&impl Shape, Point<f32>)) -> bool {
    let mut cur = (a.1 + a.0.start()) - (b.1 + b.0.start());
    let mut simplex = Simplex::Point(support(a, b, cur));
    cur = -cur;
    while cur != zero() {
        simplex.add(support(a, b, cur));
        println!("{:?}", simplex);
        if simplex.last().dot(&cur) <= 0. {
            return false;
        } else if contains(&mut simplex, &mut cur) {
            return true;
        }
    }
    true
}

fn contains(simplex: &mut Simplex, cur: &mut Vector<f32>) -> bool {
    match *simplex {
        Simplex::Triangle(b, c, a) => {
            let ao = -a;
            let ab = b - a;
            let ac = c - a;
            let ab_perp = triple_product(ac, ab, ab);
            let ac_perp = triple_product(ab, ac, ac);
            if ab_perp.dot(&ao) > 0. {
                *simplex = Simplex::Line(b, a);
                *cur = ab_perp;
            } else if ac_perp.dot(&ao) > 0. {
                *simplex = Simplex::Line(c, a);
                *cur = ac_perp;
            } else {
                return true;
            }
        },
        Simplex::Line(b, a) => {
            let ao = -a;
            let ab = b - a;
            *cur = triple_product(ab, ao, ab);
        }
        _ => unreachable!(),
    }
    return false;
}

/// a x (b x c)
fn triple_product(a: Vector<f32>, b: Vector<f32>, c: Vector<f32>) -> Vector<f32> {
    b * c.dot(&a) - a * c.dot(&b)
}