use crate::na::zero;

use crate::{Point, Vector};
use crate::shapes::Shape;
use crate::simplex::Simplex;

pub fn support((a, a_pos): (&impl Shape, Point<f32>), (b, b_pos): (&impl Shape, Point<f32>), dir: Vector<f32>) -> Vector<f32> {
  let p1 = a_pos + a.farthest_in_dir(dir);
  let p2 = b_pos + b.farthest_in_dir(-dir);
  p1 - p2
}

/// a x (b x c)
pub fn triple_product(a: Vector<f32>, b: Vector<f32>, c: Vector<f32>) -> Vector<f32> {
    b * c.dot(&a) - a * c.dot(&b)
}

pub fn collides(a: (&impl Shape, Point<f32>), b: (&impl Shape, Point<f32>)) -> bool {
    collides_internal(a, b).0
}

pub fn collides_internal(a: (&impl Shape, Point<f32>), b: (&impl Shape, Point<f32>)) -> (bool, Simplex) {
    let mut cur = (a.1 + a.0.start()) - (b.1 + b.0.start());
    if cur == zero() {
        cur = Vector::new(1., 0.);
    }
    let mut simplex = Simplex::Point(support(a, b, cur));
    cur = -cur;
    while cur != zero() {
        let support = support(a, b, cur);
        simplex.add(support);
        if support.dot(&cur) < 0. {
            return (false, simplex);
        } else if expand(&mut simplex, &mut cur) {
            return (true, simplex);
        }
    }
    (true, simplex)
}

fn expand(simplex: &mut Simplex, cur: &mut Vector<f32>) -> bool {
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