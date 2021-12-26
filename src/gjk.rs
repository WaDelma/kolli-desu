use crate::shapes::support;

use crate::shapes::Shape;
use crate::simplex::Simplex;
use crate::{Point, Vector};

/// a x (b x c)
pub fn triple_product(a: Vector<f32>, b: Vector<f32>, c: Vector<f32>) -> Vector<f32> {
    b * c.dot(&a) - a * c.dot(&b)
}

pub fn collides<S1, S2>(a: (&S1, Point<f32>), b: (&S2, Point<f32>)) -> bool
where
    S1: Shape + ?Sized,
    S2: Shape + ?Sized,
{
    collides_internal(a, b).0
}

pub fn collides_internal<S1, S2>(a: (&S1, Point<f32>), b: (&S2, Point<f32>)) -> (bool, Simplex)
where
    S1: Shape + ?Sized,
    S2: Shape + ?Sized,
{
    let mut cur = (a.1 + a.0.start()) - (b.1 + b.0.start());
    if cur == Vector::new(0., 0.) {
        cur = Vector::new(1., 0.);
    }
    let mut simplex = Simplex::Point(support(a, b, cur));
    cur = -cur;
    while cur != Vector::new(0., 0.) {
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
        }
        Simplex::Line(b, a) => {
            let ao = -a;
            let ab = b - a;
            *cur = triple_product(ab, ao, ab);
        }
        _ => unreachable!(),
    }
    false
}
