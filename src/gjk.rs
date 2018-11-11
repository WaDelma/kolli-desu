use {Point, Vector};

#[test]
fn test() {
    assert!(collides(
        Circle { center: Vector::new(1., 0.), radius: 1.1},
        Circle { center: Vector::new(-1., 0.), radius: 1.1}
    ));
    assert!(!collides(
        Circle { center: Vector::new(-1., 0.), radius: 0.9},
        Circle { center: Vector::new(1., 0.), radius: 0.9}
    ));
}

pub struct Circle {
    center: Vector<f32>,
    radius: f32,
}

impl Shape for Circle {
    fn start(&self) -> Vector<f32> {
        self.center
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        self.center + dir.normalize() * self.radius
    }
}

pub trait Shape {
    fn start(&self) -> Vector<f32>;
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32>;
}

fn support(shape1: &impl Shape, shape2: &impl Shape, dir: Vector<f32>) -> Vector<f32> {
  let p1 = shape1.farthest_in_dir(dir);
  let p2 = shape2.farthest_in_dir(-dir);
  p1 - p2
}

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

pub fn collides(a: impl Shape, b: impl Shape) -> bool {
    let mut cur = a.start();
    let mut simplex = Simplex::Point(support(&a, &b, cur));
    cur = -cur;
    loop {
        simplex.add(support(&a, &b, cur));
        if simplex.last().dot(&cur) <= 0. {
            return false;
        } else if contains(&mut simplex, &mut cur) {
            return true;
        }
    }
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
    b * a.dot(&c) - c * a.dot(&b)
}