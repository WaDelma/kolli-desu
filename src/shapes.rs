use crate::{Perp, Point, Vector};

use mopa::{mopafy, Any};

pub trait Shape: Any {
    fn start(&self) -> Vector<f32>;
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32>;
}

pub fn support<S1, S2>(
    (a, a_pos): (&S1, Point<f32>),
    (b, b_pos): (&S2, Point<f32>),
    dir: Vector<f32>,
) -> Vector<f32>
where
    S1: Shape + ?Sized,
    S2: Shape + ?Sized,
{
    let p1 = a_pos + a.farthest_in_dir(dir);
    let p2 = b_pos + b.farthest_in_dir(-dir);
    p1 - p2
}

mopafy!(Shape);

impl Shape for Point<f32> {
    fn start(&self) -> Vector<f32> {
        self.coords
    }
    fn farthest_in_dir(&self, _dir: Vector<f32>) -> Vector<f32> {
        self.coords
    }
}

impl<T> Shape for Box<T>
where
    T: Shape + ?Sized,
{
    fn start(&self) -> Vector<f32> {
        T::start(self)
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        T::farthest_in_dir(self, dir)
    }
}

impl<T> Shape for &'static T
where
    T: Shape + ?Sized,
{
    fn start(&self) -> Vector<f32> {
        T::start(self)
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        T::farthest_in_dir(self, dir)
    }
}

#[derive(Clone, Debug)]
pub struct Circle {
    pub center: Point<f32>,
    pub radius: f32,
}

impl Circle {
    pub fn new(center: Point<f32>, radius: f32) -> Self {
        Circle { center, radius }
    }
}

impl Shape for Circle {
    fn start(&self) -> Vector<f32> {
        self.center.coords
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        self.center.coords + dir.normalize() * self.radius
    }
}

#[derive(Clone, Debug)]
pub struct Aabb {
    pub from: Point<f32>,
    pub to: Point<f32>,
}

//TODO: Figure out if it makes any sense to have a separate AABB struct since we have a ConvexPolygon struct
//      and the GJK implementation doesn't seem to benefit from axis alignment.
impl Aabb {
    pub fn new(from: Point<f32>, to: Point<f32>) -> Self {
        Aabb { from, to }
    }
}

impl Shape for Aabb {
    fn start(&self) -> Vector<f32> {
        self.from.coords + (self.to.coords - self.from.coords) / 2.
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        Vector::new(
            if dir.x > 0. { self.to.x } else { self.from.x },
            if dir.y > 0. { self.to.y } else { self.from.y },
        )
    }
}

#[derive(Clone, Debug)]
pub struct ConvexPolygon {
    pub points: Vec<Point<f32>>,
}

impl ConvexPolygon {
    pub fn new(points: Vec<Point<f32>>) -> Self {
        ConvexPolygon { points }
    }

    pub fn new_rectangle(from: Point<f32>, to: Point<f32>, thickness: f32) -> Self {
        let perp = (to - from).perpendicular().normalize() * thickness;
        let fp = from + perp;
        let tp = to + perp;
        ConvexPolygon::new(vec![from, to, tp, fp])
    }

    pub fn new_line_segment(from: Point<f32>, to: Point<f32>) -> Self {
        ConvexPolygon::new(vec![from, to])
    }

    fn dot(&self, index: isize, dir: Vector<f32>) -> f32 {
        if index == -1 {
            self.points[self.points.len() - 1]
        } else if index as usize == self.points.len() {
            self.points[0]
        } else {
            self.points[index as usize]
        }
        .coords
        .dot(&dir)
    }
}

impl Shape for ConvexPolygon {
    fn start(&self) -> Vector<f32> {
        self.points[0].coords
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        let size = self.points.len() as isize;

        let mut index = 0;
        let mut cur = self.dot(index, dir);
        // Negative cur means that were looking from opposite direction
        if cur < 0. {
            index = size / 2;
            cur = self.dot(index, dir);
        }

        let left = self.dot(index - 1, dir);
        let right = self.dot(index + 1, dir);
        if left <= cur && cur >= right {
            return self.points[index as usize].coords;
        }

        let step = if left > right {
            cur = left;
            -1
        } else {
            cur = right;
            1
        };
        index += step;
        loop {
            if index == -1 {
                index = size - 1;
            }
            if index == size {
                index = 0;
            }
            let next = self.dot(index + step, dir);
            if cur >= next {
                return self.points[index as usize].coords;
            }
            cur = next;
            index += step;
        }
    }
}
