use {Point, Vector};

pub trait Shape {
    fn start(&self) -> Vector<f32>;
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32>;
}

#[derive(Debug)]
pub struct Circle {
    pub center: Vector<f32>,
    pub radius: f32,
}

impl Circle {
    pub fn new(center: Vector<f32>, radius: f32) -> Self {
        Circle {
            center,
            radius,
        }
    }
}

impl Shape for Circle {
    fn start(&self) -> Vector<f32> {
        self.center
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        self.center + dir.normalize() * self.radius
    }
}

#[derive(Debug)]
pub struct Aabb {
    pub from: Vector<f32>,
    pub to: Vector<f32>,
}

impl Aabb {
    pub fn new(from: Vector<f32>, to: Vector<f32>) -> Self {
        Aabb {
            from,
            to,
        }
    }
}

impl Shape for Aabb {
    fn start(&self) -> Vector<f32> {
        self.from + (self.to - self.from) / 2.
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        Vector::new(
            if dir.x > 0. {
                self.to.x
            } else {
                self.from.x
            },
            if dir.y > 0. {
                self.to.y
            } else {
                self.from.y
            }
        )
    }
}