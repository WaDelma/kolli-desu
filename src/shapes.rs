use {Point, Vector};

pub trait Shape {
    fn start(&self) -> Vector<f32>;
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32>;
}

impl Shape for Point<f32> {
    fn start(&self) -> Vector<f32> {
        self.coords
    }
    fn farthest_in_dir(&self, _dir: Vector<f32>) -> Vector<f32> {
        self.coords
    }
}

#[derive(Debug)]
pub struct Circle {
    pub center: Point<f32>,
    pub radius: f32,
}

impl Circle {
    pub fn new(center: Point<f32>, radius: f32) -> Self {
        Circle {
            center,
            radius,
        }
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

#[derive(Debug)]
pub struct Aabb {
    pub from: Point<f32>,
    pub to: Point<f32>,
}

impl Aabb {
    pub fn new(from: Point<f32>, to: Point<f32>) -> Self {
        Aabb {
            from,
            to,
        }
    }
}

impl Shape for Aabb {
    fn start(&self) -> Vector<f32> {
        self.from.coords + (self.to.coords - self.from.coords) / 2.
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


#[derive(Debug)]
pub struct ConvexPolygon {
    points: Vec<Point<f32>>,
}

impl Shape for ConvexPolygon {
    fn start(&self) -> Vector<f32> {
        unimplemented!()
    }
    fn farthest_in_dir(&self, dir: Vector<f32>) -> Vector<f32> {
        let mut max_point = self.points.first().unwrap().coords;
        let mut max_dot = max_point.dot(&dir);
        let right = self.points[1].coords;
        let left = self.points[self.points.len()-1].coords;
        let left_dot = left.dot(&dir);
        let right_dot = right.dot(&dir);
        if max_dot >= left_dot && max_dot >= right_dot {
            return max_point;
        }
        let mut find_largest_dot = |points: &mut dyn Iterator<Item = &Point<f32>>| {
            for p in points.into_iter() {
                let dot = p.coords.dot(&dir);
                if dot >= max_dot {
                    max_dot = dot;
                    max_point = p.coords;
                } else {
                    return max_point;
                }
            }
            max_point
        };
        if left_dot >= right_dot {
            find_largest_dot(&mut {self.points[2..].iter()})
        } else {
            find_largest_dot(&mut {self.points.iter().rev().skip(1)})
        }
    }
}