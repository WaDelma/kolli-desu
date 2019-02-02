use crate::Vector;

#[derive(Copy, Clone)]
pub enum Winding {
    Left, Right,
}

#[derive(Debug)]
pub enum Simplex {
    Point(Vector<f32>),
    Line(Vector<f32>, Vector<f32>),
    Triangle(Vector<f32>, Vector<f32>, Vector<f32>),
}

impl Simplex {
    pub fn winding(&self) -> Winding {
        use self::{Simplex::*, Winding::*};
        if let Triangle(v1, v2, _) = self {
            let dot = v1.dot(&v2);
            if dot < 0. {
                Right
            } else {
                Left
            }
        } else {
            panic!("Winding is only defined for the triangle.");
        }
    }

    pub fn add(&mut self, p: Vector<f32>) {
        use self::Simplex::*;
        *self = match self {
            Point(p2) => Line(*p2, p),
            Line(p2, p3) => Triangle(*p2, *p3, p),
            _ => panic!(),
        }
    }

    pub fn last(&self) -> &Vector<f32> {
        use self::Simplex::*;
        match self {
            Point(p) | Line(_, p) | Triangle(_, _, p) => p,
        }
    }
}

impl<'a> IntoIterator for &'a Simplex {
    type IntoIter = SimplexIter<'a>;
    type Item = Vector<f32>;
    fn into_iter(self) -> Self::IntoIter {
        SimplexIter {
            simplex: self,
            cur: 0,
        }
    }
}

pub struct SimplexIter<'a> {
    simplex: &'a Simplex,
    cur: u8,
}

impl Iterator for SimplexIter<'_> {
    type Item = Vector<f32>;
    fn next(&mut self) -> Option<Self::Item> {
        use self::Simplex::*;
        match self.simplex {
            Triangle(v1, v2, v3) => {
                let v = match self.cur {
                    0 => v1,
                    1 => v2,
                    2 => v3,
                    _ => return None,
                };
                self.cur += 1;
                Some(*v)
            }
            Line(v1, v2) => {
                let v = match self.cur {
                    0 => v1,
                    1 => v2,
                    _ => return None,
                };
                self.cur += 1;
                Some(*v)
            }
            Point(v1) => {
                let v = match self.cur {
                    0 => v1,
                    _ => return None,
                };
                self.cur += 1;
                Some(*v)
            }
        }
    }
}
