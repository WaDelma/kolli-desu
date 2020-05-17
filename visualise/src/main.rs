use std::collections::VecDeque;
use std::path::Path;

use kolli_desu::{
    shapes::{Aabb, Circle, ConvexPolygon, Shape},
    Point, Vector,
};
use nalgebra::Isometry2;

const TAU: f32 = 2. * ::std::f32::consts::PI;

fn main() {
    let imgx = 5000; //1920;
    let imgy = 5000; //1080;

    // let shape1 = ConvexPolygon::new_line_segment(Point::new(-1., -1.), Point::new(1., 1.));
    // let shape2 = ConvexPolygon::new_line_segment(Point::new(1., -1.), Point::new(-1., 1.));

    // let shape1 = Aabb::new(Point::new(-0.1, 0.), Point::new(0.5, 0.5));
    // let shape2 = Aabb::new(Point::new(-0.5, 0.), Point::new(0., 0.5));

    // let shape1 = Aabb::new(Point::new(-0.1, 0.1), Point::new(0.5, 0.5));
    // let shape2 = Aabb::new(Point::new(-0.5, 0.), Point::new(0., 0.5));

    // let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0.5, 0.5));
    // let shape2 = Circle::new(Point::new(0.1, 0.1), 1.);

    // let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0., 0.));
    // let shape2 = Circle::new(Point::new(1., 1.), 0.5);

    // let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0.0, 0.5));
    // let shape2 = Circle::new(Point::new(0.5, 0.), 0.4);

    // let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0.5, 0.5));
    // let shape2 = ConvexPolygon::new_rectangle(Point::new(-1., -1.), Point::new(-0.4, 0.), 1.);

    // let shape1 = ConvexPolygon::new_rectangle(Point::new(0., 0.), Point::new(1., 1.), 2f32.sqrt());
    // let shape2 = Circle::new(Point::new(2., 1.), 1.1);

    // let shape1 = Point::new(1., -1.);
    // let shape2 = ConvexPolygon::new_rectangle(Point::new(-1., -1.), Point::new(1., -1.), 2.);

    // let shape1 = Circle::new(Point::new(-0.75, 0.), 1.);
    // let shape2 = Circle::new(Point::new(0.75, 0.), 1.);

    // let shape1 = Circle::new(Point::new(0., 0.), 0.5);
    // let shape2 = Circle::new(Point::new(0., 0.), 0.5);

    // let shape1 = Circle::new(Point::new(0., 1.5), 0.5);
    // let shape2 = Circle::new(Point::new(0., 0.), 0.5);

    // let shape1 = Circle::new(Point::new(0.1, 0.1), 0.5);
    // let shape2 = Circle::new(Point::new(0., 0.), 0.5);

    // let shape1 = Circle::new(Point::new(0., 0.), 0.5);
    // let mut shape2 = Circle::new(Point::new(1., 0.), 0.5);
    // shape2.center = Isometry2::new(Vector::new(0., 0.), 45. * TAU / 360.) * shape2.center;

    let shape1 =
        ConvexPolygon::new_rectangle(Point::new(0., 0.), Point::new(0.5, 0.5), 1f32.sqrt());
    let shape2 = Circle::new(Point::new(1., 0.5), 0.55);

    let mut imgbuf = image::ImageBuffer::from_pixel(imgx, imgy, image::Rgb([25, 25, 25]));

    let scale = 1000.;
    let transform = |v: Vector<_>| {
        Vector::new(
            imgx as f32 / 2. + v.x * scale,
            imgy as f32 / 2. - v.y * scale,
        )
    };

    let (collides, simplex1) = ::kolli_desu::gjk::collides_internal(
        (&shape1, Point::new(0., 0.)),
        (&shape2, Point::new(0., 0.)),
    );
    println!("collides: {}", collides);
    println!(
        "{:?}",
        (&simplex1)
            .into_iter()
            .map(|v| (v.x, v.y))
            .collect::<Vec<_>>()
    );
    let (penetration, depth, simplex) = ::kolli_desu::epa::solve_internal(
        (&shape1, Point::new(0., 0.)),
        (&shape2, Point::new(0., 0.)),
        simplex1.clone(),
    );
    println!(
        "{}: {:?}",
        simplex.len(),
        simplex.iter().map(|v| (v.x, v.y)).collect::<Vec<_>>()
    );
    println!("penetration: {}, depth: {}", penetration, depth);

    let mut colors = VecDeque::new();
    for c in &[[255, 255, 0], [0, 255, 255], [255, 0, 255]][..] {
        colors.push_back(image::Rgb(*c));
    }
    let points1 = simplex1.into_iter().map(transform).collect::<Vec<_>>();
    let steps = 10000;
    for (from, to) in points1
        .iter()
        .zip(points1.iter().skip(1).chain(points1.first()))
    {
        let color = colors.pop_back().unwrap();
        colors.push_front(color);
        for n in 0..steps {
            if n % 400 < 200 {
                continue;
            }
            let f = n as f32 / steps as f32;
            let p = from * f + to * (1. - f);
            imgbuf[(p.x as u32, p.y as u32)] = color;
        }
    }

    let points = simplex.into_iter().map(transform).collect::<Vec<_>>();
    let mut colors = VecDeque::new();
    for c in &[
        [185, 255, 195],
        [228, 52, 233],
        [70, 242, 69],
        [254, 0, 198],
        [57, 203, 0],
        [190, 56, 210],
        [164, 248, 20],
        [92, 111, 255],
        [239, 255, 54],
        [32, 143, 255],
        [162, 209, 0],
        [248, 137, 255],
        [0, 182, 52],
        [242, 0, 135],
        [1, 240, 141],
        [221, 53, 78],
        [0, 246, 197],
        [255, 124, 199],
        [164, 255, 131],
        [115, 110, 188],
        [255, 218, 82],
        [21, 126, 188],
        [255, 156, 54],
        [0, 149, 192],
        [190, 91, 17],
        [100, 255, 254],
        [255, 107, 96],
        [2, 194, 141],
        [223, 168, 255],
        [51, 136, 34],
        [240, 209, 255],
        [132, 131, 0],
        [106, 213, 255],
        [185, 137, 0],
        [0, 183, 216],
        [255, 212, 110],
        [1, 178, 188],
        [255, 170, 125],
        [1, 152, 123],
        [170, 97, 128],
        [236, 255, 163],
        [51, 131, 126],
        [255, 205, 158],
        [78, 129, 114],
        [255, 226, 233],
        [80, 131, 77],
        [173, 249, 255],
        [120, 123, 56],
        [249, 255, 210],
        [146, 112, 98],
    ][..]
    {
        colors.push_back(image::Rgb(*c));
    }
    let steps = 10000;
    for (from, to) in points
        .iter()
        .zip(points.iter().skip(1).chain(points.first()))
    {
        let color = colors.pop_back().unwrap();
        colors.push_front(color);
        for n in 0..steps {
            let f = n as f32 / steps as f32;
            let p = from * f + to * (1. - f);
            imgbuf[(p.x as u32, p.y as u32)] = color;
        }
    }
    let steps = 36000;
    for i in 0..steps {
        let i = (i as f32 / steps as f32) * TAU;
        let dir = Vector::new(i.sin(), i.cos());

        // let val = shape1.farthest_in_dir(dir);
        let val = kolli_desu::shapes::support(
            (&shape1, Point::new(0., 0.)),
            (&shape2, Point::new(0., 0.)),
            dir,
        );
        let p = transform(val);
        imgbuf[(p.x as u32, p.y as u32)] = image::Rgb([255, 0, 0]);
    }

    let zero = transform(Vector::new(0., 0.));
    for (x, y) in &[(0., 1.), (-1., 0.), (0., 0.), (1., 0.), (0., -1.)] {
        imgbuf[((zero.x + x) as u32, (zero.y + y) as u32)] = image::Rgb([0, 255, 0]);
    }

    let p = "visualise.png";
    let fout = Path::new(&p);

    let _ = image::ImageRgb8(imgbuf).save(fout);

    let mut imgbuf = image::ImageBuffer::from_pixel(imgx, imgy, image::Rgb([25, 25, 25]));
    let steps = 36000;
    for i in 0..steps {
        let i = (i as f32 / steps as f32) * TAU;
        let dir = Vector::new(i.sin(), i.cos());

        let val = shape1.farthest_in_dir(dir);
        let p = transform(val);
        imgbuf[(p.x as u32, p.y as u32)] = image::Rgb([255, 0, 0]);
    }
    for i in 0..steps {
        let i = (i as f32 / steps as f32) * TAU;
        let dir = Vector::new(i.sin(), i.cos());

        let val = shape2.farthest_in_dir(dir);
        let p = transform(val);
        imgbuf[(p.x as u32, p.y as u32)] = image::Rgb([0, 0, 255]);
    }

    let steps = 10000;

    let val = shape1.farthest_in_dir(penetration);
    let from = transform(val);
    let to = transform(val - penetration * depth);
    for n in 0..steps {
        let f = n as f32 / steps as f32;
        let p = from * f + to * (1. - f);
        imgbuf[(p.x as u32, p.y as u32)] =
            image::Rgb([(255. * f) as u8, 255, (255. * (1. - f)) as u8]);
    }

    let p = "visualise_orig.png";
    let fout = Path::new(&p);

    let _ = image::ImageRgb8(imgbuf).save(fout);
}
