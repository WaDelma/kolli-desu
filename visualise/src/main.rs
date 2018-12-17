use std::path::Path;

use kolli_desu::{Point, Vector, shapes::{Circle, ConvexPolygon, Aabb, Shape}};

const TAU: f32 = 2. * ::std::f32::consts::PI;

fn main() {
    let imgx = 2000;//1920;
    let imgy = 2000;//1080;

    // let shape1 = ConvexPolygon::new_line_segment(Point::new(-1., -1.), Point::new(1., 1.));
    // let shape2 = ConvexPolygon::new_line_segment(Point::new(1., -1.), Point::new(-1., 1.));
    // let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0.5, 0.5));
    // let shape2 = Circle::new(Point::new(0.1, 0.1), 1.);
    let shape1 = Aabb::new(Point::new(-0.5, -0.5), Point::new(0.5, 0.5));
    let shape2 = ConvexPolygon::new_rectangle(Point::new(-1., -1.), Point::new(-0.4, 0.), 1.);
    // let shape1 = ConvexPolygon::new_rectangle(Point::new(0., 0.), Point::new(1., 1.), 2f32.sqrt());
    // let shape2 = Circle::new(Point::new(2., 1.), 1.1);

    let mut imgbuf = image::ImageBuffer::from_pixel(imgx, imgy, image::Rgb([25, 25, 25]));

    let scale = 200.;
    let transform = |v: Vector<_>| {
        Vector::new(
            imgx as f32 / 2. + v.x * scale,
            imgy as f32 / 2. - v.y * scale
        )
    };

    let (_, simplex) = ::kolli_desu::gjk::collides_internal((&shape1, Point::new(0., 0.)), (&shape2, Point::new(0., 0.)));

    let points = simplex.into_iter().map(transform).collect::<Vec<_>>();
    let mut colors = vec![
        image::Rgb([255, 255,   0]),
        image::Rgb([  0, 255, 255]),
        image::Rgb([255,   0, 255])
    ];
    let steps = 1000;
    for (from, to) in points.iter().zip(points.iter().skip(1).chain(points.first())) {
        let color = colors.pop().unwrap();
        for n in 0..steps {
            let f = n as f32 / steps as f32;
            let p = from * f + to * (1. - f);
            imgbuf[(p.x as u32, p.y as u32)] = color;
        }
    }
    
    let steps = 3600;
    for i in 0..steps {
        let i = (i as f32 / steps as f32) * TAU;
        let dir = Vector::new(i.sin(), i.cos());

        // let val = shape1.farthest_in_dir(dir);
        let val = kolli_desu::gjk::support((&shape1, Point::new(0., 0.)), (&shape2, Point::new(0., 0.)), dir);
        let p = transform(val);
        imgbuf[(p.x as u32, p.y as u32)] = image::Rgb([255, 0, 0]);
    }

    let zero = transform(Vector::new(0., 0.));
    for (x, y) in vec![(0., 1.), (-1., 0.), (0., 0.), (1., 0.), (0., -1.)] {
        imgbuf[((zero.x + x) as u32, (zero.y + y) as u32)] = image::Rgb([0, 255, 0]);
    }

    let p = format!("visualise.png");
    let fout = Path::new(&p);

    let _ = image::ImageRgb8(imgbuf).save(fout);
}