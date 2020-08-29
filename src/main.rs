use printer_geo::{geo::*, util::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{BufReader, Error, ErrorKind, Result, Write},
};

pub struct Tool {
    pub points: Vec<Line3d>,
    pub circle: Circle,
}

impl Tool {
    pub fn new_endmill(radius: f32) -> Tool {
        let circle = Circle::new(Point3d::new(radius, radius, 0.0), radius);
        let points: Vec<Line3d> = (0..(radius * 20.0) as i32)
            .flat_map(|x| {
                (0..(radius * 20.0) as i32).map(move |y| Line3d {
                    p1: Point3d::new(x as f32 / 10.0, y as f32 / 10.0, 0.0),
                    p2: Point3d::new(0.0, 0.0, -1.0),
                })
            })
            .filter(|x| circle.in_2d_bounds(&x.p1))
            .collect();
        Tool { points, circle }
    }
}

fn main() {
    let radius = 1.0;
    let tool = Tool::new_endmill(radius);
    // {
    // let mut file = File::create("endmill.xyz").unwrap();
    // let output = tool.points
    // .iter()
    // .map(|x| format!("{:.3} {:.3} {:.3}\n", x.x, x.y, x.z))
    // .collect::<Vec<String>>()
    // .join("");
    // file.write_all(output.as_bytes()).unwrap();
    // }

    let file = File::open("vt_flat.stl").unwrap();
    let mut buf_reader = BufReader::new(file);
    let stl = read_stl(&mut buf_reader).unwrap();
    let triangles = to_triangles3d(&stl);
    // TODO: add option for overlap on passes
    let max_x = (triangles
        .iter()
        .map(|x| x.bbox().p2.x)
        .fold(f32::NAN, f32::max)) as i32;
    let min_x = (triangles
        .iter()
        .map(|x| x.bbox().p1.x)
        .fold(f32::NAN, f32::min)) as i32;
    let max_y = (triangles
        .iter()
        .map(|x| x.bbox().p2.y)
        .fold(f32::NAN, f32::max)
        * 10.0) as i32;
    let min_y = (triangles
        .iter()
        .map(|x| x.bbox().p1.y)
        .fold(f32::NAN, f32::min)
        * 10.0) as i32;
    let tests: Vec<Vec<_>> = (min_x..max_x)
        .step_by(radius as usize)
        .map(|x| {
            (min_y..max_y)
                .map(move |y| Line3d {
                    p1: Point3d::new(x as f32, y as f32 / 10.0, 200.0),
                    p2: Point3d::new(0.0, 0.0, -1.0),
                })
                .collect()
        })
        .collect();
    // println!("{:?} {:?}", triangles[0], bboxs[0]);
    // let intersects: Vec<&Triangle3d> = triangles.par_iter().filter(|x|
    // x.bbox().in_2d_bounds(&test)).collect(); println!("{:?} {:?}\n{:?}",
    // triangles.len(), intersects.len(), intersects); let line = Line3d{p1:
    // test, p2: Point3d::new(0.0, 0.0, -1.0)}; let points: Vec<Point3d> =
    // triangles.par_iter().filter_map(|x| x.intersect(line)).collect();
    let mut result = Vec::new();
    for line in tests {
        let bounds = Line3d {
            p1: Point3d::new(line[0].p1.x - radius, min_y as f32 / radius, 0.0),
            p2: Point3d::new(line[0].p1.x + radius, max_y as f32 / radius, 0.0),
        };
        let intersects: Vec<&Triangle3d> = triangles
            .par_iter()
            .filter(|x| x.in_2d_bounds(&bounds))
            .collect();
        for point in &line {
            let bbox = Line3d {
                p1: tool.circle.bbox().p1 + point.p1 - tool.circle.center,
                p2: tool.circle.bbox().p2 + point.p1 - tool.circle.center,
            };
            let filtered: Vec<_> = intersects
                .par_iter()
                .filter(|x| x.in_2d_bounds(&bbox))
                .collect();
            // let points: Vec<Point3d> = filtered
            //    .par_iter()
            //    .filter_map(|x| x.intersect(*point))
            //    .collect();
            let points: Vec<Point3d> = tool
                .points
                .par_iter()
                .flat_map(|points| {
                    filtered
                        .par_iter()
                        .filter_map(move |tri| tri.intersect(*points + *point))
                })
                .collect();
            let max = points.iter().map(|x| x.z).fold(f32::NAN, f32::max);
            if !max.is_nan() {
                result.push(Point3d {
                    x: point.p1.x,
                    y: point.p1.y,
                    z: max,
                });
            }
        }
    }
    let mut file = File::create("pcl.xyz").unwrap();
    let output = result
        .iter()
        .map(|x| format!("{:.3} {:.3} {:.3}\n", x.x, x.y, x.z))
        .collect::<Vec<String>>()
        .join("");
    file.write_all(output.as_bytes()).unwrap();
}
