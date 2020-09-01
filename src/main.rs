use float_cmp::approx_eq;
use printer_geo::{compute::*, geo::*, util::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{BufReader, Error, ErrorKind, Result, Write},
};

// pub struct Tool {
// pub points: Vec<Line3d>,
// pub circle: Circle,
// }
//
// impl Tool {
// pub fn new_endmill(radius: f32) -> Tool {
// let circle = Circle::new(Point3d::new(radius, radius, 0.0), radius);
// let points: Vec<Line3d> = (0..(radius * 20.0) as i32)
// .flat_map(|x| {
// (0..(radius * 20.0) as i32).map(move |y| Line3d {
// p1: Point3d::new(x as f32 / 10.0, y as f32 / 10.0, 0.0),
// p2: Point3d::new(0.0, 0.0, -1.0),
// })
// })
// .filter(|x| circle.in_2d_bounds(&x.p1))
// .collect();
// Tool { points, circle }
// }
// }

fn main() {
    // let radius = 1.0;
    // let tool = Tool::new_endmill(radius);
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

    ///// REGULAR
    // TODO: add option for overlap on passes
    // let bounds = get_bounds(&triangles);
    // let max_x = bounds.p2.x as i32;
    // let min_x = bounds.p1.x as i32;
    // let max_y = (bounds.p2.y * 10.) as i32;
    // let min_y = (bounds.p1.y * 10.) as i32;
    // let tests: Vec<Vec<_>> = (min_x..max_x)
    // .step_by(radius as usize)
    // .map(|x| {
    // (min_y..max_y)
    // .map(move |y| Line3d {
    // p1: Point3d::new(x as f32, y as f32 / 10.0, 200.0),
    // p2: Point3d::new(0.0, 0.0, -1.0),
    // })
    // .collect()
    // })
    // .collect();
    // println!("{:?} {:?}", triangles[0], bboxs[0]);
    // let intersects: Vec<&Triangle3d> = triangles.par_iter().filter(|x|
    // x.bbox().in_2d_bounds(&test)).collect(); println!("{:?} {:?}\n{:?}",
    // triangles.len(), intersects.len(), intersects); let line = Line3d{p1:
    // test, p2: Point3d::new(0.0, 0.0, -1.0)}; let points: Vec<Point3d> =
    // triangles.par_iter().filter_map(|x| x.intersect(line)).collect();
    // let mut result = Vec::new();
    // for line in tests {
    // let bounds = Line3d {
    // p1: Point3d::new(line[0].p1.x - radius, min_y as f32 / radius, 0.0),
    // p2: Point3d::new(line[0].p1.x + radius, max_y as f32 / radius, 0.0),
    // };
    // let intersects: Vec<&Triangle3d> = triangles
    // .par_iter()
    // .filter(|x| x.in_2d_bounds(&bounds))
    // .collect();
    // for point in &line {
    // let bbox = Line3d {
    // p1: tool.circle.bbox().p1 + point.p1 - tool.circle.center,
    // p2: tool.circle.bbox().p2 + point.p1 - tool.circle.center,
    // };
    // let filtered: Vec<_> = intersects
    // .par_iter()
    // .filter(|x| x.in_2d_bounds(&bbox))
    // .collect();
    // Just do one point per step
    // let points: Vec<Point3d> = filtered
    // .par_iter()
    // .filter_map(|x| x.intersect(*point))
    // .collect();
    // Use tool instead
    //
    // let points: Vec<Point3d> = tool
    // .points
    // .par_iter()
    // .flat_map(|points| {
    // filtered
    // .par_iter()
    // .filter_map(move |tri| tri.intersect(*points + *point))
    // })
    // .collect();
    // /
    // let max = points.iter().map(|x| x.z).fold(f32::NAN, f32::max);
    // if !max.is_nan() {
    // result.push(Point3d {
    // x: point.p1.x,
    // y: point.p1.y,
    // z: max,
    // });
    // }
    // }
    // }
    // let mut file = File::create("pcl.xyz").unwrap();
    // let output = result
    // .iter()
    // .map(|x| format!("{:.3} {:.3} {:.3}\n", x.x, x.y, x.z))
    // .collect::<Vec<String>>()
    // .join("");
    // file.write_all(output.as_bytes()).unwrap();

    ////// COMPUTE
    let vk = init_vk();
    let radius = 1.0;
    let tool = Tool::new_endmill(radius);
    {
        let mut file = File::create("endmill.xyz").unwrap();
        let output = tool
            .points
            .iter()
            .map(|x| {
                format!(
                    "{:.3} {:.3} {:.3}\n",
                    x.position[0], x.position[1], x.position[2]
                )
            })
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes()).unwrap();

        println!("tool bounds: {:?}", tool.bbox);
    }
    let bounds = get_bounds(&triangles);
    /*
    println!("{:?}", bounds);
    let triangles: Vec<_> = triangles
        .par_iter()
        .map(|x| {
            Triangle3d::new(
                (
                    x.p1.x - bounds.p1.x,
                    x.p1.y - bounds.p1.y,
                    x.p1.z,
                ),
                (
                    x.p2.x - bounds.p1.x,
                    x.p2.y - bounds.p1.y,
                    x.p2.z - bounds.p1.z,
                ),
                (
                    x.p3.x - bounds.p1.x,
                    x.p3.y - bounds.p1.y,
                    x.p3.z - bounds.p1.z,
                ),
            )
        })
        .collect();
    let bounds = get_bounds(&triangles);
    println!("{:?}", bounds);
    */
    let tri_vk = to_tri_vk(&triangles);

    let max_x = (bounds.p2.x * 10.) as i32;
    let min_x = (bounds.p1.x * 10.) as i32;
    let max_y = (bounds.p2.y * 10.) as i32;
    let min_y = (bounds.p1.y * 10.) as i32;
    let mut rev = false;
    let tests: Vec<PointVk> = (min_x..max_x)
        .step_by((radius * 10.) as usize)
        .flat_map(|x| {
            rev = !rev;
            if rev {
            (min_y..max_y).rev()
                .map(move |y| PointVk::new(x as f32 / 10.0, y as f32 / 10.0, bounds.p2.z + 1.))
                .collect::<Vec<_>>()
            } else {
                (min_y..max_y)
                .map(move |y| PointVk::new(x as f32 / 10.0, y as f32 / 10.0, bounds.p2.z + 1.))
                .collect::<Vec<_>>()
            }
        })
        .collect();
    println!(
        "tris: {} tests: {} bounds: {:?}",
        triangles.len(),
        tests.len(),
        bounds
    );
    let result = compute_drop(&tri_vk, &tests, tool, &vk);

    let mut file = File::create("pcl.xyz").unwrap();
    let output = result
        .iter()
        .map(|x| {
            format!(
                "{:.3} {:.3} {:.3}\n",
                x.position[0], x.position[1], x.position[2]
            )
        })
        .collect::<Vec<String>>()
        .join("");
    file.write_all(output.as_bytes()).unwrap();

    let mut file = File::create("out.gcode").unwrap();
    let mut last = result[0];
    let mut last_diff = 0;
    let mut output = format!(
        "G0 X{:.3} Y{:.3} Z{:.3}\n",
        last.position[0], last.position[1], last.position[2]
    )
    .to_string();
    for line in result {
        let diff = difference(&last, &line);
        // if check_diff(diff, last_diff) {
        // output.push_str(&format!(
        //    "G0 X{:.3} Y{:.3} Z{:.3}\n",
        //    last.position[0], last.position[1], last.position[2]
        //));
        output.push_str(&format!(
            "G0 X{:.3} Y{:.3} Z{:.3}\n",
            line.position[0], line.position[1], line.position[2]
        ));
        //}
        last_diff = diff;
        last = line;
    }
    file.write_all(output.as_bytes()).unwrap();
}

pub fn difference(left: &PointVk, right: &PointVk) -> u8 {
    let mut result = 0;
    if !approx_eq!(f32, left.position[0], right.position[0], ulps = 3) {
        result += 1
    }
    if !approx_eq!(f32, left.position[1], right.position[1], ulps = 3) {
        result += 2
    }
    if !approx_eq!(f32, left.position[2], right.position[2], ulps = 3) {
        result += 4
    }
    result
}

pub fn check_diff(current: u8, last: u8) -> bool {
    if current != last {
        true
    } else if [1, 2, 4].contains(&current) {
        false
    } else {
        true
    }
}
