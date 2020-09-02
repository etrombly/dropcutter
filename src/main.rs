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

    let file = File::open("flower.stl").unwrap();
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
    let radius = 0.5;
    //let angle = 60.; // 60 degree
    //let tool = Tool::new_v_bit(radius, angle);
    let tool = Tool::new_endmill(radius);
    {
        let mut file = File::create("v_bit.xyz").unwrap();
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
    let tri_vk = to_tri_vk(&triangles);

    let max_x = (bounds.p2.x * 10.) as i32;
    let min_x = (bounds.p1.x * 10.) as i32;
    let max_y = (bounds.p2.y * 10.) as i32;
    let min_y = (bounds.p1.y * 10.) as i32;
    let mut rev = false;
    let columns: Vec<_> = (min_x..max_x + 2)
    .step_by((radius * 20.) as usize)
    .map(|x| x as f32 / 10.)
    .collect();
    let tests: Vec<Vec<PointVk>> = (min_x..max_x + 2)
        .step_by((radius * 20.) as usize)
        .map(|x| {
            rev = !rev;
            
            if rev {
            (min_y..max_y + 1).step_by((radius * 10.) as usize).rev()
                .map(move |y| PointVk::new(x as f32 / 10.00, y as f32 / 10.0, bounds.p1.z))
                .collect::<Vec<_>>()
            } else {
                (min_y..max_y + 1).step_by((radius * 10.) as usize)
                .map(move |y| {
                    PointVk::new(x as f32 / 10.00, y as f32 / 10.0, bounds.p1.z)})
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
    let result: Vec<_> = tests.iter().zip(columns).flat_map(|(row, column)| {
        let bounds = LineVk{p1: PointVk::new(column - radius ,min_y as f32 / 10.0, 0.), p2: PointVk::new(column + radius,max_y as f32 / 10.0, 0.)};
        let tris = tri_vk.iter().filter(|x| x.in_2d_bounds(&bounds)).copied().collect::<Vec<_>>();
        compute_drop(&tris, &row, &tool, &vk)
    }).collect();
    println!("tests: {:?} results: {:?}", tests.iter().map(|x| x.len()).sum::<usize>(), result.len());

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
    let mut output = format!(
        "G0 X{:.3} Y{:.3} Z{:.3}\n",
        last.position[0], last.position[1], last.position[2]
    )
    .to_string();
    for line in result {
        if !approx_eq!(f32, last.position[1], line.position[1], ulps = 2) || !approx_eq!(f32, last.position[1], line.position[1], ulps = 2) { 
            output.push_str(&format!(
                "G0 X{:.3} Y{:.3} Z{:.3}\n",
                line.position[0], line.position[1], line.position[2]
            ));
        }
        last = line;
    }
    file.write_all(output.as_bytes()).unwrap();
}
