use anyhow::Result;
use float_cmp::approx_eq;
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle, TickTimeLimit};
use printer_geo::{bfs::*, compute::*, config::*, stl::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{Read, Write},
    sync::Arc,
    thread,
    time::Duration,
};
use structopt::StructOpt;

pub fn generate_heightmap(
    tests: &[Vec<Point3d>],
    partition: Vec<Vec<Triangle3d>>,
    bar: &ProgressBar,
    total_bar: &ProgressBar,
    vk: &Vk,
) -> Vec<Vec<Point3d>> {
    let mut result = Vec::with_capacity(tests.len());
    for (column, test) in tests.chunks(10).enumerate() {
        // ray cast on the GPU to figure out the highest point for each point in this
        // column
        bar.inc(1);
        // TODO: there's probably a better way to process this in chunks
        total_bar.tick();
        let len = test[0].len();
        let tris = intersect_tris(
            &partition[column],
            &test.iter().flat_map(|x| x).copied().collect::<Vec<_>>(),
            &vk,
        )
        .unwrap();
        //intersect_tris_fallback(&partition[column], &test)
        for chunk in tris.chunks(len) {
            result.push(chunk.to_vec());
        }
    }
    result
}

pub fn generate_rest_map(heightmap: &[Vec<Point3d>]) -> Vec<Vec<Point3d>> {
    heightmap
        .iter()
        .map(|column| {
            column
                .iter()
                .map(|point| Point3d::new(point.pos.x, point.pos.y, 0.))
                .collect::<Vec<_>>()
        })
        .collect()
}

fn main() -> Result<()> {
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();
    let radius = opt.diameter / 2.;
    let stepover = opt.diameter * (opt.stepover / 100.);
    let scale = 1. / opt.resolution;
    let resolution = opt.resolution;
    let tool = opt.tool.create(opt.diameter, opt.angle, scale);

    let mp = Arc::new(MultiProgress::with_draw_target(ProgressDrawTarget::stderr_nohz()));
    let partition_bar = mp.add(ProgressBar::new(0));
    partition_bar.set_style(ProgressStyle::default_bar().template("[1/4] Filtering mesh"));
    partition_bar.tick();
    let height_bar = mp.add(ProgressBar::new(0));
    height_bar.set_style(ProgressStyle::default_bar().template("[2/4] Computing height map"));
    height_bar.tick();
    let tool_bar = mp.add(ProgressBar::new(0));
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/4] Processing tool path"));
    tool_bar.tick();
    let gcode_bar = mp.add(ProgressBar::new(0));
    gcode_bar.set_style(ProgressStyle::default_bar().template("[4/4] Processing Gcode"));
    gcode_bar.tick();
    let total_bar = mp.add(ProgressBar::new(0));
    total_bar.set_style(ProgressStyle::default_bar().template("Total elapsed: {elapsed_precise}"));
    total_bar.tick();

    let mp_child = mp.clone();
    let _ = thread::spawn(move || loop {
        mp_child.tick(TickTimeLimit::Indefinite).unwrap();
        thread::sleep(Duration::from_millis(10));
    });

    // open stl
    // TODO: give a nicer error if this isn't a valid stl
    let mut triangles = stl_to_tri(&opt.input)?;

    // initialize vulkan
    let vk = Vk::new()?;

    if opt.debug {
        let mut file = File::create("tool.xyz")?;
        let output = tool
            .points
            .iter()
            .map(|x| format!("{:.3} {:.3} {:.3}\n", x.pos.x, x.pos.y, x.pos.z))
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes())?;
    }

    // shift the model lower edge to x: 0 y: 0 and z_max to 0
    move_to_zero(&mut triangles);
    // get bounds for the model
    let bounds = get_bounds(&triangles);

    let stepdown = match opt.stepdown {
        Some(x) => x,
        None => bounds.p2.pos.z - bounds.p1.pos.z,
    };

    // create the test points for the height map
    let grid = generate_grid(&bounds, &scale);

    // create columns
    let columns = generate_columns_chunks(&grid, &bounds, &opt.resolution, &scale);

    let clock = std::time::Instant::now();
    let partition = partition_tris(&triangles, &columns, &vk).unwrap();
    partition_bar.set_style(ProgressStyle::default_bar().template("[1/4] Filtering mesh elapsed: {elapsed}"));
    partition_bar.finish();
    total_bar.tick();
    if opt.debug {
        println!("partition time {:?}", clock.elapsed());
    }

    let clock = std::time::Instant::now();
    height_bar.set_length(grid.len() as u64 / 10);
    height_bar.set_style(ProgressStyle::default_bar().template("[2/5] Computing height map {bar:40.cyan/blue}"));
    height_bar.reset_elapsed();
    let heightmap: Vec<Vec<_>> = match opt.heightmap {
        Some(file) => {
            let mut file = File::open(file).unwrap();
            let mut buffer = Vec::new();
            file.read_to_end(&mut buffer).unwrap();
            let map: Vec<Vec<Point3d>> = bincode::deserialize(&buffer).unwrap();
            if map.len() != grid.len() && map[0].len() != grid[0].len() {
                println!("Input heightmap does not match stl or resolution, recomputing");
                generate_heightmap(&grid, partition, &height_bar, &total_bar, &vk)
            } else {
                map
            }
        },
        _ => generate_heightmap(&grid, partition, &height_bar, &total_bar, &vk),
    };

    height_bar.set_style(ProgressStyle::default_bar().template("[2/4] Computing height map elapsed: {elapsed}"));
    height_bar.finish();
    if opt.debug {
        println!("drop time {:?}", clock.elapsed());
    }

    let clock = std::time::Instant::now();
    let segments = heightmap.len();
    let rows = heightmap[0].len();
    // TODO: read in previous map if rest milling
    let mut processed_map: Vec<Vec<_>> = match opt.restmap {
        Some(file) => {
            let mut file = File::open(file).unwrap();
            let mut buffer = Vec::new();
            file.read_to_end(&mut buffer).unwrap();
            let map: Vec<Vec<Point3d>> = bincode::deserialize(&buffer).unwrap();
            if map.len() != grid.len() && map[0].len() != grid[0].len() {
                println!("Input restmap does not match stl or resolution, recomputing");
                generate_rest_map(&heightmap)
            } else {
                map
            }
        },
        _ => generate_rest_map(&heightmap),
    };
    // TODO: rename to layers?
    let mut layers = Vec::new();
    // process height map with selected tool to find heights
    tool_bar.reset_elapsed();
    let mut count = 1;
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/4] Processing tool path {spinner} {msg}"));
    loop {
        tool_bar.set_message(&format!("pass {}", count));
        tool_bar.tick();
        total_bar.tick();
        let mut current_layer_map = processed_map.clone();
        let layer: Vec<Vec<Point3d>> = ((radius * scale) as usize..segments)
        .into_par_iter()
        // space each column based on diameter and stepover
        .step_by((stepover * scale).ceil() as usize)
        .map(|x| {
            let mut column = Vec::new();
            ((radius * scale) as usize..rows)
            .for_each(|y| {
                let mut max = tool
                .points
                .iter()
                .map(|tpoint| {
                    // for each point in the tool adjust it's location to the height map and calculate the intersection
                    let x_offset = (x as f32 + (tpoint.pos.x * scale)).round() as i32;
                    let y_offset = (y as f32 + (tpoint.pos.y * scale)).round() as i32;
                    if x_offset < segments as i32
                        && x_offset >= 0
                        && y_offset < rows as i32
                        && y_offset >= 0
                    {
                        if current_layer_map[x_offset as usize][y_offset as usize].pos.z > heightmap[x_offset as usize][y_offset as usize].pos.z {
                            heightmap[x_offset as usize][y_offset as usize].pos.z
                            - current_layer_map[x_offset as usize][y_offset as usize].pos.z
                            - tpoint.pos.z
                        } else {
                            0.
                        }
                    } else {
                        f32::NAN
                    }
                })
                .fold(f32::NAN, f32::max); // same as calling max on all the values for this tool to find the heighest
                if max.abs() > stepdown {
                    max = -stepdown;
                }
                if max.abs() < resolution {
                    max = f32::NAN;
                }
                column.push(Point3d::new(x as f32 / scale, y as f32 / scale, max + current_layer_map[x][y].pos.z));
            });
            column
        }).collect();
        layer.iter().for_each(|column| {
            column.iter().for_each(|point| {
                if !point.pos.z.is_nan() {
                    tool.points.iter().for_each(|tpoint| {
                        // for each point in the tool adjust it's location to the height map and
                        // calculate the intersection
                        let x_offset = ((point.pos.x + tpoint.pos.x) * scale).round() as i32;
                        let y_offset = ((point.pos.y + tpoint.pos.y) * scale).round() as i32;
                        if x_offset < segments as i32 && x_offset >= 0 && y_offset < rows as i32 && y_offset >= 0 {
                            let new_pos = point.pos.z + tpoint.pos.z;
                            if new_pos < current_layer_map[x_offset as usize][y_offset as usize].pos.z {
                                current_layer_map[x_offset as usize][y_offset as usize].pos.z = new_pos;
                            };
                        }
                    });
                }
            })
        });
        if layer
            .par_iter()
            .all(|column| column.par_iter().all(|point| point.pos.z.is_nan()))
        {
            break;
        }
        layers.push(layer);

        if opt.debug {
            let mut file = File::create(format!("layer{}.xyz", count))?;

            let output = current_layer_map
                .iter()
                .flat_map(|column| {
                    column
                        .iter()
                        .map(|point| format!("{:.3} {:.3} {:.3}\n", point.pos.x, point.pos.y, point.pos.z))
                })
                .collect::<Vec<String>>()
                .join("");
            file.write_all(output.as_bytes())?;
        }

        processed_map = current_layer_map.clone();
        count += 1;
    }

    tool_bar.set_style(ProgressStyle::default_bar().template("[3/4] Processing tool path {msg} elapsed: {elapsed}"));
    tool_bar.finish();
    if opt.debug {
        println!("tool time {:?}", clock.elapsed());
    }

    if opt.debug {
        // write out the height map to a point cloud for debugging
        let mut file = File::create("pcl.xyz")?;

        let output = heightmap
            .iter()
            .flat_map(|column| {
                column
                    .iter()
                    .map(|point| format!("{:.3} {:.3} {:.3}\n", point.pos.x, point.pos.y, point.pos.z))
            })
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes())?;
    }
    {
        // write out height map
        // TODO: add support for reading back in
        let encoded = bincode::serialize(&heightmap).unwrap();
        let mut file = File::create("height.map")?;
        file.write_all(&encoded).unwrap();

        let encoded = bincode::serialize(&processed_map).unwrap();
        let mut file = File::create("rest.map")?;
        file.write_all(&encoded).unwrap();
    }

    let clock = std::time::Instant::now();
    gcode_bar.set_length(layers.len() as u64);
    gcode_bar.reset_elapsed();
    gcode_bar.set_style(ProgressStyle::default_bar().template("[5/5] Processing Gcode {bar:40.cyan/blue}"));
    let mut file = File::create(opt.output)?;
    // start by moving to max Z
    // TODO: add actual feedrate
    let mut output = format!("G0 Z{:.2} F300\n", 0.);
    let mut last = Point3d::new(0., 0., 0.);

    for layer in layers.iter() {
        gcode_bar.inc(1);
        total_bar.tick();
        let mut islands = get_islands(&layer);
        let (mut current_island, _) = get_next_island(&islands, &last);
        // TODO: sort islands before processing
        while islands.len() > 0 {
            let mut island = islands.remove(current_island);
            island.sort();

            let mut segments = partition_segments(&island, opt.diameter);
            let (mut current_segment, _) = get_next_segment(&mut segments, &last);

            while segments.len() > 0 {
                let mut segment = segments.remove(current_segment);
                // TODO: instead of retracting to safe height build a move to travel closer to
                // model if the segments aren't adjacent retract and travel to
                // next segment
                if distance(&segment[segment.len() - 1].pos.xy(), &last.pos.xy()) > opt.diameter * 2. {
                    output.push_str(&format!(
                        "G0 Z0\nG0 X{:.2} Y{:.2}\n",
                        segment[segment.len() - 1].pos.x,
                        segment[segment.len() - 1].pos.y
                    ));
                }

                // don't write a move if only Y changed
                while let Some(point) = segment.pop() {
                    //if !approx_eq!(f32, last.pos.x, point.pos.x, ulps = 3)
                    //    || !approx_eq!(f32, last.pos.z, point.pos.z, ulps = 3)
                    //{
                    output.push_str(&format!(
                        "G1 X{:.2} Y{:.2} Z{:.2}\n",
                        point.pos.x, point.pos.y, point.pos.z
                    ));
                    //}

                    last = point;
                }
                output.push_str(&format!(
                    "G1 X{:.2} Y{:.2} Z{:.2}\n",
                    last.pos.x, last.pos.y, last.pos.z
                ));
                let tmp = get_next_segment(&mut segments, &last);
                current_segment = tmp.0;
                //let dist = tmp.1;
                // if we need to hop multiple columns see if there is a closer
                // island if dist > opt.diameter * 6. {
                //    let tmp = get_next_island(&islands, &last);
                //    if tmp.1 < dist {
                //        segments.push(segment);
                //        let mut tmp = Vec::new();
                //        segments.iter().for_each(|x| x.iter().for_each(|y|
                // tmp.push(*y)));        islands.push(tmp);
                //        break;
                //    }
                //}
            }
            output.push_str(&format!("G0 Z{:.2}\n", 0.));
            let tmp = get_next_island(&islands, &last);
            current_island = tmp.0;
        }
    }
    gcode_bar.set_style(ProgressStyle::default_bar().template("[4/4] Processing Gcode elapsed: {elapsed}"));
    gcode_bar.finish();
    if opt.debug {
        println!("gcode processing {:?}", clock.elapsed());
    }
    total_bar.finish();
    mp.join()?;
    file.write_all(output.as_bytes())?;
    Ok(())
}

pub fn partition_segments(data: &Vec<Point3d>, diameter: f32) -> Vec<Vec<Point3d>> {
    let mut output = Vec::new();
    let mut data = data.clone();
    while data.len() > 0 {
        let mut column = Vec::new();
        if let Some(current) = data.pop() {
            column.push(current);
            data = data
                .into_iter()
                .filter(|point| {
                    if approx_eq!(f32, current.pos.x, point.pos.x, ulps = 3) {
                        column.push(*point);
                        false
                    } else {
                        true
                    }
                })
                .collect();
            column.sort();
            let mut segment = Vec::new();
            let mut last = column[0];
            for point in column {
                if (point.pos.y - last.pos.y).abs() > diameter * 2. {
                    output.push(segment.clone());
                    segment.clear();
                }
                segment.push(point);
                last = point;
            }
            output.push(segment);
        }
    }
    output
}

pub fn get_next_segment(segments: &mut Vec<Vec<Point3d>>, last: &Point3d) -> (usize, f32) {
    let mut dist = f32::MAX;
    let mut current_segment = 0;
    // TODO: see if splitting line segments gives better results
    for (index, segment) in segments.iter_mut().enumerate() {
        let curr_dist = distance(&segment[segment.len() - 1].pos.xy(), &last.pos.xy());
        if curr_dist < dist {
            dist = curr_dist;
            current_segment = index;
        }
        let curr_dist = distance(&segment[0].pos.xy(), &last.pos.xy());
        if curr_dist < dist {
            dist = curr_dist;
            segment.reverse();
            current_segment = index;
        }
    }
    (current_segment, dist)
}

pub fn get_next_island(islands: &Vec<Vec<Point3d>>, last: &Point3d) -> (usize, f32) {
    let mut dist = f32::MAX;
    let mut current_segment = 0;
    // TODO: see if splitting line segments gives better results
    for (index, island) in islands.iter().enumerate() {
        let curr_dist = distance(&island[island.len() - 1].pos.xy(), &last.pos.xy());
        if curr_dist < dist {
            dist = curr_dist;
            current_segment = index;
        }
        let curr_dist = distance(&island[0].pos.xy(), &last.pos.xy());
        if curr_dist < dist {
            dist = curr_dist;
            current_segment = index;
        }
    }
    (current_segment, dist)
}
