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
    tests: Vec<Vec<Point3d>>,
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

fn main() -> Result<()> {
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();

    let mp = Arc::new(MultiProgress::with_draw_target(ProgressDrawTarget::stderr_nohz()));
    let partition_bar = mp.add(ProgressBar::new(0));
    partition_bar.set_style(ProgressStyle::default_bar().template("[1/5] Filtering mesh"));
    partition_bar.tick();
    let height_bar = mp.add(ProgressBar::new(0));
    height_bar.set_style(ProgressStyle::default_bar().template("[2/5] Computing height map"));
    height_bar.tick();
    let tool_bar = mp.add(ProgressBar::new(0));
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/5] Processing tool path"));
    tool_bar.tick();
    let layer_bar = mp.add(ProgressBar::new(0));
    layer_bar.set_style(ProgressStyle::default_bar().template("[4/5] Processing layers"));
    layer_bar.tick();
    let gcode_bar = mp.add(ProgressBar::new(0));
    gcode_bar.set_style(ProgressStyle::default_bar().template("[5/5] Processing Gcode"));
    gcode_bar.tick();
    let total_bar = mp.add(ProgressBar::new(0));
    total_bar.set_style(ProgressStyle::default_bar().template("Total elapsed: {elapsed_precise}"));
    total_bar.tick();

    let mp_child = mp.clone();
    let _ = thread::spawn(move || loop {
        mp_child.tick(TickTimeLimit::Indefinite).unwrap();
        thread::sleep(Duration::from_millis(10));
    });

    let scale = 1. / opt.resolution;

    // open stl
    // TODO: give a nicer error if this isn't a valid stl
    let mut triangles = stl_to_tri(&opt.input)?;

    // initialize vulkan
    let vk = Vk::new()?;

    // TODO: add support for multiple passes with different tools?
    let radius = opt.diameter / 2.;
    let stepover = opt.diameter * (opt.stepover / 100.);
    let tool = opt.tool.create(radius, opt.angle, scale);

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

    // create the test points for the height map
    let grid = generate_grid(&bounds, &scale);

    // create columns
    // TODO: add a spiral pattern
    let columns = generate_columns_chunks(&grid, &bounds, &opt.resolution, &scale);

    let clock = std::time::Instant::now();
    let partition = partition_tris(&triangles, &columns, &vk).unwrap();
    partition_bar.set_style(ProgressStyle::default_bar().template("[1/5] Filtering mesh elapsed: {elapsed}"));
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
                generate_heightmap(grid, partition, &height_bar, &total_bar, &vk)
            } else {
                map
            }
        },
        _ => generate_heightmap(grid, partition, &height_bar, &total_bar, &vk),
    };

    height_bar.set_style(ProgressStyle::default_bar().template("[2/5] Computing height map elapsed: {elapsed}"));
    height_bar.finish();
    if opt.debug {
        println!("drop time {:?}", clock.elapsed());
    }

    {
        // write out height map
        // TODO: add support for reading back in
        let encoded = bincode::serialize(&heightmap).unwrap();
        let mut file = File::create("out.map")?;
        file.write_all(&encoded).unwrap();
    }

    let clock = std::time::Instant::now();
    let segments = heightmap.len();
    let rows = heightmap[0].len();
    // process height map with selected tool to find heights
    let count = segments / (radius * stepover * scale).ceil() as usize;
    tool_bar.set_length(count as u64);
    tool_bar.reset_elapsed();
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/5] Processing tool path {bar:40.cyan/blue}"));
    let processed: Vec<Vec<_>> = ((radius * scale) as usize..segments)
        .into_par_iter()
        // space each column based on radius and stepover
        .step_by((radius * stepover * scale).ceil() as usize)
        .map(|x| {
            tool_bar.inc(1);
            total_bar.tick();
            // alternate direction for each column, have to collect into a vec to get types to match
            /*
            let steps = if column_num % 2 == 0 {
                ((radius * scale) as usize..rows).collect::<Vec<_>>().into_par_iter()
            } else {
                ((radius * scale) as usize..rows).rev().collect::<Vec<_>>().into_par_iter()
            };
            */
            let steps = (radius * scale) as usize..rows;
            steps
                .map(|y| {
                    let max = tool
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

                                heightmap[x_offset as usize][y_offset as usize].pos.z
                                    - tpoint.pos.z
                            } else {
                                bounds.p1.pos.z
                            }
                        })
                        .fold(f32::NAN, f32::max); // same as calling max on all the values for this tool to find the heighest
                    Point3d::new(x as f32 / scale, y as f32 / scale, max)
                })
                .collect()
        })
        .collect();
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/5] Processing tool path elapsed: {elapsed}"));
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

    let clock = std::time::Instant::now();
    // start multi-pass processing
    let stepdown = match opt.stepdown {
        Some(x) => x,
        None => bounds.p2.pos.z - bounds.p1.pos.z,
    };
    let steps = ((bounds.p2.pos.z - bounds.p1.pos.z) / stepdown) as u64;
    layer_bar.set_length(steps);
    layer_bar.reset_elapsed();
    layer_bar.set_style(ProgressStyle::default_bar().template("[4/5] Processing layers {bar:40.cyan/blue}"));
    let points: Vec<Vec<Vec<_>>> = (1..steps + 1)
        .map(|step| {
            layer_bar.inc(1);
            total_bar.tick();
            processed
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|x| match step as f32 * -stepdown {
                            z if z > x.pos.z => Point3d::new(x.pos.x, x.pos.y, z),
                            _ => *x,
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    layer_bar.set_style(ProgressStyle::default_bar().template("[4/5] Processing layers elapsed: {elapsed}"));
    layer_bar.finish();
    if opt.debug {
        println!("multi-pass processing {:?}", clock.elapsed());
    }

    let clock = std::time::Instant::now();
    gcode_bar.set_length(points.len() as u64);
    gcode_bar.reset_elapsed();
    gcode_bar.set_style(ProgressStyle::default_bar().template("[5/5] Processing Gcode {bar:40.cyan/blue}"));
    let mut file = File::create(opt.output)?;
    // start by moving to max Z
    // TODO: add actual feedrate
    let mut output = format!("G0 Z{:.2} F300\n", 0.);
    let mut last = Point3d::new(0., 0., 0.);

    for (layer_index, layer) in points.iter().enumerate() {
        gcode_bar.inc(1);
        total_bar.tick();
        let mut islands = get_islands(&layer, bounds.p2.pos.z - (stepdown * (layer_index as f32)));
        // TODO: sort islands by distance from each other
        for island in islands.iter_mut() {
            island.sort();

            let mut segments = partition_segments(&island, opt.diameter);
            let mut current_segment = get_next_segment(&mut segments, &last);

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
                    if !approx_eq!(f32, last.pos.x, point.pos.x, ulps = 3)
                        || !approx_eq!(f32, last.pos.z, point.pos.z, ulps = 3)
                    {
                        output.push_str(&format!(
                            "G1 X{:.2} Y{:.2} Z{:.2}\n",
                            point.pos.x, point.pos.y, point.pos.z
                        ));
                    }

                    last = point;
                }
                output.push_str(&format!(
                    "G1 X{:.2} Y{:.2} Z{:.2}\n",
                    last.pos.x, last.pos.y, last.pos.z
                ));
                current_segment = get_next_segment(&mut segments, &last);
            }
            output.push_str(&format!("G0 Z{:.2}\n", 0.));
        }
    }
    gcode_bar.set_style(ProgressStyle::default_bar().template("[5/5] Processing Gcode elapsed: {elapsed}"));
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

pub fn get_next_segment(segments: &mut Vec<Vec<Point3d>>, last: &Point3d) -> usize {
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
    current_segment
}
