use anyhow::Result;
use float_cmp::approx_eq;
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle, TickTimeLimit};
use printer_geo::{
    bfs::*,
    config::*,
    geo::*,
    stl::*,
    tsp::{nn::*, two_opt::*},
    vulkan::{
        compute::*,
        vkstate::{init_vulkan, VulkanState},
    },
};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{Read, Write},
    rc::Rc,
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
    vk: Rc<VulkanState>,
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
            &test.iter().flatten().copied().collect::<Vec<_>>(),
            vk.clone(),
        )
        .unwrap();
        //let tris = intersect_tris_fallback(
        //    &partition[column],
        //    &test.iter().flat_map(|x| x).copied().collect::<Vec<_>>(),
        //);
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
    let stepdown = opt.stepdown;
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
    let vk = Rc::new(init_vulkan()?);

    if opt.debug {
        let mut file = File::create("tool.xyz")?;
        let output = to_point_cloud(&tool.points);
        file.write_all(output.as_bytes())?;
    }

    // shift the model lower edge to x: 0 y: 0 and z_max to 0
    move_to_zero(&mut triangles);
    // get bounds for the model
    let bounds = get_bounds(&triangles);
    println!("{:#?}", bounds);

    // create the test points for the height map
    let grid = generate_grid(&bounds, &scale);

    // create columns
    let columns = generate_columns_chunks(&bounds, &scale);

    let clock = std::time::Instant::now();
    let partition = partition_tris(&triangles, &columns, vk.clone()).unwrap();
    //let partition = partition_tris_fallback(&triangles, &columns);
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
                generate_heightmap(&grid, partition, &height_bar, &total_bar, vk)
            } else {
                map
            }
        },
        _ => generate_heightmap(&grid, partition, &height_bar, &total_bar, vk),
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
    let mut layers: Vec<Vec<Vec<Point3d>>> = Vec::new();
    // process height map with selected tool to find heights
    tool_bar.reset_elapsed();
    let mut count = 1;
    tool_bar.set_style(ProgressStyle::default_bar().template("[3/4] Processing tool path {spinner} {msg}"));
    loop {
        tool_bar.set_message(&format!("pass {}", count));
        let mut current_layer_map = processed_map.clone();
        let layer: Vec<Vec<Point3d>> = ((radius * scale) as usize..segments)
        .into_par_iter()
        // space each column based on diameter and stepover
        .step_by((stepover * scale).ceil() as usize)
        .map(|x| {
            tool_bar.tick();
            total_bar.tick();
            let mut column = Vec::new();
            ((radius * scale) as usize..rows)
            .for_each(|y| {
                let mut max = tool
                .points
                .par_iter()
                .map(|tpoint| {
                    // for each point in the tool adjust it's location to the height map and calculate the intersection
                    let x_offset = (x as f32 + (tpoint.pos.x * scale)).round() as i32;
                    let y_offset = (y as f32 + (tpoint.pos.y * scale)).round() as i32;
                    if x_offset < segments as i32
                        && x_offset >= 0
                        && y_offset < rows as i32
                        && y_offset >= 0
                    {
                        // for each tool point find the amount still needed to mill
                        let mut diff = heightmap[x_offset as usize][y_offset as usize].pos.z
                                       - current_layer_map[x_offset as usize][y_offset as usize].pos.z;
                        // if amount is over the stepdown, clamp it
                        if diff < -stepdown {
                            diff = -stepdown;
                        }
                        // calculate the offset compared to current_layer_map[x][y]
                        diff = current_layer_map[x_offset as usize][y_offset as usize].pos.z + diff - tpoint.pos.z;
                        diff
                    } else {
                        // TODO: this should probably be bounds.p1.pos.z
                        f32::NAN
                    }
                }).reduce(|| f32::NAN, f32::max);
                // if the max depth is the same as where we are, filter this point out
                if approx_eq!(f32, max, current_layer_map[x][y].pos.z, ulps = 3) {
                    max = f32::NAN;
                }
                column.push(Point3d::new(x as f32 / scale, y as f32 / scale, max));
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
        if opt.debug {
            let mut file = File::create(format!("layer{}.xyz", count))?;

            let output = layer
                .par_iter()
                .map(|x| to_point_cloud(x))
                .collect::<Vec<String>>()
                .join("");
            file.write_all(output.as_bytes())?;
        }

        if layers.len() > 0 {
            if current_layer_map
                .par_iter()
                .zip(&processed_map)
                .all(|(column_cur, column_proc)| {
                    total_bar.tick();
                    column_cur
                        .par_iter()
                        .zip(column_proc)
                        .all(|(point_cur, point_proc)| point_cur == point_proc)
                })
            {
                break;
            }
        }

        layers.push(layer);

        if opt.debug {
            let mut file = File::create(format!("heightmap{}.xyz", count))?;

            let output = current_layer_map
                .par_iter()
                .map(|x| to_point_cloud(x))
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
            .par_iter()
            .map(|x| to_point_cloud(x))
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

        let stl = heightmap_to_stl(&heightmap)?;
        let mut file = File::create("heightmap.stl")?;
        file.write_all(&stl).unwrap();

        let encoded = bincode::serialize(&processed_map).unwrap();
        let mut file = File::create("rest.map")?;
        file.write_all(&encoded).unwrap();

        let stl = heightmap_to_stl(&processed_map)?;
        let mut file = File::create("restmap.stl")?;
        file.write_all(&stl).unwrap();
    }

    let clock = std::time::Instant::now();
    gcode_bar.set_length(layers.len() as u64);
    gcode_bar.reset_elapsed();
    gcode_bar.set_style(ProgressStyle::default_bar().template("[5/5] Processing Gcode {bar:40.cyan/blue}"));
    let mut file = File::create(opt.output)?;
    // start by moving to max Z
    // TODO: add actual feedrate
    let mut output = format!("G0 Z{:.3} F300\n", 0.);
    let mut last = Point3d::new(0., 0., 0.);

    for (layer_i, layer) in layers.iter().enumerate() {
        gcode_bar.inc(1);
        total_bar.tick();
        let mut islands = get_islands(&layer, opt.diameter);
        islands = nn(&islands, last);
        //islands = optimize_kopt(&islands, &last);
        if opt.debug {
            for (island_i, island) in islands.iter().enumerate() {
                let mut file = File::create(format!("island{}_{}.xyz", layer_i, island_i))?;
                let output = island
                    .iter()
                    .map(|segment| to_point_cloud(&segment))
                    .collect::<Vec<_>>()
                    .join("\n");
                file.write_all(output.as_bytes())?;
            }
        }
        for island in islands.iter_mut() {
            for segment in island.iter_mut() {
                // TODO: instead of retracting to safe height build a move to travel closer to
                // model if the segments aren't adjacent retract and travel to
                // next segment
                if distance(&segment[0].pos.xy(), &last.pos.xy()) > opt.diameter * 1.5 {
                    output.push_str(&format!(
                        "G0 Z0\nG0 X{:.3} Y{:.3}\n",
                        segment[0].pos.x, segment[0].pos.y
                    ));
                }

                // don't write a move if only Y changed
                for point in segment {
                    //if !approx_eq!(f32, last.pos.x, point.pos.x, ulps = 3)
                    //    || !approx_eq!(f32, last.pos.z, point.pos.z, ulps = 3)
                    //{
                    output.push_str(&format!(
                        "G1 X{:.3} Y{:.3} Z{:.3}\n",
                        point.pos.x, point.pos.y, point.pos.z
                    ));
                    //}

                    last = *point;
                }
                output.push_str(&format!(
                    "G1 X{:.3} Y{:.3} Z{:.3}\n",
                    last.pos.x, last.pos.y, last.pos.z
                ));
            }
            output.push_str(&format!("G0 Z{:.3}\n", 0.));
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
