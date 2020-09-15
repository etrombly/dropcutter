use anyhow::{anyhow, Result};
use float_cmp::approx_eq;
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle, TickTimeLimit};
use printer_geo::{compute::*, config::*, geo::*, stl::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{Read, Write},
    sync::Arc,
    thread,
    time::Duration,
};
use structopt::StructOpt;

/*
pub fn generate_heightmap(
    tests: Vec<Vec<PointVk>>,
    partition: Vec<Vec<TriangleVk>>,
    bar: &ProgressBar,
    total_bar: &ProgressBar,
    vk: &Vk,
) -> Vec<Vec<PointVk>> {
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
}*/

fn main() -> Result<()> {
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();

    let mp = Arc::new(MultiProgress::with_draw_target(
        ProgressDrawTarget::stderr_nohz(),
    ));
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

    heightmap(&triangles, &vk);
    return Ok(());
    /*

    // TODO: add support for multiple passes with different tools?
    let radius = opt.diameter / 2.;
    let stepover = opt.diameter * (opt.stepover / 100.);
    let tool = opt.tool.create(radius, opt.angle, scale);

    if opt.debug {
        let mut file = File::create("v_bit.xyz")?;
        let output = tool
            .points
            .iter()
            .map(|x| format!("{:.3} {:.3} {:.3}\n", x[0], x[1], x[2]))
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes())?;
    }

    // shift the model lower edge to x: 0 y: 0 and z_max to 0
    move_to_zero(&mut triangles);
    // get bounds for the model
    let bounds = get_bounds(&triangles);

    // trnaslate triangles to a vulkan friendly format
    let tri_vk: Vec<TriangleVk> = to_tri_vk(&triangles);

    // create the test points for the height map
    let grid = generate_grid(&bounds, &scale);

    // create columns
    // TODO: add a spiral pattern
    let columns = generate_columns_chunks(&grid, &bounds, &opt.resolution, &scale);

    let clock = std::time::Instant::now();
    let partition = partition_tris(&tri_vk, &columns, &vk).unwrap();
    partition_bar.set_style(
        ProgressStyle::default_bar().template("[1/5] Filtering mesh elapsed: {elapsed}"),
    );
    partition_bar.finish();
    total_bar.tick();
    if opt.debug {
        println!("partition time {:?}", clock.elapsed());
    }

    let clock = std::time::Instant::now();
    height_bar.set_length(grid.len() as u64 / 10);
    height_bar.set_style(
        ProgressStyle::default_bar().template("[2/5] Computing height map {bar:40.cyan/blue}"),
    );
    height_bar.reset_elapsed();
    let heightmap: Vec<Vec<_>> = match opt.heightmap {
        Some(file) => {
            let mut file = File::open(file).unwrap();
            let mut buffer = Vec::new();
            file.read_to_end(&mut buffer).unwrap();
            let map: Vec<Vec<PointVk>> = bincode::deserialize(&buffer).unwrap();
            if map.len() != grid.len() && map[0].len() != grid[0].len() {
                println!("Input heightmap does not match stl or resolution, recomputing");
                generate_heightmap(grid, partition, &height_bar, &total_bar, &vk)
            } else {
                map
            }
        },
        _ => generate_heightmap(grid, partition, &height_bar, &total_bar, &vk),
    };
    height_bar.set_style(
        ProgressStyle::default_bar().template("[2/5] Computing height map elapsed: {elapsed}"),
    );
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
    let columns = heightmap.len();
    let rows = heightmap[0].len();
    // process height map with selected tool to find heights
    let count = columns / (radius * stepover * scale).ceil() as usize;
    tool_bar.set_length(count as u64);
    tool_bar.reset_elapsed();
    tool_bar.set_style(
        ProgressStyle::default_bar().template("[3/5] Processing tool path {bar:40.cyan/blue}"),
    );
    let processed: Vec<Vec<_>> = ((radius * scale) as usize..columns)
        .into_par_iter()
        // space each column based on radius and stepover
        .step_by((radius * stepover * scale).ceil() as usize)
        .enumerate()
        .map(|(column_num, x)| {
            tool_bar.inc(1);
            total_bar.tick();
            // alternate direction for each column, have to collect into a vec to get types to match
            let steps = if column_num % 2 == 0 {
                ((radius * scale) as usize..rows).collect::<Vec<_>>().into_par_iter()
            } else {
                ((radius * scale) as usize..rows).rev().collect::<Vec<_>>().into_par_iter()
            };
            steps
                .map(|y| {
                    let max = tool
                        .points
                        .iter()
                        .map(|tpoint| {
                            // for each point in the tool adjust it's location to the height map and calculate the intersection
                            let x_offset = (x as f32 + (tpoint[0] * scale)).round() as i32;
                            let y_offset = (y as f32 + (tpoint[1] * scale)).round() as i32;
                            if x_offset < columns as i32
                                && x_offset >= 0
                                && y_offset < rows as i32
                                && y_offset >= 0
                            {

                                heightmap[x_offset as usize][y_offset as usize][2]
                                    - tpoint[2]
                            } else {
                                bounds.p1.z
                            }
                        })
                        .fold(f32::NAN, f32::max); // same as calling max on all the values for this tool to find the heighest
                    PointVk::new(x as f32 / scale, y as f32 / scale, max)
                })
                .collect()
        })
        .collect();
    tool_bar.set_style(
        ProgressStyle::default_bar().template("[3/5] Processing tool path elapsed: {elapsed}"),
    );
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
                    .map(|point| format!("{:.3} {:.3} {:.3}\n", point[0], point[1], point[2]))
            })
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes())?;
    }

    let clock = std::time::Instant::now();
    // start multi-pass processing
    let stepdown = match opt.stepdown {
        Some(x) => x,
        None => bounds.p2.z - bounds.p1.z,
    };
    let steps = ((bounds.p2.z - bounds.p1.z) / stepdown) as u64;
    layer_bar.set_length(steps);
    layer_bar.reset_elapsed();
    layer_bar.set_style(
        ProgressStyle::default_bar().template("[4/5] Processing layers {bar:40.cyan/blue}"),
    );
    let points: Vec<Vec<Vec<_>>> = (1..steps + 1)
        .map(|step| {
            layer_bar.inc(1);
            total_bar.tick();
            processed
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|x| match step as f32 * -stepdown {
                            z if z > x[2] => PointVk::new(x[0], x[1], z),
                            _ => *x,
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    layer_bar.set_style(
        ProgressStyle::default_bar().template("[4/5] Processing layers elapsed: {elapsed}"),
    );
    layer_bar.finish();
    if opt.debug {
        println!("multi-pass processing {:?}", clock.elapsed());
    }

    let clock = std::time::Instant::now();
    gcode_bar.set_length(points.len() as u64);
    gcode_bar.reset_elapsed();
    gcode_bar.set_style(
        ProgressStyle::default_bar().template("[5/5] Processing Gcode {bar:40.cyan/blue}"),
    );
    let mut file = File::create(opt.output)?;
    let mut last = processed[0][0];
    // start by moving to max Z
    // TODO: add a safe travel height
    // TODO: add actual feedrate
    let mut output = format!("G1 Z{:.2} F300\n", bounds.p2.z);

    for layer in points {
        gcode_bar.inc(1);
        total_bar.tick();
        output.push_str(&format!(
            "G0 X{:.2} Y{:.2}\nG0 Z{:.2}\n",
            layer[0][0][0], layer[0][0][1], layer[0][0][2]
        ));
        for row in layer {
            output.push_str(&format!(
                "G0 X{:.2} Y{:.2}\nG0 Z{:.2}\n",
                row[0][0], row[0][1], row[0][2]
            ));
            for point in row {
                if !approx_eq!(f32, last[0], point[0], ulps = 2)
                    || !approx_eq!(f32, last[2], point[2], ulps = 2)
                {
                    output.push_str(&format!(
                        "G1 X{:.2} Y{:.3} Z{:.2}\n",
                        point[0], point[1], point[2]
                    ));
                }
                last = point;
            }
            output.push_str(&format!(
                "G1 X{:?} Y{:?} Z{:?}\nG0 Z{:.2}\n",
                last[0], last[1], last[2], bounds.p2.z
            ));
        }
    }
    gcode_bar.set_style(
        ProgressStyle::default_bar().template("[5/5] Processing Gcode elapsed: {elapsed}"),
    );
    gcode_bar.finish();
    if opt.debug {
        println!("gcode processing {:?}", clock.elapsed());
    }
    total_bar.finish();
    mp.join()?;
    file.write_all(output.as_bytes())?;
    */
    Ok(())
}
