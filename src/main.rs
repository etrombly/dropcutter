use anyhow::{anyhow, Result};
use clap::arg_enum;
use float_cmp::approx_eq;
use indicatif::{MultiProgress, ProgressBar, ProgressDrawTarget, ProgressStyle, TickTimeLimit};
use printer_geo::{compute::*, geo::*, stl::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{Read, Write},
    path::PathBuf,
    thread,
    time::Duration,
};
use structopt::StructOpt;

arg_enum! {
    #[derive(Debug)]
    enum ToolType {
        Endmill,
        VBit,
        Ball
    }
}

impl ToolType {
    pub fn create(&self, radius: f32, angle: Option<f32>, scale: f32) -> Tool {
        match self {
            ToolType::Endmill => Tool::new_endmill(radius, scale),
            ToolType::VBit => {
                Tool::new_v_bit(radius, angle.expect("V-Bit requires tool angle"), scale)
            },
            ToolType::Ball => Tool::new_ball(radius, scale),
        }
    }
}

fn parse_stepover(src: &str) -> Result<f32> {
    let stepover = src.parse::<f32>()?;
    if stepover < 1. || stepover > 100. {
        Err(anyhow!("stepover must be between 1 and 100"))
    } else {
        Ok(stepover)
    }
}

fn parse_angle(src: &str) -> Result<f32> {
    let angle = src.parse::<f32>()?;
    if angle < 1. || angle > 180. {
        Err(anyhow!("angle must be between 1 and 180"))
    } else {
        Ok(angle)
    }
}

fn parse_resolution(src: &str) -> Result<f32> {
    let resolution = src.parse::<f32>()?;
    if resolution < 0.001 || resolution > 1. {
        Err(anyhow!("resolution must be between 0.001 and 1.0"))
    } else {
        Ok(resolution)
    }
}

// set up program arguments
#[derive(Debug, StructOpt)]
#[structopt(name = "Dropcutter")]
struct Opt {
    #[structopt(short, long, parse(from_os_str))]
    input: PathBuf,

    #[structopt(short, long, parse(from_os_str))]
    output: PathBuf,

    #[structopt(short, long, parse(from_os_str))]
    heightmap: Option<PathBuf>,

    #[structopt(short, long)]
    diameter: f32,

    #[structopt(long)]
    debug: bool,

    #[structopt(short, long, parse(try_from_str = parse_angle))]
    angle: Option<f32>,

    #[structopt(long, default_value="100", parse(try_from_str = parse_stepover))]
    stepover: f32,

    #[structopt(short, long, default_value="0.05", parse(try_from_str = parse_resolution))]
    resolution: f32,

    #[structopt(short, long)]
    stepdown: Option<f32>,

    #[structopt(short, long, possible_values = &ToolType::variants(), default_value="ball", case_insensitive = true)]
    tool: ToolType,
}

pub fn generate_heightmap(
    tests: Vec<Vec<PointVk>>,
    partition: Vec<Vec<TriangleVk>>,
    bar: &ProgressBar,
    total_bar: &ProgressBar,
    vk: &Vk,
) -> Vec<Vec<PointVk>> {
    tests
        .iter()
        .enumerate()
        .map(|(column, test)| {
            // ray cast on the GPU to figure out the highest point for each point in this
            // column
            bar.inc(1);
            total_bar.tick();
            compute_drop(&partition[column], &test, &vk).unwrap()
        })
        .collect()
}

fn main() -> Result<()> {
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();

    let mp = MultiProgress::with_draw_target(ProgressDrawTarget::stderr_nohz());
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

    let _ = thread::spawn(move || loop {
        mp.tick(TickTimeLimit::Indefinite).unwrap();
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
        let mut file = File::create("v_bit.xyz")?;
        let output = tool
            .points
            .iter()
            .map(|x| format!("{:.3} {:.3} {:.3}\n", x[0], x[1], x[2]))
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes())?;
    }

    // get the bounds for the model
    let bounds = get_bounds(&triangles);
    // shift the model lower edge to x: 0 y: 0 and z_max to 0
    triangles
        .par_iter_mut()
        .for_each(|tri| tri.translate(-bounds.p1.x, -bounds.p1.y, -bounds.p2.z));
    // get new bounds
    let bounds = get_bounds(&triangles);

    // trnaslate triangles to a vulkan friendly format
    let tri_vk: Vec<TriangleVk> = to_tri_vk(&triangles);

    // can't step by floats in rust, so need to scale up
    // TODO: scaling by 20 gives .05mm precision, is that good enough?
    let max_x = (bounds.p2.x * scale) as i32;
    let min_x = (bounds.p1.x * scale) as i32;
    let max_y = (bounds.p2.y * scale) as i32;
    let min_y = (bounds.p1.y * scale) as i32;

    // create the test points for the height map
    // TODO: add a spiral pattern
    let tests: Vec<Vec<PointVk>> = (min_x..=max_x)
        .map(|x| {
            (min_y..=max_y)
                .map(move |y| PointVk::new(x as f32 / scale, y as f32 / scale, bounds.p1.z))
                .collect::<Vec<_>>()
        })
        .collect();

    // create height map

    let columns: Vec<_> = tests
        .iter()
        .map(|test| {
            // bounding box for this column
            LineVk {
                p1: PointVk::new(
                    ((test[0][0] - radius) * 100.).round() / 100.,
                    min_y as f32 / scale,
                    0.,
                ),
                p2: PointVk::new(
                    ((test[0][0] + radius) * 100.).round() / 100.,
                    max_y as f32 / scale,
                    0.,
                ),
            }
        })
        .collect();

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
    height_bar.set_length(tests.len() as u64);
    height_bar.set_style(
        ProgressStyle::default_bar().template("[2/5] Computing height map {bar:40.cyan/blue}"),
    );
    height_bar.reset_elapsed();
    let results: Vec<Vec<_>> = match opt.heightmap {
        Some(file) => {
            let mut file = File::open(file).unwrap();
            let mut buffer = Vec::new();
            file.read_to_end(&mut buffer).unwrap();
            let map: Vec<Vec<PointVk>> = bincode::deserialize(&buffer).unwrap();
            if map.len() != tests.len() && map[0].len() != tests[0].len() {
                println!("Input heightmap does not match stl or resolution, recomputing");
                generate_heightmap(tests, partition, &height_bar, &total_bar, &vk)
            } else {
                map
            }
        },
        _ => generate_heightmap(tests, partition, &height_bar, &total_bar, &vk),
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
        let encoded = bincode::serialize(&results).unwrap();
        let mut file = File::create("out.map")?;
        file.write_all(&encoded).unwrap();
    }

    let clock = std::time::Instant::now();
    let columns = results.len();
    let rows = results[0].len();
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

                                results[x_offset as usize][y_offset as usize][2]
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

        let output = results
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
    file.write_all(output.as_bytes())?;
    Ok(())
}
