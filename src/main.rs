use anyhow::{anyhow, Result};
use clap::arg_enum;
use float_cmp::approx_eq;
use indicatif::ProgressBar;
use ndarray::arr2;
use printer_geo::{compute::*, geo::*, stl::*};
use rayon::prelude::*;
use std::{
    fs::File,
    io::{BufReader, Write},
    path::PathBuf,
};
use structopt::StructOpt;

// supported tool types
arg_enum! {
    #[derive(Debug)]
    enum ToolType {
        Endmill,
        VBit,
        Ball
    }
}

impl ToolType {
    pub fn create(&self, radius: f32, angle: Option<f32>) -> Tool {
        match self {
            ToolType::Endmill => Tool::new_endmill(radius),
            ToolType::VBit => Tool::new_v_bit(radius, angle.expect("V-Bit requires tool angle")),
            ToolType::Ball => Tool::new_ball(radius),
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

// set up program arguments
#[derive(Debug, StructOpt)]
#[structopt(name = "Dropcutter")]
struct Opt {
    #[structopt(short, long, parse(from_os_str))]
    input: PathBuf,

    #[structopt(short, long, parse(from_os_str))]
    output: PathBuf,

    #[structopt(short, long)]
    diameter: f32,

    #[structopt(short, long, parse(try_from_str = parse_angle))]
    angle: Option<f32>,

    #[structopt(long, default_value="100", parse(try_from_str = parse_stepover))]
    stepover: f32,

    #[structopt(short, long)]
    stepdown: Option<f32>,

    #[structopt(short, long, possible_values = &ToolType::variants(), default_value="ball", case_insensitive = true)]
    tool: ToolType,
}

fn main() -> Result<()> {
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();

    // open stl
    // TODO: give a nicer error if this isn't a valid stl
    let file = File::open(opt.input)?;
    let mut buf_reader = BufReader::new(file);
    let stl = read_stl(&mut buf_reader).unwrap();
    let mut triangles = to_triangles3d(&stl);

    // initialize vulkan
    let vk = Vk::new()?;

    // TODO: add support for multiple passes with different tools?
    let radius = opt.diameter / 2.;
    let stepover = opt.diameter * (opt.stepover / 100.);
    let tool = opt.tool.create(radius, opt.angle);

    // TODO: remove writing out the tool to a file once it's working well (or add it
    // as a debug option)
    {
        let mut file = File::create("v_bit.xyz")?;
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
    let tri_vk = to_tri_vk(&triangles);

    // can't step by floats in rust, so need to scale up
    // TODO: scaling by 10 gives .1mm precision, is that good enough?
    let max_x = (bounds.p2.x * 10.) as i32;
    let min_x = (bounds.p1.x * 10.) as i32;
    let max_y = (bounds.p2.y * 10.) as i32;
    let min_y = (bounds.p1.y * 10.) as i32;

    // used to track direction for column travel
    let mut rev = true;

    // create a set of points to check tool intersection
    // TODO: add a spiral pattern
    let tests: Vec<Vec<PointVk>> = (min_x..=max_x)
        .map(|x| {
            rev = !rev;

            // alternate direction every other column
            // TODO: find a better value for the Y sampling value .1mm now
            if rev {
                (min_y..=max_y)
                    .rev()
                    .map(move |y| PointVk::new(x as f32 / 10.00, y as f32 / 10.0, bounds.p1.z))
                    .collect::<Vec<_>>()
            } else {
                (min_y..=max_y)
                    .map(move |y| PointVk::new(x as f32 / 10.00, y as f32 / 10.0, bounds.p1.z))
                    .collect::<Vec<_>>()
            }
        })
        .collect();

    let bar = ProgressBar::new(tests.len() as u64);
    let results: Vec<Vec<_>> = tests
        .iter()
        .map(|test| {
            let filtered= tri_vk.par_iter().copied().filter(|tri| tri.filter_row(test[0].position[0] - radius, test[0].position[0] + radius)).collect::<Vec<_>>();
            bar.inc(1);
            compute_drop(&filtered, &test, &vk).unwrap()
        })
        .collect();
    bar.finish();

    println!(
        "{:?} {:?} {:?} {:?}",
        results[1][1],
        PointVk::new(1 as f32 * stepover, 1 as f32 / 10., 0.),
        tool.points[0],
        tool.points[0] + PointVk::new(1 as f32 * stepover, 1 as f32 / 10., 0.)
    );
    let processed: Vec<Vec<_>> = (0..results.len()).into_par_iter().step_by((radius * stepover) as usize)
        .map(|x| {
            (0..results[0].len())
                .map(|y| {
                    let max = tool
                        .points
                        .iter()
                        .map(|tpoint| {
                            let x_offset = (x as f32 + (tpoint.position[0] * stepover)) as i32;
                            let y_offset = (y as f32 + (tpoint.position[1] * 10.)) as i32;
                            if x_offset < results.len() as i32 && x_offset >= 0 && y_offset < results[0].len() as i32 && y_offset >= 0{
                                results[x_offset as usize][y_offset as usize].position[2] - tpoint.position[2]
                            } else {
                                bounds.p1.z
                            }
                        })
                        .fold(0. / 0., f32::max);
                    PointVk::new(x as f32 /10., y as f32 / 10., max)
                })
                .collect()
        })
        .collect();

    let mut file = File::create("pcl.xyz")?;

    let output = processed
        .iter()
        .flat_map(|column| {
            column.iter().map(|point| {
                format!(
                    "{:.3} {:.3} {:.3}\n",
                    point.position[0], point.position[1], point.position[2]
                )
            })
        })
        .collect::<Vec<String>>()
        .join("");
    file.write_all(output.as_bytes())?;

    // start multi-pass processing
    let stepdown = match opt.stepdown {
        Some(x) => x,
        None => bounds.p2.z - bounds.p1.z,
    };
    let steps = ((bounds.p2.z - bounds.p1.z) / stepdown) as u64;
    let points: Vec<Vec<Vec<_>>> = (1..steps + 1)
        .map(|step| {
            processed
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|x| match step as f32 * -stepdown {
                            z if z > x.position[2] => PointVk::new(x.position[0], x.position[1], z),
                            _ => *x,
                        })
                        .collect()
                })
                .collect()
        })
        .collect();

    let mut file = File::create(opt.output)?;
    let mut last = processed[0][0];
    // start by moving to max Z
    // TODO: add a safe travel height
    // TODO: add actual feedrate
    let mut output = format!("G1 Z{:.3} F300\n", bounds.p2.z);

    for layer in points {
        output.push_str(&format!(
            "G0 X{:.3} Y{:.3}\nG0 Z{:.3}\n",
            layer[0][0].position[0], layer[0][0].position[1], layer[0][0].position[2]
        ));
        for row in layer {
            output.push_str(&format!(
                "G0 X{:.3} Y{:.3} Z{:.3}\n",
                row[0].position[0], row[0].position[1], row[0].position[2]
            ));
            for point in row {
                if !approx_eq!(f32, last.position[1], point.position[1], ulps = 2)
                    || !approx_eq!(f32, last.position[2], point.position[2], ulps = 2)
                {
                    output.push_str(&format!(
                        "G1 X{:.3} Y{:.3} Z{:.3}\n",
                        point.position[0], point.position[1], point.position[2]
                    ));
                }
                last = point;
            }
            output.push_str(&format!("G0 Z{:.3}\n", bounds.p2.z));
        }
    }
    file.write_all(output.as_bytes())?;
    Ok(())
}
