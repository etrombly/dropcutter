use anyhow::Result;
use clap::arg_enum;
use float_cmp::approx_eq;
use printer_geo::{compute::*, stl::*, geo::*};
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

// set up program arguments
#[derive(Debug, StructOpt)]
#[structopt(name = "Droppcutter")]
struct Opt {
    #[structopt(short, long, parse(from_os_str))]
    input: PathBuf,

    #[structopt(short, long, parse(from_os_str))]
    output: PathBuf,

    #[structopt(short, long)]
    diameter: f32,

    #[structopt(short, long)]
    angle: Option<f32>,

    #[structopt(short, long, possible_values = &ToolType::variants(), case_insensitive = true)]
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
    let triangles = to_triangles3d(&stl);

    // initialize vulkan
    let vk = init_vk();

    // TODO: add support for multiple passes with different tools?
    let radius = opt.diameter / 2.;
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
    // TODO: shift the model lower edge to x: 0 y: 0
    let tri_vk = to_tri_vk(&triangles);

    // can't step by floats in rust, so need to scale up
    // TODO: scaling by 10 gives .1mm precision, is that good enough?
    let max_x = (bounds.p2.x * 20.) as i32;
    let min_x = (bounds.p1.x * 20.) as i32;
    let max_y = (bounds.p2.y * 10.) as i32;
    let min_y = (bounds.p1.y * 10.) as i32;

    // used to track direction for column travel
    let mut rev = false;

    // find the columns to use later for filtering triangles
    let columns: Vec<_> = (min_x..max_x + 2)
        .step_by((radius * 20.) as usize)
        .map(|x| x as f32 / 20.)
        .collect();

    // create a set of points to check tool intersection
    // TODO: add a spiral pattern
    let tests: Vec<Vec<PointVk>> = (min_x..max_x + 2)
        .step_by((radius * 20.) as usize)
        .map(|x| {
            rev = !rev;

            // alternate direction every other column
            if rev {
                (min_y..max_y + 1)
                    .step_by((radius * 10.) as usize)
                    .rev()
                    .map(move |y| PointVk::new(x as f32 / 20.00, y as f32 / 10.0, bounds.p1.z))
                    .collect::<Vec<_>>()
            } else {
                (min_y..max_y + 1)
                    .step_by((radius * 10.) as usize)
                    .map(move |y| PointVk::new(x as f32 / 20.00, y as f32 / 10.0, bounds.p1.z))
                    .collect::<Vec<_>>()
            }
        })
        .collect();

    let result: Vec<_> = tests
        .par_iter()
        .zip(columns)
        .flat_map(|(row, column)| {
            // find bounding box for this column
            let bounds = LineVk {
                p1: PointVk::new(column - radius, min_y as f32 / 10.0, 0.),
                p2: PointVk::new(column + radius, max_y as f32 / 10.0, 0.),
            };
            // filter mesh to only tris in this column
            let tris = tri_vk
                .par_iter()
                .filter(|x| x.in_2d_bounds(&bounds))
                .copied()
                .collect::<Vec<_>>();
            // check for highest Z intersection with tool for each point in this column
            compute_drop(&tris, &row, &tool, &vk)
        })
        .collect();

    // TODO: remove writing out the point cloud once done debugging, or add it as a
    // debug option
    let mut file = File::create("pcl.xyz")?;
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
    file.write_all(output.as_bytes())?;

    let mut file = File::create(opt.output)?;
    let mut last = result[0];
    // start by moving to max Z
    // TODO: add a safe travel height
    let mut output = format!("G1 Z{:.3}\n", bounds.p2.z);

    // Move to initial X,Y then plunge to initial Z
    output.push_str(&format!(
        "G0 X{:.3} Y{:.3}\nG0 Z{:.3}\n",
        last.position[0], last.position[1], last.position[2]
    ));
    for line in result {
        if !approx_eq!(f32, last.position[1], line.position[1], ulps = 2)
            || !approx_eq!(f32, last.position[1], line.position[1], ulps = 2)
        {
            output.push_str(&format!(
                "G1 X{:.3} Y{:.3} Z{:.3}\n",
                line.position[0], line.position[1], line.position[2]
            ));
        }
        last = line;
    }
    file.write_all(output.as_bytes())?;
    Ok(())
}
