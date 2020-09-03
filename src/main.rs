use float_cmp::approx_eq;
use printer_geo::{compute::*, geo::*, util::*};
use rayon::prelude::*;
use std::{
    fs::File,
    path::PathBuf,
    io::{BufReader, Error, ErrorKind,Write},
};
use structopt::StructOpt;
use anyhow::Result;

#[derive(Debug, StructOpt)]
#[structopt(name = "example", about = "An example of StructOpt usage.")]
struct Opt {
    /// Activate debug mode
    // short and long flags (-d, --debug) will be deduced from the field's name
    #[structopt(short, long, parse(from_os_str))]
    file: PathBuf,

    #[structopt(short, long)]
    diameter: f64,
}

fn main() -> Result<()>{
    // parse input args, may remove this once I build the GUI
    let opt = Opt::from_args();

    // open stl 
    let file = File::open(opt.file)?;
    let mut buf_reader = BufReader::new(file);
    let stl = read_stl(&mut buf_reader).unwrap();
    let triangles = to_triangles3d(&stl);
   
    // initialize vulkan
    let vk = init_vk();

    // TODO: change radius to diameter
    // TODO: add support for multiple passes with different tools? or just run multiple times
    // build tool
    let radius = opt.diameter;
    let angle = 30.; // 30 degree
    let tool = Tool::new_v_bit(radius, angle);
    //let tool = Tool::new_endmill(radius);

    // TODO: remove writing out the tool to a file once it's working well
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

        println!("tool bounds: {:?}", tool.bbox);
    }

    // get the bounds for the model
    let bounds = get_bounds(&triangles);
    let tri_vk = to_tri_vk(&triangles);
    let max_x = (bounds.p2.x * 20.) as i32;
    let min_x = (bounds.p1.x * 20.) as i32;
    let max_y = (bounds.p2.y * 10.) as i32;
    let min_y = (bounds.p1.y * 10.) as i32;
    let mut rev = false;
    let columns: Vec<_> = (min_x..max_x + 2)
    .step_by((radius * 20.) as usize)
    .map(|x| x as f32 / 20.)
    .collect();
    let tests: Vec<Vec<PointVk>> = (min_x..max_x + 2)
        .step_by((radius * 20.) as usize)
        .map(|x| {
            rev = !rev;
            
            if rev {
            (min_y..max_y + 1).step_by((radius * 10.) as usize).rev()
                .map(move |y| PointVk::new(x as f32 / 20.00, y as f32 / 10.0, bounds.p1.z))
                .collect::<Vec<_>>()
            } else {
                (min_y..max_y + 1).step_by((radius * 10.) as usize)
                .map(move |y| {
                    PointVk::new(x as f32 / 20.00, y as f32 / 10.0, bounds.p1.z)})
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
    
    let result: Vec<_> = tests.par_iter().zip(columns).flat_map(|(row, column)| {
        let bounds = LineVk{p1: PointVk::new(column - radius ,min_y as f32 / 10.0, 0.), p2: PointVk::new(column + radius,max_y as f32 / 10.0, 0.)};
        let tris = tri_vk.par_iter().filter(|x| x.in_2d_bounds(&bounds)).copied().collect::<Vec<_>>();
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
    file.write_all(output.as_bytes())?;
    Ok(())
}
