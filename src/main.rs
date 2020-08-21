use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use printer_geo::*;
use rayon::prelude::*;
use std::fs::File;
use std::io::BufReader;
use std::io::{Error, ErrorKind, Result, Write};

use vulkano::buffer::{BufferUsage, CpuAccessibleBuffer};
use vulkano::command_buffer::AutoCommandBufferBuilder;
use vulkano::descriptor::descriptor_set::PersistentDescriptorSet;
use vulkano::descriptor::PipelineLayoutAbstract;
use vulkano::device::{Device, DeviceExtensions};
use vulkano::instance::{Instance, InstanceExtensions, PhysicalDevice};
use vulkano::pipeline::ComputePipeline;
use vulkano::sync;
use vulkano::sync::GpuFuture;

use std::ops::Index;
use std::sync::Arc;

#[derive(Default, Debug, Clone, Copy)]
pub struct Point {
    position: [f32; 3],
}

impl Index<usize> for Point {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        &self.position[index]
    }
}

pub struct Triangle {
    normal: Point,
    v1: Point,
    v2: Point,
    v3: Point,
    attr_byte_count: u16,
}

fn point_eq(lhs: Point, rhs: Point) -> bool {
    lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2]
}

impl PartialEq for Triangle {
    fn eq(&self, rhs: &Triangle) -> bool {
        point_eq(self.normal, rhs.normal)
            && point_eq(self.v1, rhs.v1)
            && point_eq(self.v2, rhs.v2)
            && point_eq(self.v3, rhs.v3)
            && self.attr_byte_count == rhs.attr_byte_count
    }
}

impl Eq for Triangle {}

pub struct BinaryStlHeader {
    pub header: [u8; 80],
    pub num_triangles: u32,
}

pub struct BinaryStlFile {
    pub header: BinaryStlHeader,
    pub triangles: Vec<Triangle>,
}

fn read_point<T: ReadBytesExt>(input: &mut T) -> Result<Point> {
    let x1 = input.read_f32::<LittleEndian>()?;
    let x2 = input.read_f32::<LittleEndian>()?;
    let x3 = input.read_f32::<LittleEndian>()?;

    Ok(Point {
        position: [x1, x2, x3],
    })
}

fn read_triangle<T: ReadBytesExt>(input: &mut T) -> Result<Triangle> {
    let normal = read_point(input)?;
    let v1 = read_point(input)?;
    let v2 = read_point(input)?;
    let v3 = read_point(input)?;
    let attr_count = input.read_u16::<LittleEndian>()?;

    Ok(Triangle {
        normal,
        v1,
        v2,
        v3,
        attr_byte_count: attr_count,
    })
}

fn read_header<T: ReadBytesExt>(input: &mut T) -> Result<BinaryStlHeader> {
    let mut header = [0u8; 80];

    match input.read(&mut header) {
        Ok(n) => {
            if n == header.len() {
                ()
            } else {
                return Err(Error::new(ErrorKind::Other, "Couldn't read STL header"));
            }
        }
        Err(e) => return Err(e),
    };

    let num_triangles = input.read_u32::<LittleEndian>()?;

    Ok(BinaryStlHeader {
        header,
        num_triangles,
    })
}

pub fn read_stl<T: ReadBytesExt>(input: &mut T) -> Result<BinaryStlFile> {
    // read the header
    let header = read_header(input)?;

    let mut triangles = Vec::new();
    for _ in 0..header.num_triangles {
        triangles.push(read_triangle(input)?);
    }

    Ok(BinaryStlFile { header, triangles })
}

fn write_point<T: WriteBytesExt>(out: &mut T, p: [f32; 3]) -> Result<()> {
    for x in &p {
        out.write_f32::<LittleEndian>(*x)?;
    }
    Ok(())
}

pub fn write_stl<T: WriteBytesExt>(out: &mut T, stl: &BinaryStlFile) -> Result<()> {
    assert_eq!(stl.header.num_triangles as usize, stl.triangles.len());

    //write the header.
    out.write_all(&stl.header.header)?;
    out.write_u32::<LittleEndian>(stl.header.num_triangles)?;

    // write all the triangles
    for t in &stl.triangles {
        write_point(out, t.normal.position)?;
        write_point(out, t.v1.position)?;
        write_point(out, t.v2.position)?;
        write_point(out, t.v3.position)?;
        out.write_u16::<LittleEndian>(t.attr_byte_count)?;
    }

    Ok(())
}

pub struct Tool {
    pub points: Vec<Point3d>,
    pub circle: Circle
}

impl Tool {
    pub fn new_endmill(radius: f32) -> Tool {
        let circle = Circle::new(Point3d::new(radius, radius, 0.0), radius);
        let points: Vec<Point3d> = (0..(radius * 20.0) as i32)
            .flat_map(|x| {
                (0..(radius * 20.0) as i32)
                    .map(move |y| Point3d::new(x as f32 / 10.0, y as f32 / 10.0, 0.0))
            })
            .filter(|x| circle.in_2d_bounds(x))
            .collect();
        Tool { points, circle }
    }
}

fn main() {
    /*
    // As with other examples, the first step is to create an instance.
    let instance = Instance::new(None, &InstanceExtensions::none(), None).unwrap();

    // Choose which physical device to use.
    let physical = PhysicalDevice::enumerate(&instance).next().unwrap();

    // Choose the queue of the physical device which is going to run our compute operation.
    //
    // The Vulkan specs guarantee that a compliant implementation must provide at least one queue
    // that supports compute operations.
    let queue_family = physical
        .queue_families()
        .find(|&q| q.supports_compute())
        .unwrap();

    // Now initializing the device.
    let (device, mut queues) = Device::new(
        physical,
        physical.supported_features(),
        &DeviceExtensions {
            khr_storage_buffer_storage_class: true,
            ..DeviceExtensions::none()
        },
        [(queue_family, 0.5)].iter().cloned(),
    )
    .unwrap();

    // Since we can request multiple queues, the `queues` variable is in fact an iterator. In this
    // example we use only one queue, so we just retrieve the first and only element of the
    // iterator and throw it away.
    let queue = queues.next().unwrap();

    println!("Device initialized");

    vulkano::impl_vertex!(Point, position);
    */
    let tool = Tool::new_endmill(5.0);
    {
        let mut file = File::create("endmill.xyz").unwrap();
        let output = tool.points
            .iter()
            .map(|x| format!("{:.3} {:.3} {:.3}\n", x.x, x.y, x.z))
            .collect::<Vec<String>>()
            .join("");
        file.write_all(output.as_bytes()).unwrap();
    }

    let file = File::open("vt_flat.stl").unwrap();
    let mut buf_reader = BufReader::new(file);
    let stl = read_stl(&mut buf_reader).unwrap();

    let mut triangles = Vec::new();
    for i in 0..stl.header.num_triangles - 1 {
        let i = i as usize;
        triangles.push(Triangle3d::new(
            (
                stl.triangles[i].v1[0],
                stl.triangles[i].v1[1],
                stl.triangles[i].v1[2],
            ),
            (
                stl.triangles[i].v2[0],
                stl.triangles[i].v2[1],
                stl.triangles[i].v2[2],
            ),
            (
                stl.triangles[i].v3[0],
                stl.triangles[i].v3[1],
                stl.triangles[i].v3[2],
            ),
        ));
    }
    //let bboxs: Vec<_> = triangles.par_iter().map(|x| x.bbox()).collect();
    let max_x = (triangles
        .iter()
        .map(|x| x.bbox().p2.x)
        .fold(f32::NAN, f32::max)
        * 10.0) as i32;
    let min_x = (triangles
        .iter()
        .map(|x| x.bbox().p1.x)
        .fold(f32::NAN, f32::min)
        * 10.0) as i32;
    let max_y = (triangles
        .iter()
        .map(|x| x.bbox().p2.y)
        .fold(f32::NAN, f32::max)
        * 10.0) as i32;
    let min_y = (triangles
        .iter()
        .map(|x| x.bbox().p1.y)
        .fold(f32::NAN, f32::min)
        * 10.0) as i32;
    let tests: Vec<Vec<_>> = (min_x..max_x)
        .map(|x| {
            (min_y..max_y)
                .map(move |y| Line3d {
                    p1: Point3d::new(x as f32 / 10.0, y as f32 / 10.0, 200.0),
                    p2: Point3d::new(0.0, 0.0, -1.0),
                })
                .collect()
        })
        .collect();
    //println!("{:?} {:?}", triangles[0], bboxs[0]);
    //let intersects: Vec<&Triangle3d> = triangles.par_iter().filter(|x| x.bbox().in_2d_bounds(&test)).collect();
    //println!("{:?} {:?}\n{:?}", triangles.len(), intersects.len(), intersects);
    //let line = Line3d{p1: test, p2: Point3d::new(0.0, 0.0, -1.0)};
    //let points: Vec<Point3d> = triangles.par_iter().filter_map(|x| x.intersect(line)).collect();
    let mut result = Vec::new();
    for line in tests {
        let bounds = Line3d {
            p1: Point3d::new(line[0].p1.x - 5.0, min_y as f32 / 5.0, 0.0),
            p2: Point3d::new(line[0].p1.x + 5.0, max_y as f32 / 5.0, 0.0),
        };
        let intersects: Vec<&Triangle3d> = triangles
            .par_iter()
            .filter(|x| x.in_2d_bounds(&bounds))
            .collect();
        for point in &line {
            let bbox = Line3d{p1: tool.circle.bbox().p1 + point.p1 - tool.circle.center, p2: tool.circle.bbox().p2 + point.p1 - tool.circle.center};
            let filtered: Vec<_> = intersects.par_iter().filter(|x| x.in_2d_bounds(&bbox)).collect();
            let points: Vec<Point3d> = filtered
                .par_iter()
                .filter_map(|x| x.intersect(*point))
                .collect();
            let max = points.iter().map(|x| x.z).fold(f32::NAN, f32::max);
            if !max.is_nan() {
                result.push(Point3d {
                    x: point.p1.x,
                    y: point.p1.y,
                    z: max,
                });
            }
        }
    }
    let mut file = File::create("pcl.xyz").unwrap();
    let output = result
        .iter()
        .map(|x| format!("{:.3} {:.3} {:.3}\n", x.x, x.y, x.z))
        .collect::<Vec<String>>()
        .join("");
    file.write_all(output.as_bytes()).unwrap();

        /*
    // Now let's get to the actual example.
    //
    // What we are going to do is very basic: we are going to fill a buffer with 64k integers
    // and ask the GPU to multiply each of them by 12.
    //
    // GPUs are very good at parallel computations (SIMD-like operations), and thus will do this
    // much more quickly than a CPU would do. While a CPU would typically multiply them one by one
    // or four by four, a GPU will do it by groups of 32 or 64.
    //
    // Note however that in a real-life situation for such a simple operation the cost of
    // accessing memory usually outweighs the benefits of a faster calculation. Since both the CPU
    // and the GPU will need to access data, there is no other choice but to transfer the data
    // through the slow PCI express bus.

    // We need to create the compute pipeline that describes our operation.
    //
    // If you are familiar with graphics pipeline, the principle is the same except that compute
    // pipelines are much simpler to create.
    let pipeline = Arc::new({
        mod cs {
            vulkano_shaders::shader! {
                ty: "compute",
                src: "
                    #version 450

                    layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

                    layout(set = 0, binding = 0) buffer Data {
                        uint data[];
                    } data;

                    void main() {
                        uint idx = gl_GlobalInvocationID.x;
                        data.data[idx] *= 12;
                    }
                "
            }
        }
        let shader = cs::Shader::load(device.clone()).unwrap();
        ComputePipeline::new(device.clone(), &shader.main_entry_point(), &()).unwrap()
    });

    // We start by creating the buffer that will store the data.
    let data_buffer = {
        // Iterator that produces the data.
        let data_iter = (0..65536u32).map(|n| n);
        // Builds the buffer and fills it with this iterator.
        CpuAccessibleBuffer::from_iter(device.clone(), BufferUsage::all(), false, data_iter)
            .unwrap()
    };

    // In order to let the shader access the buffer, we need to build a *descriptor set* that
    // contains the buffer.
    //
    // The resources that we bind to the descriptor set must match the resources expected by the
    // pipeline which we pass as the first parameter.
    //
    // If you want to run the pipeline on multiple different buffers, you need to create multiple
    // descriptor sets that each contain the buffer you want to run the shader on.
    let layout = pipeline.layout().descriptor_set_layout(0).unwrap();
    let set = Arc::new(
        PersistentDescriptorSet::start(layout.clone())
            .add_buffer(data_buffer.clone())
            .unwrap()
            .build()
            .unwrap(),
    );

    // In order to execute our operation, we have to build a command buffer.
    let mut builder =
        AutoCommandBufferBuilder::primary_one_time_submit(device.clone(), queue.family()).unwrap();
    builder
        // The command buffer only does one thing: execute the compute pipeline.
        // This is called a *dispatch* operation.
        //
        // Note that we clone the pipeline and the set. Since they are both wrapped around an
        // `Arc`, this only clones the `Arc` and not the whole pipeline or set (which aren't
        // cloneable anyway). In this example we would avoid cloning them since this is the last
        // time we use them, but in a real code you would probably need to clone them.
        .dispatch([1024, 1, 1], pipeline.clone(), set.clone(), ())
        .unwrap();
    // Finish building the command buffer by calling `build`.
    let command_buffer = builder.build().unwrap();

    // Let's execute this command buffer now.
    // To do so, we TODO: this is a bit clumsy, probably needs a shortcut
    let future = sync::now(device.clone())
        .then_execute(queue.clone(), command_buffer)
        .unwrap()
        // This line instructs the GPU to signal a *fence* once the command buffer has finished
        // execution. A fence is a Vulkan object that allows the CPU to know when the GPU has
        // reached a certain point.
        // We need to signal a fence here because below we want to block the CPU until the GPU has
        // reached that point in the execution.
        .then_signal_fence_and_flush()
        .unwrap();

    // Blocks execution until the GPU has finished the operation. This method only exists on the
    // future that corresponds to a signalled fence. In other words, this method wouldn't be
    // available if we didn't call `.then_signal_fence_and_flush()` earlier.
    // The `None` parameter is an optional timeout.
    //
    // Note however that dropping the `future` variable (with `drop(future)` for example) would
    // block execution as well, and this would be the case even if we didn't call
    // `.then_signal_fence_and_flush()`.
    // Therefore the actual point of calling `.then_signal_fence_and_flush()` and `.wait()` is to
    // make things more explicit. In the future, if the Rust language gets linear types vulkano may
    // get modified so that only fence-signalled futures can get destroyed like this.
    future.wait(None).unwrap();

    // Now that the GPU is done, the content of the buffer should have been modified. Let's
    // check it out.
    // The call to `read()` would return an error if the buffer was still in use by the GPU.
    let data_buffer_content = data_buffer.read().unwrap();
    for n in 0..65536u32 {
        assert_eq!(data_buffer_content[n as usize], n * 12);
    }
    */
}
