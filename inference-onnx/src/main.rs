use std::time::Duration;

use clap::Parser;
use ndarray::Array;
use ort::{
    session::{Session, SessionOutputs, builder::GraphOptimizationLevel},
    value::Tensor,
};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    model: String,

    #[arg(short, long)]
    port: String,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }

    let mut port = serialport::new(&args.port, 500000)
        .timeout(Duration::from_millis(100))
        .open()
        .expect("Failed to open port");

    ort::init().commit()?;

    let mut model = Session::builder()?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(4)?
        .commit_from_file(args.model)?;

    let mut input: Array<f32, _> = Array::zeros((1, 1));

    let mut read_buf = [0; 10];
    loop {
        if port.read_exact(&mut read_buf).is_err() {
            continue;
        }

        if read_buf[0] != 0x90 || read_buf[1] != 0x80 {
            println!("Invalid message received: {:?}", read_buf);

            loop {
                let mut b = [0x00];
                if port.read_exact(&mut b).is_err() {
                    continue;
                }
                if b[0] == 0x90 {
                    let mut clear = [0x00; 9];
                    if port.read_exact(&mut clear).is_err() {
                        continue;
                    }
                    break;
                }
            }

            continue;
        }

        let x = f32::from_ne_bytes(read_buf[2..6].try_into()?);
        let theta = f32::from_ne_bytes(read_buf[6..10].try_into()?);

        // println!("Received value: {}", value);

        let model_value = -(theta.to_degrees() / 20.0);

        input[[0, 0]] = model_value;

        let outputs: SessionOutputs = model.run(ort::inputs! {
            "obs" => Tensor::from_array(input.clone())?
        })?;

        let output = outputs["action_mu"].try_extract_tensor::<f32>()?.1;

        let speed = (output[0] + output[1]) / 2.0 * 3.0;

        let p = [0x90, 0x80];
        let msg = p
            .iter()
            .copied()
            .chain(speed.to_ne_bytes().iter().copied())
            .collect::<Vec<u8>>();
        let _ = port.write(&msg);

        println!("Input: {:?}, Output: {:?}", input, output);

        // std::thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}
