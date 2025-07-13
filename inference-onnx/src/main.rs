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
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    ort::init().commit()?;

    let mut model = Session::builder()?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(4)?
        .commit_from_file(args.model)?;

    let mut input: Array<f32, _> = Array::zeros((1, 1));

    input[[0, 0]] = 0.0;

    let outputs: SessionOutputs = model.run(ort::inputs! {
        "obs" => Tensor::from_array(input)?
    })?;

    let output = outputs["action_log_std"]
        .try_extract_tensor::<f32>()
        .unwrap()
        .1;

    println!("Output: {:?}", output);

    Ok(())
}
