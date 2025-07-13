fn main() -> anyhow::Result<()> {
    ort::init().commit()?;

    Ok(())
}
