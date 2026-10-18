//! Run the mic stream's noise suppression over a WAV, for tuning it against recordings.
//!
//! The input is resampled to 16 k mono exactly as the worker hears it; the output is the
//! denoised 16 k mono, delayed by one 32 ms frame.

use anyhow::Result;
use clap::Parser;
use pet_detect::denoise::Denoiser;
use pet_detect::load_wav_mono_16k;

#[derive(Parser)]
#[command(
    name = "pet-denoise",
    about = "Denoise a WAV as the mic worker would",
    version
)]
struct Args {
    /// A WAV file (any rate / channel count — resampled to 16 k mono).
    input: String,
    /// Where to write the denoised 16 k mono WAV.
    output: String,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let samples = load_wav_mono_16k(&args.input)?;
    let out = Denoiser::new().process(&samples);
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: 16_000,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create(&args.output, spec)?;
    for s in out {
        writer.write_sample((s.clamp(-1.0, 1.0) * 32767.0) as i16)?;
    }
    writer.finalize()?;
    Ok(())
}
