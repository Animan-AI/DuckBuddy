//! Stationary-noise suppression for the mic stream: an STFT gain per bin against a noise
//! spectrum learned while nobody is talking.
//!
//! The rev-a head carries a cooling fan whose noise is mostly broadband airflow (a 3.9 kHz
//! blade tone holds only a quarter to a third of its power), so a notch does little — on
//! the robot it bought 1-2 dB. Subtracting the fan's steady spectrum instead took the
//! speech-to-noise ratio of a voice at 1 m from 17 to 25 dB with the speech itself
//! unchanged (2026-10-08, both mics away from the fan). Anything that is not steady —
//! speech, a clap, a head scratch — sits above the learned floor and passes.
//!
//! 16 kHz mono in, the same out, 32 ms frames at 50 % overlap: the output lags the input by
//! one frame.

use std::sync::Arc;

use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// Frame length, samples (32 ms at 16 kHz).
const N: usize = 512;
const HOP: usize = N / 2;
const BINS: usize = N / 2 + 1;
/// Over-subtraction: how many times the noise estimate comes off each bin.
const OVER: f32 = 1.5;
/// Gain floor (-20 dB): full subtraction leaves "musical noise" — isolated bins blinking on
/// and off — which is worse to listen to than a little steady hiss.
const FLOOR: f32 = 0.1;
/// A bin's gain may fall to no less than this fraction of its last value per frame, so
/// gains release over a few frames instead of snapping shut between syllables.
const RELEASE: f32 = 0.5;
/// Frames spent learning the noise outright before any gating (~0.3 s).
const LEARN_FRAMES: u32 = 20;
/// A frame counts as noise only below this multiple of the learned noise power...
const GATE: f32 = 3.0;
/// ...and then moves the estimate by this much (a ~0.8 s time constant).
const TRACK: f32 = 0.02;

pub struct Denoiser {
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    /// √Hann (periodic), used for both analysis and synthesis: its square sums to exactly
    /// one at 50 % overlap, so overlap-add needs no normalisation.
    window: [f32; N],
    /// The last `N` input samples.
    frame: [f32; N],
    /// New samples since the last frame was processed.
    fresh: usize,
    /// Overlap-add accumulator; its first `HOP` samples are complete after each frame.
    overlap: [f32; N],
    noise: [f32; BINS],
    gain: [f32; BINS],
    frames: u32,
    spectrum: Vec<Complex32>,
}

impl Default for Denoiser {
    fn default() -> Self {
        Self::new()
    }
}

impl Denoiser {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        let mut window = [0.0; N];
        for (n, w) in window.iter_mut().enumerate() {
            let hann = 0.5 - 0.5 * (2.0 * std::f32::consts::PI * n as f32 / N as f32).cos();
            *w = hann.sqrt();
        }
        Self {
            fft: planner.plan_fft_forward(N),
            ifft: planner.plan_fft_inverse(N),
            window,
            frame: [0.0; N],
            fresh: 0,
            overlap: [0.0; N],
            noise: [0.0; BINS],
            gain: [1.0; BINS],
            frames: 0,
            spectrum: vec![Complex32::new(0.0, 0.0); N],
        }
    }

    /// Denoise a chunk of any length. Returns exactly as many samples as it was given; the
    /// stream is the input delayed by one frame.
    pub fn process(&mut self, input: &[f32]) -> Vec<f32> {
        let mut out = Vec::with_capacity(input.len());
        for &sample in input {
            self.frame.copy_within(1.., 0);
            self.frame[N - 1] = sample;
            self.fresh += 1;
            // The sample leaving the delay line is the oldest complete one.
            out.push(self.overlap[self.fresh - 1]);
            if self.fresh == HOP {
                self.fresh = 0;
                self.overlap.copy_within(HOP.., 0);
                self.overlap[N - HOP..].fill(0.0);
                self.step();
            }
        }
        out
    }

    fn step(&mut self) {
        for (bin, (&x, &w)) in self
            .spectrum
            .iter_mut()
            .zip(self.frame.iter().zip(&self.window))
        {
            *bin = Complex32::new(x * w, 0.0);
        }
        self.fft.process(&mut self.spectrum);

        let mut power = [0.0f32; BINS];
        for (p, bin) in power.iter_mut().zip(&self.spectrum) {
            *p = bin.norm_sqr();
        }
        if self.frames < LEARN_FRAMES {
            let keep = if self.frames == 0 { 0.0 } else { 0.9 };
            for (n, p) in self.noise.iter_mut().zip(&power) {
                *n = keep * *n + (1.0 - keep) * p;
            }
            self.frames += 1;
        } else if power.iter().sum::<f32>() < GATE * self.noise.iter().sum::<f32>() {
            for (n, p) in self.noise.iter_mut().zip(&power) {
                *n = (1.0 - TRACK) * *n + TRACK * p;
            }
        }

        for ((gain, &noise), &p) in self.gain.iter_mut().zip(&self.noise).zip(&power) {
            let g = (1.0 - OVER * noise / p.max(1e-20)).max(FLOOR);
            *gain = g.max(RELEASE * *gain);
        }
        // Bins 0..=N/2, then their mirror images N-1..=N/2+1 (bins 1..N/2), which a real
        // signal's spectrum must keep conjugate-symmetric.
        for (bin, &g) in self.spectrum[..BINS].iter_mut().zip(&self.gain) {
            *bin *= g;
        }
        for (bin, &g) in self.spectrum[BINS..].iter_mut().rev().zip(&self.gain[1..]) {
            *bin *= g;
        }
        self.ifft.process(&mut self.spectrum);
        let scale = 1.0 / N as f32;
        for ((acc, bin), &w) in self
            .overlap
            .iter_mut()
            .zip(&self.spectrum)
            .zip(&self.window)
        {
            *acc += bin.re * scale * w;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f32 = 16_000.0;

    /// Deterministic white-ish noise (xorshift), uniform in ±amp.
    fn noise(len: usize, amp: f32, mut state: u32) -> Vec<f32> {
        (0..len)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                (state as f32 / u32::MAX as f32 * 2.0 - 1.0) * amp
            })
            .collect()
    }

    fn tone(len: usize, hz: f32, amp: f32) -> Vec<f32> {
        (0..len)
            .map(|n| amp * (2.0 * std::f32::consts::PI * hz * n as f32 / SR).sin())
            .collect()
    }

    fn power(x: &[f32]) -> f32 {
        x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
    }

    fn db(ratio: f32) -> f32 {
        10.0 * ratio.log10()
    }

    /// The fan: steady broadband hiss plus a blade tone. After a second of learning it
    /// should come down by well over the 6 dB that makes it worth having.
    #[test]
    fn steady_fan_noise_comes_down() {
        let len = 4 * 16_000;
        let fan: Vec<f32> = noise(len, 0.01, 7)
            .iter()
            .zip(tone(len, 3932.0, 0.005))
            .map(|(a, b)| a + b)
            .collect();
        let out = Denoiser::new().process(&fan);
        let settled = 16_000..len;
        let drop = db(power(&out[settled.clone()]) / power(&fan[settled]));
        assert!(drop < -8.0, "fan noise only came down {drop:.1} dB");
    }

    /// A voice over the fan — here a loud burst of harmonics — must come through nearly
    /// whole: the point is to hear people better, not to quieten them.
    #[test]
    fn speech_over_the_fan_passes() {
        let len = 4 * 16_000;
        let fan = noise(len, 0.01, 11);
        let mut voice = vec![0.0; len];
        let burst = 2 * 16_000..3 * 16_000;
        for f in [220.0, 440.0, 660.0, 880.0, 1320.0] {
            for (v, t) in voice[burst.clone()].iter_mut().zip(tone(16_000, f, 0.02)) {
                *v += t;
            }
        }
        let mixed: Vec<f32> = fan.iter().zip(&voice).map(|(a, b)| a + b).collect();
        let out = Denoiser::new().process(&mixed);
        // One frame of delay, and half a frame each side for the window to fill.
        let inner = burst.start + N..burst.end;
        let kept = db(power(&out[inner.clone()]) / power(&voice[inner.start - N..inner.end - N]));
        assert!(kept > -1.5, "speech lost {kept:.1} dB");
    }

    /// Chunking must not change the stream: arecord hands over whatever it has.
    #[test]
    fn chunk_boundaries_do_not_matter() {
        let x = noise(10_000, 0.1, 3);
        let whole = Denoiser::new().process(&x);
        let mut d = Denoiser::new();
        let mut pieces = Vec::new();
        for chunk in x.chunks(37) {
            pieces.extend(d.process(chunk));
        }
        assert_eq!(whole.len(), x.len());
        assert_eq!(whole, pieces);
    }
}
