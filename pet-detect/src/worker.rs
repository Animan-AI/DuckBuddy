//! The mic, as a background worker: `arecord` → petting classifier + ambient sound sentry.
//!
//! The audio source is an `arecord` subprocess rather than an in-process ALSA binding — the
//! same pattern the standalone `pet-detect` binary documents, and no new native dependency.
//! The capture device is single-client, so everything that analyses the mic shares this one
//! stream.
//!
//! Ported from the prototype's `pet_worker.rs`; `println!` diagnostics became `tracing`.

use std::io::Read;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::Result;

use crate::denoise::Denoiser;
use crate::{PettingDetector, PettingDetectorConfig, PettingEvent, i16_to_f32};

/// Ambient sound events, from the same stream the petting classifier consumes. Pure
/// RMS-envelope heuristics — no ML:
///   * `Noise`: a sharp transient (clap, bang, door) — ≤ ~0.38 s of loud.
///   * `Voice`: a sustained utterance (speech, a quack at the duck) — up to ~3 s of loud.
///     Longer runs are continuous noise (vacuum, music) and emit nothing; the adaptive
///     floor absorbs them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SoundEvent {
    Noise,
    Voice,
}

/// 32 ms at 16 kHz.
const SENTRY_FRAME: u32 = 512;

/// RMS envelope watcher over ~32 ms frames with a slowly-adapting ambient floor. Petting
/// sounds are loud on this mic (it's practically a contact mic for head scratches), so
/// events are suppressed while the classifier reports petting (+1 s hangover).
struct SoundSentry {
    floor: f32,
    frame_acc: f32,
    frame_n: u32,
    in_event: bool,
    event_frames: u32,
    quiet_frames: u32,
    cooldown_frames: u32,
    petting_hold_frames: u32,
    /// Loudest frame RMS of the current event, for the tuning log.
    event_peak: f32,
    /// Frames since the last periodic floor log (~every 60 s).
    floor_log_frames: u32,
}

impl SoundSentry {
    fn new() -> Self {
        Self {
            floor: 0.003,
            frame_acc: 0.0,
            frame_n: 0,
            in_event: false,
            event_frames: 0,
            quiet_frames: 0,
            cooldown_frames: 0,
            petting_hold_frames: 0,
            event_peak: 0.0,
            floor_log_frames: 0,
        }
    }

    fn push(&mut self, samples: &[f32], petting: bool, out: &mut Vec<SoundEvent>) {
        for &s in samples {
            self.frame_acc += s * s;
            self.frame_n += 1;
            if self.frame_n >= SENTRY_FRAME {
                let rms = (self.frame_acc / self.frame_n as f32).sqrt();
                self.frame_acc = 0.0;
                self.frame_n = 0;
                self.frame(rms, petting, out);
            }
        }
    }

    fn frame(&mut self, rms: f32, petting: bool, out: &mut Vec<SoundEvent>) {
        if petting {
            self.petting_hold_frames = 31; // ~1 s hangover after petting
        } else if self.petting_hold_frames > 0 {
            self.petting_hold_frames -= 1;
        }
        if self.cooldown_frames > 0 {
            self.cooldown_frames -= 1;
        }
        // Periodic ambient-floor log (~every 60 s at 31 frames/s) so threshold tuning on
        // the robot is data-driven.
        self.floor_log_frames += 1;
        if self.floor_log_frames >= 1875 {
            self.floor_log_frames = 0;
            tracing::debug!(floor = self.floor, "ambient floor");
        }
        // Absolute minimums exist only so 6× a near-zero floor doesn't trigger in silence —
        // the ratio thresholds do the real work. (Samples are ±1-normalised; a low capture
        // gain puts real claps well under an absolute guess.)
        let on_thresh = (self.floor * 6.0).max(0.002);
        let off_thresh = (self.floor * 3.0).max(0.0012);
        if !self.in_event {
            // The ambient floor adapts only from non-event frames (τ ≈ 6 s), so sustained
            // noise (gait servos, music) raises the bar instead of spamming events.
            self.floor = 0.995 * self.floor + 0.005 * rms;
            if rms > on_thresh && self.cooldown_frames == 0 && self.petting_hold_frames == 0 {
                self.in_event = true;
                self.event_frames = 1;
                self.quiet_frames = 0;
                self.event_peak = rms;
            }
        } else {
            self.event_frames += 1;
            self.event_peak = self.event_peak.max(rms);
            if rms < off_thresh {
                self.quiet_frames += 1;
            } else {
                self.quiet_frames = 0;
            }
            if self.event_frames > 94 {
                // > ~3 s: continuous noise — absorb into the floor, no event.
                self.in_event = false;
                self.floor = self.floor.max(rms * 0.5);
                self.cooldown_frames = 31;
            } else if self.quiet_frames >= 3 {
                self.in_event = false;
                self.cooldown_frames = 31; // ≥ 1 s between events
                if self.petting_hold_frames == 0 {
                    let dur = self.event_frames - self.quiet_frames;
                    // ≤ ~0.38 s = transient (clap + its reverb tail); longer = an utterance.
                    let ev = if dur <= 12 {
                        SoundEvent::Noise
                    } else {
                        SoundEvent::Voice
                    };
                    tracing::debug!(
                        event = ?ev,
                        dur_s = dur as f32 * SENTRY_FRAME as f32 / 16000.0,
                        peak = self.event_peak,
                        floor = self.floor,
                        "ambient sound"
                    );
                    out.push(ev);
                }
            }
        }
    }
}

pub struct PetConfig {
    /// ALSA capture device passed to arecord.
    pub alsa_device: String,
    /// Path to the trained ONNX model.
    pub model_path: PathBuf,
    /// Probability above which petting starts.
    pub enter_threshold: f32,
    /// Probability below which petting ends.
    pub exit_threshold: f32,
    /// Suppress steady background noise (the head fan) before anything listens — see
    /// [`crate::denoise`].
    pub denoise: bool,
}

impl Default for PetConfig {
    fn default() -> Self {
        let lib = PettingDetectorConfig::default();
        Self {
            alsa_device: "plughw:aic3104,0".into(),
            model_path: PathBuf::from("/opt/robot/daemon/current/models/pet_detect.onnx"),
            enter_threshold: lib.enter_threshold,
            exit_threshold: lib.exit_threshold,
            denoise: true,
        }
    }
}

/// The worker's handle: drain events per tick, shut down on drop of the process.
pub struct PetHandle {
    rx: Receiver<PettingEvent>,
    rx_sound: Receiver<SoundEvent>,
    shutdown: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl PetHandle {
    pub fn spawn(config: PetConfig) -> Result<Self> {
        // Built here, not in the thread, so a missing model or runtime is an error the
        // caller sees instead of a worker that dies quietly on its first breath. Behind a
        // panic catch, because `ort` panics on failures it considers unrecoverable — a
        // missing libonnxruntime must read as "no mic worker", not a dead daemon.
        let detector = std::panic::catch_unwind(|| {
            PettingDetector::new(
                &config.model_path,
                PettingDetectorConfig {
                    stride: PettingDetectorConfig::default().stride,
                    enter_threshold: config.enter_threshold,
                    exit_threshold: config.exit_threshold,
                },
            )
        })
        .unwrap_or_else(|_| Err(anyhow::anyhow!("the ONNX runtime is not loadable")))?;

        let shutdown = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::channel();
        let (tx_sound, rx_sound) = mpsc::channel();
        let sd = shutdown.clone();
        let dev = config.alsa_device.clone();
        let denoiser = config.denoise.then(Denoiser::new);

        let join = thread::Builder::new()
            .name("pet-worker".into())
            .spawn(move || worker_loop(&dev, detector, denoiser, &tx, &tx_sound, &sd))
            .expect("spawn pet worker");

        Ok(Self {
            rx,
            rx_sound,
            shutdown,
            join: Some(join),
        })
    }

    /// Non-blocking; the next petting event if one is queued. Once per control tick.
    pub fn try_recv_event(&self) -> Option<PettingEvent> {
        self.rx.try_recv().ok()
    }

    /// Non-blocking; the next ambient sound event if one is queued.
    pub fn try_recv_sound(&self) -> Option<SoundEvent> {
        self.rx_sound.try_recv().ok()
    }

    pub fn shutdown(mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// Backoff between restarts of a capture that will not stay up: doubling from 250 ms to a
/// cap. A board where `arecord` exists but the codec does not — `configure_audio` fails soft
/// at every step, so a failed DKMS build leaves exactly that — makes `arecord` exit
/// immediately on every spawn. Without a backoff on *that* path (the original only slept
/// when the `arecord` binary itself was missing) the worker fork/execs as fast as the CPU
/// allows for the life of the daemon, with a `warn!` per iteration into the journal.
const RESTART_BACKOFF_MIN: Duration = Duration::from_millis(250);
const RESTART_BACKOFF_MAX: Duration = Duration::from_secs(30);

/// A capture that ran this long is not the failure this backoff is for; the delay resets.
const RESTART_HEALTHY: Duration = Duration::from_secs(5);

/// After this many consecutive immediate failures the per-restart line drops to `debug`.
/// The backoff is at its cap by then and the warning has been said — the worker keeps
/// trying (a codec that comes back should be heard) but stops narrating it.
const RESTART_QUIET_AFTER: u32 = 5;

/// Consecutive exactly-zero samples that mean the capture has gone dead (0.5 s at 16 kHz).
///
/// On the A733 the codec's capture stream turns to digital silence the moment a playback
/// stream on the same card closes, and stays that way until the capture is reopened
/// (measured 2026-10-08: any `aplay` during an `arecord` — 16 or 48 kHz alike — and every
/// sample after it is 0). The vendor I2S driver evidently stops the clock the two share;
/// `arecord` keeps reading frames and reports nothing. A live microphone never produces
/// half a second of exact zeros — its own noise alone rules that out — so this is the
/// signal to reopen.
const SILENT_SAMPLES: u32 = 8_000;

/// How one `arecord` run ended without an error.
enum Ended {
    Shutdown,
    /// The capture went dead after delivering sound: reopen it straight away. A run that
    /// never delivered anything but zeros is an ordinary failure instead, so a muted or
    /// dead mic backs off rather than restarting twice a second forever.
    WentSilent,
}

fn worker_loop(
    alsa_device: &str,
    mut detector: PettingDetector,
    // Owned here, like the detector, so its learned noise floor survives an arecord restart.
    mut denoiser: Option<Denoiser>,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
    shutdown: &Arc<AtomicBool>,
) {
    let mut sentry = SoundSentry::new();
    let mut failures: u32 = 0;
    while !shutdown.load(Ordering::Acquire) {
        let started = Instant::now();
        match spawn_arecord(alsa_device) {
            Ok(mut c) => {
                let ended = pump(
                    &mut c,
                    &mut detector,
                    denoiser.as_mut(),
                    &mut sentry,
                    tx,
                    tx_sound,
                    shutdown,
                );
                let _ = c.kill();
                let _ = c.wait();
                match ended {
                    Ok(Ended::WentSilent) => {
                        tracing::debug!(
                            "pet worker: capture went silent after playback; reopening"
                        );
                        failures = 0;
                        continue;
                    }
                    Ok(Ended::Shutdown) => {}
                    Err(e) => log_restart(failures, &e.to_string()),
                }
            }
            Err(e) => log_restart(failures, &e.to_string()),
        }
        if started.elapsed() >= RESTART_HEALTHY {
            failures = 0;
            continue;
        }
        failures = failures.saturating_add(1);
        let backoff = RESTART_BACKOFF_MIN
            .saturating_mul(1u32 << (failures - 1).min(8))
            .min(RESTART_BACKOFF_MAX);
        if failures == RESTART_QUIET_AFTER {
            tracing::warn!(
                device = alsa_device,
                ?backoff,
                "pet worker: the mic will not stay up — retrying quietly from here"
            );
        }
        // Sliced, because `shutdown()` joins this thread: a 30 s sleep would be 30 s of
        // robotd not exiting.
        if !sleep_unless_shutdown(backoff, shutdown) {
            return;
        }
    }
}

fn log_restart(failures: u32, error: &str) {
    if failures < RESTART_QUIET_AFTER {
        tracing::warn!(error, "pet worker: restarting arecord");
    } else {
        tracing::debug!(error, "pet worker: restarting arecord");
    }
}

/// Sleep, waking early if shutdown is asked for. `false` means "stop".
fn sleep_unless_shutdown(total: Duration, shutdown: &Arc<AtomicBool>) -> bool {
    const SLICE: Duration = Duration::from_millis(100);
    let deadline = Instant::now() + total;
    while Instant::now() < deadline {
        if shutdown.load(Ordering::Acquire) {
            return false;
        }
        thread::sleep(SLICE.min(deadline.saturating_duration_since(Instant::now())));
    }
    !shutdown.load(Ordering::Acquire)
}

fn spawn_arecord(device: &str) -> Result<Child> {
    Ok(Command::new("arecord")
        .args([
            "-D", device, "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw",
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()?)
}

fn pump(
    child: &mut Child,
    detector: &mut PettingDetector,
    mut denoiser: Option<&mut Denoiser>,
    sentry: &mut SoundSentry,
    tx: &Sender<PettingEvent>,
    tx_sound: &Sender<SoundEvent>,
    shutdown: &Arc<AtomicBool>,
) -> Result<Ended> {
    let mut stdout = child
        .stdout
        .take()
        .ok_or_else(|| anyhow::anyhow!("arecord stdout missing"))?;
    let mut buf = [0u8; 4096];
    let mut i16_buf: Vec<i16> = Vec::with_capacity(2048);
    let mut silence = SilenceWatch::default();

    while !shutdown.load(Ordering::Acquire) {
        let n = stdout.read(&mut buf)?;
        if n == 0 {
            return Err(anyhow::anyhow!("arecord stdout closed (EOF)"));
        }
        if !n.is_multiple_of(2) {
            return Err(anyhow::anyhow!("odd byte count from arecord"));
        }
        i16_buf.clear();
        for chunk in buf[..n].as_chunks::<2>().0 {
            i16_buf.push(i16::from_le_bytes(*chunk));
        }
        match silence.observe(&i16_buf) {
            Silence::Live => {}
            Silence::WentDead => return Ok(Ended::WentSilent),
            Silence::NeverLive => {
                return Err(anyhow::anyhow!(
                    "capture delivers only zeros (mic muted or dead?)"
                ));
            }
        }
        let mut samples = i16_to_f32(&i16_buf);
        if let Some(d) = denoiser.as_deref_mut() {
            samples = d.process(&samples);
        }
        let (events, _p) = detector.push_samples(&samples)?;
        for ev in events {
            if tx.send(ev).is_err() {
                // Receiver dropped — the daemon is shutting down.
                return Ok(Ended::Shutdown);
            }
        }
        // Ambient sound events off the same stream, muted while petting (head scratches
        // are LOUD on this mic) + a short hangover. Read off the detector rather than
        // tracked here: a Start..End session spans many seconds, and `arecord` can flap
        // inside one. The detector survives a restart (it is owned by `worker_loop`), so a
        // local copy would come back `false` mid-session — and no second `Start` would ever
        // re-arm it, leaving the sentry emitting head-scratch noise as `Voice` events until
        // the probability finally fell below the exit threshold.
        let mut sounds = Vec::new();
        sentry.push(&samples, detector.is_petting(), &mut sounds);
        for sev in sounds {
            let _ = tx_sound.send(sev);
        }
    }
    Ok(Ended::Shutdown)
}

#[derive(Debug, PartialEq)]
enum Silence {
    Live,
    /// Zeros for [`SILENT_SAMPLES`] after real sound.
    WentDead,
    /// Zeros for [`SILENT_SAMPLES`] from the start.
    NeverLive,
}

/// Counts the run of exactly-zero samples at the end of the stream so far.
#[derive(Default)]
struct SilenceWatch {
    zeros: u32,
    heard: bool,
}

impl SilenceWatch {
    fn observe(&mut self, samples: &[i16]) -> Silence {
        for &s in samples {
            if s == 0 {
                self.zeros = self.zeros.saturating_add(1);
            } else {
                self.zeros = 0;
                self.heard = true;
            }
        }
        match (self.zeros >= SILENT_SAMPLES, self.heard) {
            (false, _) => Silence::Live,
            (true, true) => Silence::WentDead,
            (true, false) => Silence::NeverLive,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mic_that_goes_dead_is_reopened_and_a_dead_one_is_a_failure() {
        let mut w = SilenceWatch::default();
        assert_eq!(w.observe(&[3, -2, 0, 1]), Silence::Live);
        assert_eq!(
            w.observe(&vec![0; SILENT_SAMPLES as usize - 1]),
            Silence::Live
        );
        assert_eq!(w.observe(&[0]), Silence::WentDead);

        let mut never = SilenceWatch::default();
        assert_eq!(
            never.observe(&vec![0; SILENT_SAMPLES as usize]),
            Silence::NeverLive
        );
    }

    /// Quiet is not silence: a real mic's noise floor keeps the run short.
    #[test]
    fn a_quiet_room_is_live() {
        let mut w = SilenceWatch::default();
        let hiss: Vec<i16> = (0..32_000).map(|n| [0, 1, 0, -1][n % 4]).collect();
        assert_eq!(w.observe(&hiss), Silence::Live);
    }
}
