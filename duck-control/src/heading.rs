//! Heading hold: keep a walk that was asked to go straight going straight.
//!
//! The walking policies track a yaw-rate command, not a heading, so a walk asked to go straight
//! drifts by whatever the robot's own asymmetries add up to — on the HL-2915 robot (2026-10-10)
//! anything from −11° to +137° over 8 s, in either direction. While a client asks to translate
//! without turning, [`HeadingHold`] keeps the heading the straight segment started on and asks
//! for the yaw rate that steers back to it: `vyaw = −gain × (error past the deadband)`, clamped,
//! where the error is that of a low-passed heading.
//!
//! The low-pass and the deadband are there because the trunk also yaws with every step — 3.5 to
//! 8.6° standard deviation on the robot's straight walks — and a hold that chases that wobble
//! drives it: on 2026-10-11, at gain 1 with neither, the heading swung ±30° within a second and
//! the robot fell. The hold is for the slow wander underneath (30–150° over 8 s).
//!
//! It never adds a turn to a robot asked to stand still: a yaw rate in the twist counts as
//! motion, and would hand the robot from the standing network to the walking one.

use std::f64::consts::{PI, TAU};

/// Requested translation at or below this (m/s, `hypot(vx, vy)`) is standing.
pub const TRANSLATE_DEADBAND: f64 = 0.01;
/// A requested yaw rate above this (rad/s) is the client steering, and wins.
pub const TURN_DEADBAND: f64 = 0.02;

#[derive(Debug, Clone)]
pub struct HeadingHold {
    gain: f64,
    max_rate: f64,
    /// Per-tick weight of the heading low-pass, `dt / tau` (1 = no filtering).
    alpha: f64,
    deadband: f64,
    reference: Option<f64>,
    /// The low-passed heading, kept unwrapped relative to `reference`'s frame.
    filtered: f64,
}

impl HeadingHold {
    /// `gain` in 1/s (yaw rate per radian of heading error; 0 turns the hold off), `max_rate`
    /// the largest yaw rate it asks for, rad/s; `tau_s` the heading low-pass time constant
    /// (0 = none), `deadband` the heading error it leaves alone, rad; `dt` the tick, s.
    pub fn new(gain: f64, max_rate: f64, tau_s: f64, deadband: f64, dt: f64) -> Self {
        Self {
            gain: gain.max(0.0),
            max_rate: max_rate.abs(),
            alpha: if tau_s > 0.0 {
                (dt / tau_s).clamp(0.0, 1.0)
            } else {
                1.0
            },
            deadband: deadband.abs(),
            reference: None,
            filtered: 0.0,
        }
    }

    pub fn enabled(&self) -> bool {
        self.gain > 0.0
    }

    /// The heading being held, radians, while a straight segment is in progress.
    pub fn reference(&self) -> Option<f64> {
        self.reference
    }

    /// Forget the held heading: the next straight segment starts from wherever the robot then
    /// points. For when something else has the robot (a fall, the stand-up).
    pub fn reset(&mut self) {
        self.reference = None;
    }

    /// The twist to run, given the client's `[vx, vy, vyaw]` and the trunk heading (`None`
    /// while the IMU has not converged, which passes the twist through).
    pub fn apply(&mut self, twist: [f64; 3], yaw: Option<f64>) -> [f64; 3] {
        let translating = twist[0].hypot(twist[1]) > TRANSLATE_DEADBAND;
        let steering = twist[2].abs() > TURN_DEADBAND;
        let yaw = match yaw {
            Some(yaw) if self.enabled() && translating && !steering => yaw,
            _ => {
                self.reference = None;
                return twist;
            }
        };
        let reference = match self.reference {
            Some(r) => r,
            None => {
                self.reference = Some(yaw);
                self.filtered = yaw;
                yaw
            }
        };
        // Step the filter towards the heading the short way round, so it never jumps at ±π.
        self.filtered += self.alpha * wrap(yaw - self.filtered);
        let error = wrap(self.filtered - reference);
        let past = (error.abs() - self.deadband).max(0.0).copysign(error);
        let rate = (-self.gain * past).clamp(-self.max_rate, self.max_rate);
        [twist[0], twist[1], rate]
    }
}

/// An angle in (−π, π].
fn wrap(angle: f64) -> f64 {
    let a = (angle + PI).rem_euclid(TAU) - PI;
    if a == -PI { PI } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WALK: [f64; 3] = [0.2, 0.0, 0.0];
    const DT: f64 = 0.02;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    /// No filter, no deadband: the error as it stands.
    fn plain(gain: f64, max_rate: f64) -> HeadingHold {
        HeadingHold::new(gain, max_rate, 0.0, 0.0, DT)
    }

    #[test]
    fn off_passes_the_twist_through() {
        let mut h = plain(0.0, 0.5);
        assert_eq!(h.apply(WALK, Some(0.0)), WALK);
        assert_eq!(h.apply(WALK, Some(0.3)), WALK);
        assert_eq!(h.reference(), None);
    }

    /// The segment's first tick sets the reference and asks for nothing; drifting left
    /// (yaw up) then asks for a right turn (negative yaw rate), in proportion.
    #[test]
    fn a_drift_left_is_steered_back_right() {
        let mut h = plain(1.0, 0.5);
        assert_eq!(h.apply(WALK, Some(0.1)), [0.2, 0.0, 0.0]);
        assert_eq!(h.reference(), Some(0.1));
        let out = h.apply(WALK, Some(0.3));
        assert!(close(out[2], -0.2), "{out:?}");
        assert!(close(h.apply(WALK, Some(-0.1))[2], 0.2));
    }

    #[test]
    fn the_turn_it_asks_for_is_clamped() {
        let mut h = plain(2.0, 0.5);
        h.apply(WALK, Some(0.0));
        assert!(close(h.apply(WALK, Some(1.0))[2], -0.5));
        assert!(close(h.apply(WALK, Some(-1.0))[2], 0.5));
    }

    /// A robot asked to stand gets no turn, however far it has drifted: a yaw rate would hand
    /// it to the walking network. Walking again starts a new segment from where it points.
    #[test]
    fn standing_never_gets_a_turn() {
        let mut h = plain(1.0, 0.5);
        h.apply(WALK, Some(0.0));
        assert_eq!(h.apply([0.0; 3], Some(0.4)), [0.0; 3]);
        assert_eq!(h.reference(), None);
        assert_eq!(h.apply(WALK, Some(0.4)), WALK);
        assert_eq!(h.reference(), Some(0.4));
    }

    /// The client steering wins, and when they let go the robot keeps its new heading
    /// instead of swinging back to the old one.
    #[test]
    fn a_client_turn_wins_and_moves_the_reference() {
        let mut h = plain(1.0, 0.5);
        h.apply(WALK, Some(0.0));
        let turning = [0.2, 0.0, 0.4];
        assert_eq!(h.apply(turning, Some(0.5)), turning);
        assert_eq!(h.reference(), None);
        assert_eq!(h.apply(WALK, Some(0.9)), WALK);
        assert!(close(h.apply(WALK, Some(0.95))[2], -0.05));
    }

    /// Across ±π the error is the short way round.
    #[test]
    fn the_error_wraps() {
        let mut h = plain(1.0, 0.5);
        h.apply(WALK, Some(3.1));
        let out = h.apply(WALK, Some(-3.1));
        // −3.1 is 0.083 rad past +3.1 going left, so steer right a little.
        assert!(close(out[2], -(TAU - 6.2)), "{out:?}");
    }

    #[test]
    fn no_heading_yet_passes_through() {
        let mut h = plain(1.0, 0.5);
        h.apply(WALK, Some(0.0));
        assert_eq!(h.apply(WALK, None), WALK);
        assert_eq!(h.reference(), None);
    }

    #[test]
    fn sideways_walking_is_held_too() {
        let mut h = plain(1.0, 0.5);
        h.apply([0.0, 0.1, 0.0], Some(0.0));
        assert!(close(h.apply([0.0, 0.1, 0.0], Some(0.1))[2], -0.1));
    }

    /// An error inside the deadband asks for nothing; past it, only the excess counts.
    #[test]
    fn the_deadband_leaves_small_errors_alone() {
        let mut h = HeadingHold::new(1.0, 0.5, 0.0, 0.1, DT);
        h.apply(WALK, Some(0.0));
        assert_eq!(h.apply(WALK, Some(0.08))[2], 0.0);
        assert!(close(h.apply(WALK, Some(0.3))[2], -0.2));
        assert!(close(h.apply(WALK, Some(-0.3))[2], 0.2));
    }

    /// The trunk's per-step wobble (±8° at 2 Hz, as on the robot) barely moves the low-passed
    /// heading, while a steady drift still comes through.
    #[test]
    fn the_filter_ignores_the_step_wobble_and_follows_the_drift() {
        let wobble = |t: f64| 8f64.to_radians() * (TAU * 2.0 * t).sin();
        let mut h = HeadingHold::new(0.5, 0.3, 0.5, 5f64.to_radians(), DT);
        let mut worst = 0.0f64;
        for k in 0..200 {
            let out = h.apply(WALK, Some(wobble(k as f64 * DT)));
            worst = worst.max(out[2].abs());
        }
        assert_eq!(
            worst, 0.0,
            "a ±8° wobble stays inside the deadband once filtered"
        );

        let mut h = HeadingHold::new(0.5, 0.3, 0.5, 5f64.to_radians(), DT);
        let drift = 20f64.to_radians(); // per second
        let mut last = [0.0; 3];
        for k in 0..150 {
            let t = k as f64 * DT;
            last = h.apply(WALK, Some(drift * t + wobble(t)));
        }
        assert!(last[2] < -0.1, "a 20°/s drift is steered against: {last:?}");
    }
}
