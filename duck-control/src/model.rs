//! The robot, as data.
//!
//! One variant — **alpha** — because that is the only robot that exists. Every shipped
//! policy is `alpha_*`; v1/v1.5/v1.6 are history. A second revision becomes a second set
//! of tables, which is honest until there is a second robot to generalise from.
//!
//! The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they
//! were measured against hardware rather than derived. Re-deriving them from a datasheet
//! is exactly the kind of change that looks right and walks wrong.

/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const NUM_JOINTS: usize = 15;

/// Dynamixel IDs, indexed as [`JOINT_NAMES`].
pub const JOINT_IDS: [u8; NUM_JOINTS] = [
    20, 21, 22, 23, 24, // left leg
    30, 31, 32, 33, 34, // neck, head, mouth
    10, 11, 12, 13, 14, // right leg
];

/// Joint names, from the protocol crate — the wire indexes `joints` and `targets`
/// positionally, so that order and this one cannot be allowed to drift apart. The
/// assertion below is what makes "cannot" true.
pub use duck_ipc_proto::JOINT_NAMES;

const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);

/// The mouth is absent from every alpha policy — they are all 61-D observation, 14-action,
/// and the action vector skips this index. Named so that omission is deliberate rather
/// than an off-by-one someone has to rediscover.
pub const MOUTH_INDEX: usize = 9;

/// Home pose. The trunk sits ~5 mm further forward than the v1.5 pose so the CoM is over
/// the ankle axis; the old pose biased the robot backwards.
///
/// Must match `HOME_FRAME` in the training env — a policy is trained against these angles
/// and observes joint positions *relative* to them, so a discrepancy here is a constant
/// offset on 14 observation slots.
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [
    0.0,     // left_hip_yaw
    -0.0873, // left_hip_roll
    -0.4579, // left_hip_pitch
    -0.0049, // left_knee
    0.4530,  // left_ankle
    0.3491,  // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth
    0.0,     // right_hip_yaw
    0.0873,  // right_hip_roll
    0.4579,  // right_hip_pitch
    0.0049,  // right_knee
    -0.4530, // right_ankle
];

/// The sitting pose: the training model's `SIT` keyframe (hips folded 30°, knees 60°, head up),
/// radians, indexed as [`JOINT_NAMES`]. The sitstand policy's real sit lands within ~0.12 rad
/// (mean over the legs) of it (2026-10-11 recording) — close enough to recognise a seated robot,
/// far enough from a limp, folded one (0.46–0.62 rad).
pub const SIT_POSITION: [f64; NUM_JOINTS] = [
    0.0,                          // left_hip_yaw
    0.0,                          // left_hip_roll
    -std::f64::consts::FRAC_PI_6, // left_hip_pitch (30 deg)
    std::f64::consts::FRAC_PI_3,  // left_knee (60 deg)
    0.0,                          // left_ankle
    0.4992,                       // neck_pitch
    1.6005,                       // head_pitch
    0.0,                          // head_yaw
    0.0,                          // head_roll
    0.0,                          // mouth
    0.0,                          // right_hip_yaw
    0.0,                          // right_hip_roll
    std::f64::consts::FRAC_PI_6,  // right_hip_pitch
    -std::f64::consts::FRAC_PI_3, // right_knee
    0.0,                          // right_ankle
];

/// Lying on the shins, feet and the soft jaw, trunk pitched 26° nose-down — the pose a limp
/// robot falls into and stays in (rev-a model, `training/LIEDOWN-DESIGN-20261013-CN.md`), and the
/// target of the `lie` policy. The mouth as at [`DEFAULT_POSITION`].
pub const LIE_POSITION: [f64; NUM_JOINTS] = [
    0.0,    // left_hip_yaw
    0.0,    // left_hip_roll
    -0.782, // left_hip_pitch
    1.485,  // left_knee
    0.227,  // left_ankle
    -0.940, // neck_pitch
    0.466,  // head_pitch
    0.0,    // head_yaw
    0.0,    // head_roll
    DEFAULT_POSITION[MOUTH_INDEX],
    0.0,    // right_hip_yaw
    0.0,    // right_hip_roll
    0.784,  // right_hip_pitch
    -1.484, // right_knee
    -0.229, // right_ankle
];

/// Projected gravity in the trunk frame lying at [`LIE_POSITION`] (upright is `[0, 0, -1]`;
/// positive x is nose-down — the robot tipping forward 51° read `[0.78, -0.05, -0.62]`).
pub const LIE_GRAVITY: [f64; 3] = [0.442, 0.0, -0.897];

/// The leg joints, indexed into [`JOINT_NAMES`].
pub const LEG_JOINTS: [usize; 10] = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14];

/// Mean absolute leg-joint distance between two poses, radians.
pub fn leg_distance(a: &[f64; NUM_JOINTS], b: &[f64; NUM_JOINTS]) -> f64 {
    LEG_JOINTS.iter().map(|&j| (a[j] - b[j]).abs()).sum::<f64>() / LEG_JOINTS.len() as f64
}

/// Per-joint travel the robot may be commanded to, radians, indexed as [`JOINT_NAMES`]:
/// the joint ranges of the training model (microduck_rl's `robot_allcollisions.xml`, which
/// every alpha and HL-2915 policy was trained in), the mouth's from [`MOUTH_CLOSED`] and
/// [`MOUTH_OPEN`].
///
/// A policy outside the states it was trained in emits targets far past these — on
/// 2026-10-05 the HL-2915 walking policy, handed a robot lying on the floor, asked for the
/// knees at +430° and -240° — and a servo commanded past where the joint can go stalls
/// against the stop at its current limit for as long as the command lasts.
pub const JOINT_LIMITS: [(f64, f64); NUM_JOINTS] = [
    (deg(-25.0), deg(30.0)),    // left_hip_yaw
    (deg(-22.0), deg(22.0)),    // left_hip_roll
    (deg(-90.0), deg(90.0)),    // left_hip_pitch
    (deg(-90.0), deg(90.0)),    // left_knee
    (deg(-90.0), deg(90.0)),    // left_ankle
    (deg(-90.0), deg(60.0)),    // neck_pitch
    (deg(-90.0), deg(90.0)),    // head_pitch
    (deg(-170.0), deg(170.0)),  // head_yaw
    (deg(-25.0), deg(25.0)),    // head_roll
    (MOUTH_CLOSED, MOUTH_OPEN), // mouth
    (deg(-30.0), deg(25.0)),    // right_hip_yaw
    (deg(-22.0), deg(22.0)),    // right_hip_roll
    (deg(-90.0), deg(90.0)),    // right_hip_pitch
    (deg(-90.0), deg(90.0)),    // right_knee
    (deg(-90.0), deg(90.0)),    // right_ankle
];

const fn deg(d: f64) -> f64 {
    d * std::f64::consts::PI / 180.0
}

/// Mouth travel, radians: closed and fully open. The alpha reuses the v1.6 range,
/// −5°..+30°, from `microduck_runtime`'s `variant.rs`.
///
/// The mouth is not part of any policy — every alpha network is 14 actions with this joint
/// skipped — so these two numbers and [`mouth_target`] are the whole of mouth control.
pub const MOUTH_CLOSED: f64 = -5.0 * std::f64::consts::PI / 180.0;
pub const MOUTH_OPEN: f64 = 30.0 * std::f64::consts::PI / 180.0;

/// Joint angle for a mouth opening fraction. 0 is closed, 1 is fully open; anything outside
/// is clamped rather than fed to a servo as an out-of-travel target.
pub fn mouth_target(open: f64) -> f64 {
    let open = if open.is_finite() {
        open.clamp(0.0, 1.0)
    } else {
        0.0
    };
    MOUTH_CLOSED + open * (MOUTH_OPEN - MOUTH_CLOSED)
}

/// The `imu_to_dxl` v2 board's Dynamixel ID. It rides the motor bus and is read in the
/// same transaction as the servos ([`crate::bus`]).
pub const IMU_DXL_ID: u8 = 200;

pub const BAUD_RATE: u32 = 1_000_000;

/// EEPROM registers asserted (and corrected) at startup.
///
/// `return_delay_time` is the load-bearing one: the XL330 ships at 250, which is 500 µs of
/// turnaround *per device*. Across 16 devices that is 8 ms per tick — 40% of a 20 ms budget
/// — spent waiting for servos to get around to answering. The rest are here because the
/// runtime found them worth pinning; `shutdown = 52` is the error mask that latches on
/// overload, overheating and input-voltage faults.
pub const EXPECTED_REGISTERS: &[(&str, u8)] = &[
    ("return_delay_time", 0),
    ("baud_rate", 3), // 3 = 1 Mbps, must agree with BAUD_RATE
    ("pwm_slope", 255),
    ("shutdown", 52),
];

/// Index of a joint by name. Linear scan over 15 entries, used at startup and in tests.
pub fn joint_index(name: &str) -> Option<usize> {
    JOINT_NAMES.iter().position(|n| *n == name)
}

// ── battery ──────────────────────────────────────────────────────────────────
//
// There is no fuel gauge and no ADC. The only measurement available is what the servos
// report as their own supply (`crate::bus::DynamixelIo::bus_voltage`), which is the pack
// seen through the bus — so it sags under load and recovers when the robot stands still.
// That is why the span below is *usable-under-load*, not the cell chemistry's range.
//
// Two packs so far, each LiPo or LiHV: 3S for the HL-2915 robot (Xiaobai, servos rated
// 9–14 V) and 2S for an HD-1910 one (4–8.4 V). robotd tells which is plugged in from a
// reading at rest, before any torque ([`identify_pack`]), and takes every threshold per cell
// from there — so one build serves both, and a robot's file only says which it takes.

/// Off a full charge, under load, per cell: LiPo. 4.10 V rather than the 4.20 the charger
/// leaves — the reading comes back through the servo bus, so even an unloaded pack reads a
/// little under its terminal voltage.
pub const CELL_FULL_V: f64 = 4.10;

/// The same for LiHV, charged to 4.35 V a cell.
pub const CELL_FULL_LIHV_V: f64 = 4.25;

/// The sag floor per cell: below this the robot starts struggling, well before the pack's own
/// protection trips. Empty for our purposes, not empty for the cells'.
///
/// 3.30 V. The cells are not flat there (3.0 is), but the HL-2915 servos want 9.0 V minimum
/// and a walking gait sags a 3S pack well over a volt — at 9.9 V idle the sags are already
/// reaching the servos' floor. It is also above the LiPo damage threshold, so a pack that sits
/// at this level overnight has not been hurt.
pub const CELL_EMPTY_V: f64 = 3.30;

/// A cell at rest, through the servo bus, reads between these: 3.0 V is flat, 4.35 V is a
/// full LiHV, with a little slack for the bus reading under the terminal.
const CELL_REST_MIN_V: f64 = 2.95;
const CELL_REST_MAX_V: f64 = 4.45;

/// Above this a cell at rest can only be LiHV: a LiPo off the charger stops at 4.20.
const CELL_LIHV_ABOVE_V: f64 = 4.22;

/// The packs a reading is matched against. Not 1S or 4S: a full 3S (12.6 V) is also a 4S at
/// 3.15 V a cell, and a robot that took either could never tell them apart.
const KNOWN_CELLS: [u8; 2] = [2, 3];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chemistry {
    LiPo,
    LiHv,
}

/// A pack: cells in series, and what they are charged to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pack {
    pub cells: u8,
    pub chemistry: Chemistry,
}

impl Pack {
    /// The 3S LiPo every number here was first measured on.
    pub const THREE_S_LIPO: Pack = Pack {
        cells: 3,
        chemistry: Chemistry::LiPo,
    };

    /// Empty under load: where `robotd` sits the robot down and powers it off.
    pub fn empty_v(&self) -> f64 {
        f64::from(self.cells) * CELL_EMPTY_V
    }

    /// Full under load.
    pub fn full_v(&self) -> f64 {
        f64::from(self.cells)
            * match self.chemistry {
                Chemistry::LiPo => CELL_FULL_V,
                Chemistry::LiHv => CELL_FULL_LIHV_V,
            }
    }

    /// The span a reading of this pack can plausibly be in — anything outside is a bad
    /// reading, and a scale computed from it would be wild.
    pub fn band(&self) -> (f64, f64) {
        let n = f64::from(self.cells);
        (n * 3.0, n * 4.35)
    }

    /// Fraction of the pack, 0–100, for a bus voltage.
    ///
    /// Linear, and the numbers come from `microduck_runtime`'s `check_battery`, where they
    /// were arrived at by running a duck flat. It lives here rather than in `robotd` so there
    /// is one mapping: the prototype had it in a CLI *and* re-derived in the app, which is
    /// how two screens end up disagreeing about the same pack.
    ///
    /// A non-finite or non-positive reading is 0 — those mean "no answer from the bus", and
    /// the caller is expected to report that as unknown rather than to display this number.
    pub fn percent(&self, volts: f64) -> f64 {
        if !volts.is_finite() || volts <= 0.0 {
            return 0.0;
        }
        ((volts - self.empty_v()) / (self.full_v() - self.empty_v())).clamp(0.0, 1.0) * 100.0
    }

    /// "3S LiPo", "2S LiHV".
    pub fn label(&self) -> String {
        let chem = match self.chemistry {
            Chemistry::LiPo => "LiPo",
            Chemistry::LiHv => "LiHV",
        };
        format!("{}S {chem}", self.cells)
    }
}

/// Which pack is plugged in, from a bus reading with the servos at rest (no torque yet).
///
/// The cell count is the one whose per-cell voltage is a cell's; exactly one must fit. 8.85–8.9 V
/// fits both a full 2S and a flat 3S, and is answered as unclear rather than guessed. LiHV shows
/// only while charged above what a LiPo can hold — below that the two discharge alike, and the
/// LiPo numbers serve both.
pub fn identify_pack(rest_v: f64) -> Result<Pack, String> {
    if !rest_v.is_finite() || rest_v <= 0.0 {
        return Err("no battery reading".to_owned());
    }
    let fits: Vec<u8> = KNOWN_CELLS
        .iter()
        .copied()
        .filter(|&n| (CELL_REST_MIN_V..=CELL_REST_MAX_V).contains(&(rest_v / f64::from(n))))
        .collect();
    match fits.as_slice() {
        [n] => Ok(Pack {
            cells: *n,
            chemistry: if rest_v / f64::from(*n) > CELL_LIHV_ABOVE_V {
                Chemistry::LiHv
            } else {
                Chemistry::LiPo
            },
        }),
        [] => Err(format!(
            "{rest_v:.2} V at rest is neither a 2S nor a 3S pack"
        )),
        _ => Err(format!(
            "{rest_v:.2} V at rest could be a full 2S or a flat 3S pack; charge it, or check it"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three tables are indexed by the same integer everywhere in the crate. If they
    /// ever diverge in length, every lookup silently reads the wrong joint.
    #[test]
    fn tables_agree_on_length() {
        assert_eq!(JOINT_IDS.len(), NUM_JOINTS);
        assert_eq!(JOINT_NAMES.len(), NUM_JOINTS);
        assert_eq!(DEFAULT_POSITION.len(), NUM_JOINTS);
    }

    /// A duplicated Dynamixel ID makes a `sync_read` return blocks that cannot be matched
    /// back to joints, and a `sync_write` command two joints at once. Both fail in ways
    /// that look like a wiring fault.
    #[test]
    fn ids_are_unique() {
        let mut seen = JOINT_IDS;
        seen.sort_unstable();
        seen.windows(2)
            .for_each(|w| assert_ne!(w[0], w[1], "duplicate Dynamixel ID {}", w[0]));
    }

    /// The IMU board shares the bus with the servos, so its ID must not collide with one.
    #[test]
    fn imu_id_does_not_collide_with_a_joint() {
        assert!(!JOINT_IDS.contains(&IMU_DXL_ID));
    }

    /// `MOUTH_INDEX` is used to skip a slot when mapping 14 policy actions onto 15 joints.
    /// Pointing it at the wrong joint would shift every action after it by one.
    #[test]
    fn mouth_index_names_the_mouth() {
        assert_eq!(JOINT_NAMES[MOUTH_INDEX], "mouth");
        assert_eq!(joint_index("mouth"), Some(MOUTH_INDEX));
    }

    /// The ends of the span and the middle of it, on the 3S LiPo the numbers were measured on
    /// (9.9 V empty, 12.3 V full). Getting the direction wrong here would report a full pack
    /// as flat, which is the kind of thing nobody double-checks.
    #[test]
    fn battery_percent_spans_the_usable_range() {
        let p = Pack::THREE_S_LIPO;
        assert!((p.empty_v() - 9.9).abs() < 1e-9 && (p.full_v() - 12.3).abs() < 1e-9);
        assert_eq!(p.percent(p.full_v()), 100.0);
        assert_eq!(p.percent(p.empty_v()), 0.0);
        assert!((p.percent(11.1) - 50.0).abs() < 0.001);
    }

    /// Voltages outside the span are ordinary — a fresh pack reads over full off the charger,
    /// and a robot being run into the ground reads under empty. Neither may produce a
    /// percentage outside 0–100 for a caller to display.
    #[test]
    fn battery_percent_clamps_rather_than_extrapolating() {
        assert_eq!(Pack::THREE_S_LIPO.percent(13.5), 100.0);
        assert_eq!(Pack::THREE_S_LIPO.percent(8.0), 0.0);
    }

    /// A bus that did not answer arrives here as 0.0, and NaN is what a mean over an empty
    /// set produces. Both must be 0 rather than a wild number a caller might print.
    #[test]
    fn battery_percent_treats_no_reading_as_zero() {
        let p = Pack::THREE_S_LIPO;
        assert_eq!(p.percent(0.0), 0.0);
        assert_eq!(p.percent(f64::NAN), 0.0);
        assert_eq!(p.percent(-1.0), 0.0);
    }

    /// The thresholds go per cell: a 2S pack is empty at 6.6 V, and an LiHV is full higher.
    #[test]
    fn a_2s_pack_and_an_lihv_pack_get_their_own_span() {
        let two = Pack {
            cells: 2,
            chemistry: Chemistry::LiPo,
        };
        assert!((two.empty_v() - 6.6).abs() < 1e-9 && (two.full_v() - 8.2).abs() < 1e-9);
        assert!((two.percent(7.4) - 50.0).abs() < 0.001);
        let hv = Pack {
            cells: 3,
            chemistry: Chemistry::LiHv,
        };
        assert!((hv.full_v() - 12.75).abs() < 1e-9);
        assert!(
            hv.percent(12.3) < 100.0,
            "an LiHV at a LiPo's full still has charge left"
        );
    }

    /// Every pack it will see, at rest, is told apart: full, half and nearly flat, both
    /// chemistries, both cell counts.
    #[test]
    fn packs_are_identified_from_a_reading_at_rest() {
        let lipo = |cells| Pack {
            cells,
            chemistry: Chemistry::LiPo,
        };
        let lihv = |cells| Pack {
            cells,
            chemistry: Chemistry::LiHv,
        };
        for (v, want) in [
            (12.45, lipo(3)), // a full 3S LiPo, read a little low through the bus
            (11.2, lipo(3)),
            (9.6, lipo(3)), // past empty: still a 3S, which robotd then sits down for
            (12.95, lihv(3)), // a full 3S LiHV (13.05 at the terminals)
            (8.3, lipo(2)),
            (7.4, lipo(2)),
            (6.4, lipo(2)),
            (8.65, lihv(2)), // a full 2S LiHV: over an HD-1910's 8.4 V, which the file's max_volts says
        ] {
            assert_eq!(identify_pack(v), Ok(want), "{v} V");
        }
    }

    /// What cannot be told is said, not guessed: a full 2S and a flat 3S overlap by a hair, a
    /// 4S or a 1S is neither, and no reading is no pack.
    #[test]
    fn what_cannot_be_told_apart_is_not_guessed() {
        assert!(
            identify_pack(8.87)
                .unwrap_err()
                .contains("full 2S or a flat 3S")
        );
        assert!(identify_pack(15.5).is_err());
        assert!(identify_pack(4.0).is_err());
        assert!(identify_pack(0.0).is_err());
        assert!(identify_pack(f64::NAN).is_err());
    }

    /// The mouth range is the prototype's: −5° closed, +30° open. A fraction outside 0..1
    /// (or a NaN from a broken client) must clamp rather than command a servo past travel.
    #[test]
    fn mouth_target_spans_the_prototype_range() {
        assert!((mouth_target(0.0) - (-5.0f64.to_radians())).abs() < 1e-12);
        assert!((mouth_target(1.0) - 30.0f64.to_radians()).abs() < 1e-12);
        assert_eq!(mouth_target(-3.0), mouth_target(0.0));
        assert_eq!(mouth_target(7.0), mouth_target(1.0));
        assert_eq!(mouth_target(f64::NAN), mouth_target(0.0));
    }

    /// The legs are mirrored: the roll/pitch/ankle pairs are equal and opposite. A sign
    /// typo in the home pose is invisible by inspection and makes the robot stand crooked.
    #[test]
    fn home_pose_legs_are_mirrored() {
        for (left, right) in [
            ("left_hip_roll", "right_hip_roll"),
            ("left_hip_pitch", "right_hip_pitch"),
            ("left_knee", "right_knee"),
            ("left_ankle", "right_ankle"),
        ] {
            let l = DEFAULT_POSITION[joint_index(left).unwrap()];
            let r = DEFAULT_POSITION[joint_index(right).unwrap()];
            assert!(
                (l + r).abs() < 1e-9,
                "{left} ({l}) and {right} ({r}) should be equal and opposite"
            );
        }
    }
}
