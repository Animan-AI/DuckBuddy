//! Where the robot is, estimated from its own legs and IMU.
//!
//! A port of the prototype runtime's contact-based odometry, itself derived
//! from Rhoban's humanoid `model_service`. The idea: at any moment one point of
//! one sole is the robot's contact with the ground. Anchor that point to the
//! world (it is on flat ground, so its world Z is 0 and its world X/Y are
//! wherever it was when it became the anchor), orient the trunk by the IMU, and
//! the trunk's world position follows by forward kinematics. When some other
//! sole corner drops below the anchor — a step — the anchor moves there, at
//! that corner's current world X/Y, so the estimate never jumps.
//!
//! Two changes from the prototype:
//!   - The foot chains come from the `kinematics` crate's MJCF model instead of
//!     hand-transcribed segment tables — the geometry has one source of truth.
//!   - Each foot's chain is evaluated once per tick and its four corners are
//!     transformed through the result; the prototype re-walked a full leg chain
//!     per corner (9 chain evaluations per tick, now 2).
//!
//! Heading is whatever the IMU's integrated yaw says — there is no
//! magnetometer, so the world frame is "wherever the robot was looking at
//! boot", which is all a relative-motion consumer needs.
//!
//! Alpha only, like the daemon: v1/v1.5 geometry stayed in the prototype.

use duck_ipc_proto::JOINT_NAMES;
use kinematics::{Model, Pose, Quat, SiteId};

/// Sole half-extents along the foot-site frame X (front/back) and Y
/// (left/right). Placeholder carried over from the prototype: the v1.5 sole
/// bbox values, until alpha's `sole_left.stl` is measured.
const SOLE_HALF_LEN: f64 = 0.0270;
const SOLE_HALF_WIDTH: f64 = 0.0206;

/// A candidate corner must sit below world Z = `-SWITCH_MARGIN` to bid for the
/// anchor. The anchor itself sits at Z = 0, so the margin is slack for FK and
/// IMU noise, not a physical depth.
const SWITCH_MARGIN: f64 = -0.010;

/// Ticks a candidate must stay the lowest point before the anchor moves. At
/// the control loop's 50 Hz this is 40 ms — well inside a stance phase, well
/// past a one-tick glitch.
const SWITCH_CONFIRM_TICKS: u32 = 2;

const LEFT: usize = 0;
const RIGHT: usize = 1;

pub struct Odometry {
    model: &'static Model,
    feet: [SiteId; 2],
    /// `JOINT_NAMES` position → model joint index. `None` for the mouth, which
    /// moves no leg.
    joint_map: [Option<usize>; JOINT_NAMES.len()],
    /// Scratch angle slice in model order, reused every tick.
    angles: Vec<f64>,
    /// Which foot the anchor is on.
    anchor_foot: usize,
    /// The contact point, in that foot's site frame.
    anchor_local: [f64; 3],
    /// The contact point's world X/Y. Its Z is 0 by definition: flat ground.
    anchor_xy: [f64; 2],
    position: [f64; 3],
    yaw: f64,
    /// A corner bidding to become the anchor: (foot, local point, world X/Y).
    pending: Option<(usize, [f64; 3], [f64; 2])>,
    pending_ticks: u32,
    /// True until the first update seeds `anchor_xy` from the actual startup
    /// pose, so the trunk starts at (0, 0) instead of offset by the foot.
    needs_init: bool,
}

impl Odometry {
    pub fn new(model: &'static Model) -> Self {
        let feet = [
            model.site("left_foot").expect("model has a left_foot site"),
            model
                .site("right_foot")
                .expect("model has a right_foot site"),
        ];
        Self {
            feet,
            joint_map: JOINT_NAMES.map(|name| model.joint_index(name)),
            angles: vec![0.0; model.num_joints()],
            model,
            anchor_foot: LEFT,
            anchor_local: [0.0; 3],
            anchor_xy: [0.0; 2],
            position: [0.0; 3],
            yaw: 0.0,
            pending: None,
            pending_ticks: 0,
            needs_init: true,
        }
    }

    pub fn alpha() -> Self {
        Self::new(Model::alpha())
    }

    /// Advance the estimate by one sensor sample.
    ///
    /// `joints` are measured angles in [`JOINT_NAMES`] order (the mouth is
    /// ignored); `quat_wxyz` is the IMU's trunk-in-world orientation. Call it
    /// on fresh samples only — a coasted tick has nothing new to say.
    pub fn update(&mut self, joints: &[f64; JOINT_NAMES.len()], quat_wxyz: [f64; 4]) {
        for (angle, idx) in joints.iter().zip(self.joint_map) {
            if let Some(idx) = idx {
                self.angles[idx] = *angle;
            }
        }
        let [w, x, y, z] = quat_wxyz;
        let rot = Quat::new(w, x, y, z).normalized();

        // Both foot poses, once — everything below reads these.
        let feet = [
            self.model.site_pose(self.feet[LEFT], &self.angles),
            self.model.site_pose(self.feet[RIGHT], &self.angles),
        ];

        if self.needs_init {
            let anchor_world = rot.rotate(feet[self.anchor_foot].pos);
            self.anchor_xy = [anchor_world[0], anchor_world[1]];
            self.needs_init = false;
        }

        self.reproject(rot, &feet);

        // A corner below the anchor is a step landing — but only after it holds
        // the claim for SWITCH_CONFIRM_TICKS, so FK jitter cannot walk the
        // anchor around mid-stance.
        match self.lowest_corner(rot, &feet) {
            None => {
                self.pending = None;
                self.pending_ticks = 0;
            }
            Some((foot, local, world_xy)) => {
                if self.pending.is_some_and(|(pf, _, _)| pf == foot) {
                    self.pending_ticks += 1;
                } else {
                    self.pending = Some((foot, local, world_xy));
                    self.pending_ticks = 1;
                }
                if self.pending_ticks >= SWITCH_CONFIRM_TICKS {
                    let (foot, local, world_xy) = self.pending.take().expect("just matched");
                    self.anchor_foot = foot;
                    self.anchor_local = local;
                    // The corner keeps the world X/Y it already has, so the
                    // estimate is continuous across the switch.
                    self.anchor_xy = world_xy;
                    self.reproject(rot, &feet);
                    self.pending_ticks = 0;
                }
            }
        }

        self.yaw = rot.yaw();
    }

    /// Trunk position in the world frame, metres. Z is height above the ground
    /// plane the anchor defines.
    pub fn position(&self) -> [f64; 3] {
        self.position
    }

    /// Trunk heading, radians, in the IMU's boot-relative world frame.
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    /// The contact anchor's world X/Y — where the estimator believes the stance
    /// foot touches the ground. For telemetry; `position` is the answer.
    pub fn anchor_xy(&self) -> [f64; 2] {
        self.anchor_xy
    }

    /// Which foot carries the anchor: 0 = left, 1 = right.
    pub fn anchor_foot(&self) -> usize {
        self.anchor_foot
    }

    /// Trunk position from the anchor: the contact point sits at
    /// (`anchor_xy`, 0), the trunk is minus the world-rotated trunk→contact
    /// vector away from it.
    fn reproject(&mut self, rot: Quat, feet: &[Pose; 2]) {
        let contact_in_trunk = feet[self.anchor_foot].transform_point(self.anchor_local);
        let contact = rot.rotate(contact_in_trunk);
        self.position = [
            self.anchor_xy[0] - contact[0],
            self.anchor_xy[1] - contact[1],
            -contact[2],
        ];
    }

    /// The lowest sole corner below the switch threshold, if any, with its
    /// current world X/Y.
    fn lowest_corner(&self, rot: Quat, feet: &[Pose; 2]) -> Option<(usize, [f64; 3], [f64; 2])> {
        const CORNERS: [[f64; 3]; 4] = [
            [SOLE_HALF_LEN, SOLE_HALF_WIDTH, 0.0],
            [SOLE_HALF_LEN, -SOLE_HALF_WIDTH, 0.0],
            [-SOLE_HALF_LEN, SOLE_HALF_WIDTH, 0.0],
            [-SOLE_HALF_LEN, -SOLE_HALF_WIDTH, 0.0],
        ];

        let mut lowest = -SWITCH_MARGIN;
        let mut best = None;
        for foot in [LEFT, RIGHT] {
            for corner in CORNERS {
                let in_world = rot.rotate(feet[foot].transform_point(corner));
                let world = [
                    self.position[0] + in_world[0],
                    self.position[1] + in_world[1],
                    self.position[2] + in_world[2],
                ];
                if world[2] < lowest {
                    lowest = world[2];
                    best = Some((foot, corner, [world[0], world[1]]));
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw_quat(yaw: f64) -> [f64; 4] {
        [(yaw / 2.0).cos(), 0.0, 0.0, (yaw / 2.0).sin()]
    }

    fn roll_quat(roll: f64) -> [f64; 4] {
        [(roll / 2.0).cos(), (roll / 2.0).sin(), 0.0, 0.0]
    }

    /// A robot standing still is at the origin and stays there — the first
    /// update seeds the anchor so the trunk starts at (0, 0), and constant
    /// inputs must not integrate into drift.
    #[test]
    fn standing_still_stays_at_the_origin() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..200 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }
        let [x, y, z] = odo.position();
        assert!(x.abs() < 1e-9 && y.abs() < 1e-9, "drifted to ({x}, {y})");
        assert!(z > 0.02, "trunk should stand above the ground, z = {z}");
        assert_eq!(odo.yaw(), 0.0);
    }

    #[test]
    fn yaw_follows_the_imu() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, yaw_quat(0.3));
        }
        assert!((odo.yaw() - 0.3).abs() < 1e-9);
    }

    /// The mouth is a face, not a leg: index 9 must not reach the FK.
    #[test]
    fn the_mouth_moves_no_odometry() {
        let mut still = Odometry::alpha();
        let mut chatting = Odometry::alpha();
        let quiet = [0.0; JOINT_NAMES.len()];
        let mut open = quiet;
        open[9] = 42.0;
        for _ in 0..10 {
            still.update(&quiet, [1.0, 0.0, 0.0, 0.0]);
            chatting.update(&open, [1.0, 0.0, 0.0, 0.0]);
        }
        assert_eq!(still.position(), chatting.position());
    }

    /// A one-tick disturbance must not move the anchor; a held one must. This
    /// is the temporal confirmation doing its job against FK/IMU glitches.
    #[test]
    fn the_anchor_switches_only_after_the_claim_holds() {
        let mut odo = Odometry::alpha();
        let joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }
        let settled_foot = odo.anchor_foot();

        // Roll the trunk so the OTHER foot is clearly lower — for one tick.
        // Positive roll about +x pushes -y (the right foot) down.
        let away = if settled_foot == LEFT { 0.35 } else { -0.35 };
        odo.update(&joints, roll_quat(away));
        odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            odo.anchor_foot(),
            settled_foot,
            "one glitchy tick stole the anchor"
        );

        // Held for several ticks, the other foot takes it.
        for _ in 0..5 {
            odo.update(&joints, roll_quat(away));
        }
        assert_ne!(odo.anchor_foot(), settled_foot, "a held claim must win");
    }

    /// Sweeping the stance hip moves the foot under the trunk, so the trunk
    /// must travel over the anchored foot — displacement with no teleporting:
    /// every tick's step stays small.
    #[test]
    fn stance_leg_motion_translates_the_trunk_continuously() {
        let mut odo = Odometry::alpha();
        let mut joints = [0.0; JOINT_NAMES.len()];
        for _ in 0..10 {
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
        }

        let hips = [
            JOINT_NAMES
                .iter()
                .position(|n| *n == "left_hip_pitch")
                .expect("exists"),
            JOINT_NAMES
                .iter()
                .position(|n| *n == "right_hip_pitch")
                .expect("exists"),
        ];
        let start = odo.position();
        let mut last = start;
        for i in 1..=50 {
            let angle = 0.2 * (i as f64 / 50.0);
            joints[hips[0]] = angle;
            joints[hips[1]] = angle;
            odo.update(&joints, [1.0, 0.0, 0.0, 0.0]);
            let now = odo.position();
            let step: f64 = (0..3)
                .map(|k| (now[k] - last[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(step < 0.02, "tick {i} jumped {step} m");
            last = now;
        }
        let moved = ((last[0] - start[0]).powi(2) + (last[1] - start[1]).powi(2)).sqrt();
        assert!(
            moved > 0.005,
            "a 0.2 rad hip sweep moved the trunk only {moved} m"
        );
        assert!(last.iter().all(|v| v.is_finite()));
    }
}
