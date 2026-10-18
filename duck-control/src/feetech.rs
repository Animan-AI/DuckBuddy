//! The Feetech bus: HL-2915 servos (HLS series) on the Orange Pi's UART7.
//!
//! Same shape as [`crate::bus`] — one combined `sync_read` per tick covering the IMU board
//! and all 15 servos, one `sync_write` of goal positions — on a different wire protocol and
//! a different register map. Feetech speaks Dynamixel protocol 1.0 framing (`FF FF`, one-byte
//! inverted-sum checksum, `SYNC_READ` 0x82), so `rustypot`'s v1 handler drives it; the
//! register *map* is Feetech's own, and the HLS map is not the STS one at 44/46 — see
//! [`GOAL_CURRENT`].
//!
//! What differs from the XL330 bus, and why each matters:
//!
//! - **500 kbit/s, not 1 Mbit/s.** UART7 on the A733 is clocked from a 24 MHz crystal and
//!   cannot divide down to 1 Mbit/s: asking for it silently yields 750 kbit/s, which no servo
//!   understands. 500k divides exactly. A 16-device read costs ~7.5 ms of a 20 ms tick.
//! - **Positions are sign-magnitude**, bit 15 the sign, not two's complement. Single-turn
//!   servos never set it, but velocity and current do.
//! - **Voltage and temperature sit inside the tick's block**, so [`RobotIo::slow_sensors`]
//!   costs no transaction of its own: it reports what the last tick already read.
//! - **Three RAM registers must be written at every power-up or the servo does not move**:
//!   goal current (44), running speed (46) and acceleration (41). See [`FeetechIo::check_registers`].
//! - **The ID → joint mapping is configuration**, not a constant: the servos were numbered
//!   1..15 down the harness on this robot, not 10–34 by limb as the original was.

use std::f64::consts::PI;
use std::time::Duration;

use rustypot::servo::feetech::sts3215::Sts3215Controller;

use crate::bus::StaleImuTracker;
use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
use crate::io::{ImuStale, IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::{IMU_DXL_ID, NUM_JOINTS};

/// Exactly representable on the A733's UART7 (24 MHz / 16 / 3). 1 Mbit/s is not.
pub const BAUD_RATE: u32 = 500_000;

/// Value of the baud register (6) that means [`BAUD_RATE`]: 0..7 = 1M/500k/250k/128k/115200/
/// 76800/57600/38400.
pub const BAUD_REGISTER_VALUE: u8 = 1;

// ── registers ────────────────────────────────────────────────────────────────

const BAUD: u8 = 6;
const MIN_ANGLE_LIMIT: u8 = 9;
const MAX_ANGLE_LIMIT: u8 = 11;
/// The *live* position-loop gains, in RAM: Kp (1/8 units) and Kd (1/4 units). The HLS copies
/// its EEPROM gains (21, 22) here at power-up and never reads 21/22 again — writing 21 while
/// running changes the register and nothing about the servo, which is how the first stand
/// test ran at full factory stiffness. Measured against the mouth's stop: torque per radian
/// rises with Kp at roughly 0.35 N·m/rad per unit past a small dead band.
const KP: u8 = 50;
const KD: u8 = 51;
const MODE: u8 = 33;
const TORQUE_ENABLE: u8 = 40;
const ACCELERATION: u8 = 41;
const GOAL_POSITION: u8 = 42;
/// **HLS only**: on the STS map this is "goal time". On the HLS it is the current the motor
/// may draw, 6.5 mA per count — and at 0 the servo has no torque at all. RAM.
const GOAL_CURRENT: u8 = 44;
/// Running speed, 0.732 rpm per count. **0 means stop**, not "unlimited". RAM.
const GOAL_SPEED: u8 = 46;
const PRESENT_POSITION: u8 = 56;
const PRESENT_VOLTAGE: u8 = 62;

/// Start and length of the block read every tick: 56..70 — position, speed, load, voltage,
/// temperature, (64), status, moving, (67..68), current. The IMU board serves a block of the
/// same length at the same address, its first [`IMU_BLOCK_LEN`] bytes being the orientation.
const READ_ADDR: u8 = PRESENT_POSITION;
const READ_LEN: u8 = 15;

// Offsets into the per-servo block.
const OFF_POSITION: usize = 0;
const OFF_SPEED: usize = 2;
const OFF_VOLTAGE: usize = 6;
const OFF_TEMPERATURE: usize = 7;
const OFF_STATUS: usize = 9;
const OFF_CURRENT: usize = 13;

// ── units ────────────────────────────────────────────────────────────────────

/// One revolution of the magnetic encoder.
const COUNTS_PER_REV: f64 = 4096.0;
/// Encoder reading at the joint's zero, before the per-joint offset. The servos were centred
/// here with the horns mounted at the neutral pose.
const CENTER: i32 = 2048;
const MAX_POSITION: i32 = 4095;

/// Present speed, rad/s per count: 0.732 rpm, the same unit as [`GOAL_SPEED`] — *not* encoder
/// counts per second as on the STS series. Measured on the robot (head yaw, 2026-10-04):
/// commanded 20 units, the joint covered the middle 60% of its travel at 1011–1015 counts/s
/// while the register read 20.0 — 50.5 counts/s per unit, against 0.732 rpm's 50.0. Getting
/// this wrong scales every joint velocity in the observation by 50.
///
/// Coarse: one count is ~4.4°/s, about three times the XL330's 0.229 rpm.
const RAD_PER_SEC_PER_COUNT: f64 = 0.732 * 2.0 * PI / 60.0;

/// Present current, mA per count.
const MA_PER_COUNT: f64 = 6.5;

/// Present voltage, volts per count.
const VOLTS_PER_COUNT: f64 = 0.1;

/// Written to [`GOAL_SPEED`] at bring-up: the power-up default, far above the HL-2915's
/// ~110 rpm, so the servo is never speed-limited and position tracking is the policy's to
/// shape. Written anyway because a servo reset by a brown-out comes back with whatever its
/// firmware chose, and "it does not move" is the symptom of a zero here.
const RUNNING_SPEED: u16 = 250;

/// Written to [`ACCELERATION`]: 0 is the servo's maximum. The XL330 bus used no profile at all.
const RUNNING_ACCELERATION: u8 = 0;

/// The XL330 position gain the policies run at (`policy.gain`). A request for this gain maps to
/// [`FeetechConfig::position_kp`]; anything else scales proportionally, so the standing and
/// limp ratios the control loop relies on carry over unchanged.
pub const KP_REFERENCE: f64 = 200.0;

/// A healthy 16-device read at 500k takes ~7.5 ms. Same cap as the Dynamixel bus.
const READ_TIMEOUT: Duration = Duration::from_millis(30);

// ── configuration ────────────────────────────────────────────────────────────

/// Which servo is which joint, and how its encoder maps onto the joint angle.
///
/// Everything is indexed as [`crate::model::JOINT_NAMES`].
#[derive(Debug, Clone, PartialEq)]
pub struct FeetechConfig {
    /// Bus ID of the servo driving each joint.
    pub ids: [u8; NUM_JOINTS],
    /// +1 where a positive encoder step is a positive joint angle, −1 where the servo is
    /// mounted the other way round.
    pub signs: [i8; NUM_JOINTS],
    /// Encoder counts from [`CENTER`] to the joint's zero. Calibration lives here, not in the
    /// servos' own offset register, so it travels with the robot's configuration.
    pub offsets: [i16; NUM_JOINTS],
    /// Whether the IMU board is on the bus. Off for bring-up before its firmware exists: the
    /// loop then reads only the servos, and [`RobotIo::imu_ready`] stays false so nothing that
    /// needs orientation will run.
    pub imu: bool,
    /// Current each servo may draw, mA. Written to [`GOAL_CURRENT`] at bring-up.
    pub current_limit_ma: f64,
    /// Joints left out of every torque-enable and goal write, so their servo stays limp while
    /// the rest hold. Both have to skip it: an HLS switches its own torque on at a goal write.
    /// (A goal current of 0 does not do it — measured 2026-10-12, the mouth went rigid at its
    /// limp pose and heated 8 °C in a minute.)
    pub limp: [bool; NUM_JOINTS],
    /// IMU sensor→trunk rotation, scalar-first — see [`crate::imu::mount_from_axes`].
    pub imu_mount: [f64; 4],
    /// Median of three ticks on the IMU — see [`SflpDecoder::with_median`].
    pub imu_median: bool,
    /// Live Kp (register 50) at the policies' running gain, per joint. The factory 32 is
    /// ~11 N·m/rad — some fifteen times the 0.55 N·m/rad the policies were trained against,
    /// and enough to make the walking policy kick and fall in simulation and on the robot.
    /// Per joint because the neck carries the head: at a leg's softness it sags 15°.
    pub position_kp: [u8; NUM_JOINTS],
    /// Live Kd (register 51), written once at bring-up.
    pub position_kd: u8,
}

impl FeetechConfig {
    /// Check a mapping before it reaches the bus. A duplicated ID would command two joints
    /// at once and make a `sync_read` unmatchable; a sign of 0 would freeze a joint at zero.
    pub fn validate(&self) -> std::result::Result<(), String> {
        let mut seen = [false; 256];
        for (joint, &id) in self.ids.iter().enumerate() {
            if id == 0 || id >= 0xFE {
                return Err(format!("joint {joint}: ID {id} is not a servo ID"));
            }
            if id == IMU_DXL_ID {
                return Err(format!("joint {joint}: ID {id} is the IMU board's"));
            }
            if std::mem::replace(&mut seen[id as usize], true) {
                return Err(format!("ID {id} is assigned to more than one joint"));
            }
        }
        if let Some(joint) = self.signs.iter().position(|s| *s != 1 && *s != -1) {
            return Err(format!("joint {joint}: sign must be 1 or -1"));
        }
        if let Some(joint) = self.offsets.iter().position(|o| o.unsigned_abs() > 2048) {
            return Err(format!("joint {joint}: offset beyond half a turn"));
        }
        if self.position_kp.iter().any(|k| *k == 0 || *k == 255) || self.position_kd == 255 {
            return Err(format!(
                "position gains {:?}/{} outside 1..254 / 0..254",
                self.position_kp, self.position_kd
            ));
        }
        if !(self.current_limit_ma > 0.0 && self.current_limit_ma <= 2047.0 * MA_PER_COUNT) {
            return Err(format!(
                "current limit {} mA outside 0..{} mA",
                self.current_limit_ma,
                2047.0 * MA_PER_COUNT
            ));
        }
        Ok(())
    }

    fn current_limit_counts(&self) -> u16 {
        (self.current_limit_ma / MA_PER_COUNT)
            .round()
            .clamp(1.0, 2047.0) as u16
    }
}

// ── encoding ─────────────────────────────────────────────────────────────────

/// Sign-magnitude (bit 15 = negative) to a signed integer.
fn sign_magnitude(raw: u16) -> i32 {
    let magnitude = (raw & 0x7FFF) as i32;
    if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

fn le16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

/// Encoder reading to joint angle, radians.
fn counts_to_rad(raw: u16, sign: i8, offset: i16) -> f64 {
    let counts = sign_magnitude(raw) - CENTER - offset as i32;
    sign as f64 * counts as f64 * 2.0 * PI / COUNTS_PER_REV
}

/// Joint angle to the goal-position register. Clamped to the single-turn range, so a target
/// past the end of travel is commanded as the end of travel rather than wrapping to the other
/// side of the servo.
fn rad_to_counts(rad: f64, sign: i8, offset: i16) -> u16 {
    let rad = if rad.is_finite() { rad } else { 0.0 };
    let counts = CENTER as f64 + offset as f64 + sign as f64 * rad * COUNTS_PER_REV / (2.0 * PI);
    (counts.round() as i64).clamp(0, MAX_POSITION as i64) as u16
}

/// The live Kp for an XL330-scale gain, given the Kp that stands for the running gain.
fn kp_for_gain(kp: u16, running: u8) -> u8 {
    (running as f64 * kp as f64 / KP_REFERENCE)
        .round()
        .clamp(1.0, 254.0) as u8
}

/// What one tick's block says about one servo.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ServoSample {
    position: f64,
    velocity: f64,
    current_ma: f64,
    volts: f64,
    temp_c: f64,
    status: u8,
}

fn decode_block(block: &[u8], sign: i8, offset: i16) -> ServoSample {
    ServoSample {
        position: counts_to_rad(le16(block, OFF_POSITION), sign, offset),
        velocity: sign as f64
            * sign_magnitude(le16(block, OFF_SPEED)) as f64
            * RAD_PER_SEC_PER_COUNT,
        current_ma: (sign_magnitude(le16(block, OFF_CURRENT)) as f64 * MA_PER_COUNT).abs(),
        volts: block[OFF_VOLTAGE] as f64 * VOLTS_PER_COUNT,
        temp_c: block[OFF_TEMPERATURE] as f64,
        status: block[OFF_STATUS],
    }
}

// ── the bus ──────────────────────────────────────────────────────────────────

pub struct FeetechIo {
    controller: Sts3215Controller,
    config: FeetechConfig,
    /// The servos in joint order, then the IMU when it is on the bus — the order blocks come
    /// back in.
    ///
    /// **The IMU goes last**, the opposite of the XL330 bus. Feetech servos do not wait to hear
    /// the device ahead of them finish: each starts its reply on its own timing, so a board that
    /// answers even ~60 µs slower than a servo has its reply's tail trampled by the next servo
    /// (measured: 990/1000 reads clean with the IMU first, 1000/1000 with it last). The IMU
    /// firmware counts the replies ahead of it before speaking, and nothing follows it.
    ids: Vec<u8>,
    imu: SflpDecoder,
    stale_imu: StaleImuTracker,
    /// Voltage and temperatures from the last tick, which is where [`RobotIo::slow_sensors`]
    /// gets them — they arrive in the tick's block for free.
    last_slow: Option<SlowSensors>,
    /// Last status byte per joint, so a fault is logged when it appears rather than every tick.
    last_status: [u8; NUM_JOINTS],
    /// Whether torque was last switched on. **An HLS turns its own torque on when it receives
    /// a goal position** — unicast or SYNC_WRITE, measured — so with torque off, goal writes
    /// are dropped here: otherwise the control loop, which writes targets every tick whether
    /// or not anything is driving, stiffens a limp robot 20 ms after `relax` turned it off,
    /// and holds it rigid from the moment the daemon starts.
    torque_on: bool,
    /// Joints left out of torque-enable and goal writes: the config's (`FeetechConfig::limp`,
    /// the mouth) plus whatever [`RobotIo::set_limp`] asked for at run time.
    limp: [bool; NUM_JOINTS],
}

impl FeetechIo {
    pub fn open(port: &str, config: FeetechConfig) -> Result<Self> {
        config.validate().map_err(IoError::Bus)?;
        let serial = serialport::new(port, BAUD_RATE)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(|e| IoError::Port {
                path: port.to_owned(),
                source: std::io::Error::other(e),
            })?;

        let controller = Sts3215Controller::new()
            .with_protocol_v1()
            .with_serial_port(serial);

        let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
        ids.extend_from_slice(&config.ids);
        if config.imu {
            ids.push(IMU_DXL_ID);
        }

        let imu = SflpDecoder::new(config.imu_mount).with_median(config.imu_median);
        let limp = config.limp;
        Ok(Self {
            controller,
            config,
            ids,
            imu,
            stale_imu: StaleImuTracker::default(),
            last_slow: None,
            last_status: [0; NUM_JOINTS],
            torque_on: false,
            limp,
        })
    }

    fn read_u8(&mut self, id: u8, addr: u8) -> Result<u8> {
        let raw = self
            .controller
            .read_raw_data(id, addr, 1)
            .map_err(|e| IoError::Bus(format!("read {addr} on {id}: {e}")))?;
        raw.first().copied().ok_or(IoError::ShortRead {
            what: "register read",
            expected: 1,
            got: 0,
        })
    }

    fn read_u16(&mut self, id: u8, addr: u8) -> Result<u16> {
        let raw = self
            .controller
            .read_raw_data(id, addr, 2)
            .map_err(|e| IoError::Bus(format!("read {addr} on {id}: {e}")))?;
        if raw.len() != 2 {
            return Err(IoError::ShortRead {
                what: "register read",
                expected: 2,
                got: raw.len(),
            });
        }
        Ok(le16(&raw, 0))
    }

    fn write(&mut self, id: u8, addr: u8, data: Vec<u8>) -> Result<()> {
        self.controller
            .write_raw_data(id, addr, data)
            .map_err(|e| IoError::Bus(format!("write {addr} on {id}: {e}")))
    }

    /// Verify the EEPROM settings the loop depends on, then write the RAM registers a servo
    /// needs before it will move.
    ///
    /// EEPROM is **checked, not corrected**. Changing it means unlocking the servo, and the
    /// settings that matter here — baud, mode, travel limits — were set deliberately with the
    /// servo tools; a mismatch means a swapped or reset servo that someone should look at, not
    /// something to paper over at boot. (A wrong baud cannot even be read: that servo simply
    /// does not answer, which fails this the same way an unpowered bus does.)
    ///
    /// The RAM writes are every boot, because the servo forgets them at every power cycle *and*
    /// at every brown-out reset — after which goal current is back at its 2.9 A default and the
    /// limit configured here is silently gone. Returns 0: nothing is "corrected".
    pub fn check_registers(&mut self) -> Result<usize> {
        let limit = self.config.current_limit_counts();
        for (joint, &id) in self.config.ids.clone().iter().enumerate() {
            let baud = self.read_u8(id, BAUD)?;
            let mode = self.read_u8(id, MODE)?;
            let min = self.read_u16(id, MIN_ANGLE_LIMIT)?;
            let max = self.read_u16(id, MAX_ANGLE_LIMIT)?;
            if baud != BAUD_REGISTER_VALUE || mode != 0 || min != 0 || max != MAX_POSITION as u16 {
                return Err(IoError::Bus(format!(
                    "servo {id} is not configured for this robot: baud register {baud} \
                     (want {BAUD_REGISTER_VALUE}), mode {mode} (want 0), limits {min}/{max} \
                     (want 0/{MAX_POSITION}) — reconfigure it with tools/servo"
                )));
            }
            self.write(id, GOAL_CURRENT, limit.to_le_bytes().to_vec())?;
            self.write(id, GOAL_SPEED, RUNNING_SPEED.to_le_bytes().to_vec())?;
            self.write(id, ACCELERATION, vec![RUNNING_ACCELERATION])?;
            // The control loop sets Kp on its first tick; this covers `robotd init` and the
            // window before, which would otherwise run on the servo's factory 32.
            self.write(id, KP, vec![self.config.position_kp[joint]])?;
            self.write(id, KD, vec![self.config.position_kd])?;
            // Start from a known state, matching `torque_on = false`: a servo left holding by an
            // earlier process (or a goal write) would otherwise stay rigid with nothing driving.
            self.write(id, TORQUE_ENABLE, vec![0])?;
        }
        self.torque_on = false;
        Ok(0)
    }

    /// Present positions only — used once at startup to adopt the pose the robot is in.
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let blocks = self
            .controller
            .sync_read_raw_data(&self.config.ids, PRESENT_POSITION, 2)
            .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
        if blocks.len() != NUM_JOINTS || blocks.iter().any(|b| b.len() != 2) {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: NUM_JOINTS,
                got: blocks.len(),
            });
        }
        let mut out = [0.0; NUM_JOINTS];
        for (j, block) in blocks.iter().enumerate() {
            out[j] = counts_to_rad(le16(block, 0), self.config.signs[j], self.config.offsets[j]);
        }
        Ok(out)
    }

    /// Torque on or off, every servo. One transaction per joint: not for the tick.
    ///
    /// Turning on, each servo's goal is first set to where it already is. A servo keeps its
    /// last goal across torque-off — whatever the previous process, or a test script, left
    /// there — and enabling torque drives straight to it, at full speed and full current.
    /// On a limp robot that goal is typically far from the pose it has slumped into.
    /// IDs and joint indices of the servos that are driven — all but [`FeetechConfig::limp`].
    fn driven(&self) -> (Vec<u8>, Vec<usize>) {
        let joints: Vec<usize> = (0..NUM_JOINTS).filter(|&j| !self.limp[j]).collect();
        (joints.iter().map(|&j| self.config.ids[j]).collect(), joints)
    }

    /// See [`RobotIo::set_limp`]. A joint going limp gets its torque cut now (if torque is on);
    /// one coming back gets its goal set to where it is, then torque — the no-jump rule
    /// `set_torque` follows. The config's own limp joints (the mouth) stay limp. Tracked per
    /// joint, so a write failing part-way leaves `self.limp` saying what the servos are doing.
    pub fn set_limp(&mut self, limp: [bool; NUM_JOINTS]) -> Result<()> {
        for (joint, &asked) in limp.iter().enumerate() {
            let want = asked || self.config.limp[joint];
            if want == self.limp[joint] {
                continue;
            }
            if self.torque_on {
                let id = self.config.ids[joint];
                if want {
                    self.write(id, TORQUE_ENABLE, vec![0])?;
                } else {
                    let present = self.read_u16(id, PRESENT_POSITION)?;
                    self.write(id, GOAL_POSITION, present.to_le_bytes().to_vec())?;
                    self.write(id, TORQUE_ENABLE, vec![1])?;
                }
            }
            self.limp[joint] = want;
        }
        Ok(())
    }

    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        if !on {
            // Before the writes: if one fails part-way, goal writes must already have stopped,
            // or the next tick turns the servos that did switch off straight back on.
            self.torque_on = false;
        }
        if on {
            let blocks = self
                .controller
                .sync_read_raw_data(&self.config.ids, PRESENT_POSITION, 2)
                .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
            if blocks.len() != NUM_JOINTS || blocks.iter().any(|b| b.len() != 2) {
                return Err(IoError::ShortRead {
                    what: "present positions",
                    expected: NUM_JOINTS,
                    got: blocks.len(),
                });
            }
            let (ids, joints) = self.driven();
            let blocks: Vec<Vec<u8>> = joints.iter().map(|&j| blocks[j].clone()).collect();
            // Every joint limp (a rest with the head down too) leaves nothing to write, and a
            // sync_write to no servos divides by zero inside rustypot — 2026-10-13, it took the
            // control thread down.
            if !ids.is_empty() {
                self.controller
                    .sync_write_raw_data(&ids, GOAL_POSITION, &blocks)
                    .map_err(|e| IoError::Bus(format!("sync_write hold positions: {e}")))?;
            }
        }
        for (joint, &id) in self.config.ids.clone().iter().enumerate() {
            // Off goes to every servo, limp ones included, in case one was left holding.
            if on && self.limp[joint] {
                continue;
            }
            self.write(id, TORQUE_ENABLE, vec![on as u8])?;
        }
        self.torque_on = on;
        Ok(())
    }

    /// Ramp every joint from where it is to `target`, linearly. Only `robotd init` calls this.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64())
            .ceil()
            .max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mut next = [0.0; NUM_JOINTS];
            for j in 0..NUM_JOINTS {
                next[j] = start[j] + (target[j] - start[j]) * t;
            }
            RobotIo::write(self, &JointTargets::new(next))?;
            std::thread::sleep(step);
        }
        Ok(())
    }

    /// Log a servo's status byte when it changes. Bits are protection trips (voltage, angle,
    /// overheat, overcurrent, overload); a servo that has tripped is limp until reset, which
    /// looks exactly like a mechanical fault unless someone says so.
    fn note_status(&mut self, joint: usize, status: u8) {
        if status != self.last_status[joint] {
            if status != 0 {
                tracing::warn!(
                    joint = crate::model::JOINT_NAMES[joint],
                    id = self.config.ids[joint],
                    status = format_args!("{status:#04x}"),
                    "servo reports a fault"
                );
            } else {
                tracing::info!(
                    joint = crate::model::JOINT_NAMES[joint],
                    id = self.config.ids[joint],
                    "servo fault cleared"
                );
            }
            self.last_status[joint] = status;
        }
    }
}

impl RobotIo for FeetechIo {
    fn read(&mut self) -> Result<Sensors> {
        let blocks = self
            .controller
            .sync_read_raw_data(&self.ids, READ_ADDR, READ_LEN)
            .map_err(|e| IoError::Bus(format!("combined imu+motor sync_read: {e}")))?;
        if blocks.len() != self.ids.len() {
            return Err(IoError::ShortRead {
                what: "sync_read blocks",
                expected: self.ids.len(),
                got: blocks.len(),
            });
        }

        let mut sensors = Sensors::default();
        let servo_blocks = if self.config.imu {
            let block = &blocks[NUM_JOINTS];
            if block.len() < IMU_BLOCK_LEN {
                return Err(IoError::ShortRead {
                    what: "imu block",
                    expected: IMU_BLOCK_LEN,
                    got: block.len(),
                });
            }
            let mut raw = [0u8; IMU_BLOCK_LEN];
            raw.copy_from_slice(&block[..IMU_BLOCK_LEN]);
            let run = self.stale_imu.observe(&raw);
            if run == crate::bus::STALE_RUN_WARN
                || (run > crate::bus::STALE_RUN_WARN && run.is_multiple_of(500))
            {
                tracing::warn!(
                    consecutive = run,
                    total = self.stale_imu.stale.total,
                    "imu board has returned the same sample {run} reads running — orientation is frozen"
                );
            }
            sensors.imu = self.imu.decode(&raw);
            &blocks[..NUM_JOINTS]
        } else {
            &blocks[..]
        };

        let mut volts = 0.0;
        let mut counted = 0;
        let mut temps_c = [0.0; NUM_JOINTS];
        let mut statuses = [0u8; NUM_JOINTS];
        for (joint, block) in servo_blocks.iter().enumerate() {
            if block.len() != READ_LEN as usize {
                return Err(IoError::ShortRead {
                    what: "motor block",
                    expected: READ_LEN as usize,
                    got: block.len(),
                });
            }
            let s = decode_block(block, self.config.signs[joint], self.config.offsets[joint]);
            sensors.positions[joint] = s.position;
            sensors.velocities[joint] = s.velocity;
            sensors.currents_ma[joint] = s.current_ma;
            temps_c[joint] = s.temp_c;
            statuses[joint] = s.status;
            // A zero is a device answering nonsense, not a flat pack; keep it out of the mean.
            if s.volts > 0.0 {
                volts += s.volts;
                counted += 1;
            }
        }
        for (joint, status) in statuses.into_iter().enumerate() {
            self.note_status(joint, status);
        }
        if counted > 0 {
            self.last_slow = Some(SlowSensors {
                volts: volts / counted as f64,
                temps_c,
            });
        }
        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        if !self.torque_on {
            // A goal write would switch torque on (see `torque_on`).
            return Ok(());
        }
        let (ids, joints) = self.driven();
        if ids.is_empty() {
            // nothing driven: every joint limp (see `set_torque`)
            return Ok(());
        }
        let data: Vec<Vec<u8>> = joints
            .iter()
            .map(|&j| {
                rad_to_counts(
                    targets.positions[j],
                    self.config.signs[j],
                    self.config.offsets[j],
                )
                .to_le_bytes()
                .to_vec()
            })
            .collect();
        self.controller
            .sync_write_raw_data(&ids, GOAL_POSITION, &data)
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))
    }

    fn set_limp(&mut self, limp: [bool; NUM_JOINTS]) -> Result<()> {
        FeetechIo::set_limp(self, limp)
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        FeetechIo::set_torque(self, on)
    }

    /// Scale the servos' live Kp (register 50) to an XL330-scale gain. RAM: a power cycle
    /// restores the servo's own EEPROM value, which bring-up then overwrites again.
    fn set_gain(&mut self, kp: u16) -> Result<()> {
        for (joint, &id) in self.config.ids.clone().iter().enumerate() {
            let p = kp_for_gain(kp, self.config.position_kp[joint]);
            self.write(id, KP, vec![p])?;
        }
        Ok(())
    }

    /// Voltage and temperatures as of the last tick. Only before the first tick does this cost
    /// a transaction of its own.
    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        if let Some(slow) = self.last_slow {
            return Ok(slow);
        }
        let blocks = self
            .controller
            .sync_read_raw_data(&self.config.ids, PRESENT_VOLTAGE, 2)
            .map_err(|e| IoError::Bus(format!("voltage+temperature sync_read: {e}")))?;
        let mut temps_c = [0.0; NUM_JOINTS];
        let mut volts = Vec::with_capacity(NUM_JOINTS);
        for (joint, block) in blocks.iter().enumerate().take(NUM_JOINTS) {
            if block.len() != 2 {
                return Err(IoError::ShortRead {
                    what: "voltage+temperature block",
                    expected: 2,
                    got: block.len(),
                });
            }
            let v = block[0] as f64 * VOLTS_PER_COUNT;
            if v > 0.0 {
                volts.push(v);
            }
            temps_c[joint] = block[1] as f64;
        }
        if volts.is_empty() {
            return Err(IoError::ShortRead {
                what: "input voltage",
                expected: NUM_JOINTS,
                got: 0,
            });
        }
        Ok(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    }

    fn imu_stale(&self) -> ImuStale {
        self.stale_imu.stale
    }

    fn imu_ready(&self) -> bool {
        self.config.imu && self.imu.ready()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> FeetechConfig {
        FeetechConfig {
            ids: std::array::from_fn(|j| j as u8 + 1),
            signs: [1; NUM_JOINTS],
            offsets: [0; NUM_JOINTS],
            imu: true,
            current_limit_ma: 2925.0,
            limp: [false; NUM_JOINTS],
            imu_mount: SflpDecoder::DEFAULT_MOUNT,
            imu_median: true,
            position_kp: [8; NUM_JOINTS],
            position_kd: 8,
        }
    }

    #[test]
    fn a_plain_mapping_validates() {
        assert_eq!(config().validate(), Ok(()));
    }

    /// Two joints on one ID would be commanded together and make every `sync_read` ambiguous.
    #[test]
    fn duplicate_ids_are_refused() {
        let mut c = config();
        c.ids[3] = c.ids[7];
        assert!(c.validate().unwrap_err().contains("more than one joint"));
    }

    #[test]
    fn the_imu_id_and_broadcast_are_refused() {
        let mut c = config();
        c.ids[0] = IMU_DXL_ID;
        assert!(c.validate().is_err());
        let mut c = config();
        c.ids[0] = 0xFE;
        assert!(c.validate().is_err());
    }

    #[test]
    fn a_zero_sign_is_refused() {
        let mut c = config();
        c.signs[5] = 0;
        assert!(c.validate().is_err());
    }

    /// Bit 15 is the sign, the rest magnitude — not two's complement. 0x8005 is −5, where
    /// reading it as an i16 would give −32763.
    #[test]
    fn sign_magnitude_is_not_twos_complement() {
        assert_eq!(sign_magnitude(5), 5);
        assert_eq!(sign_magnitude(0x8005), -5);
        assert_eq!(sign_magnitude(0x8000), 0);
        assert_eq!(sign_magnitude(0x7FFF), 32767);
    }

    /// 2048 is the joint's zero, a quarter turn either side is ±π/2 — the XL330 bus's
    /// convention, so policies see the same angles on either servo.
    #[test]
    fn the_center_is_zero_and_the_scale_is_one_turn() {
        assert_eq!(counts_to_rad(2048, 1, 0), 0.0);
        assert!((counts_to_rad(3072, 1, 0) - PI / 2.0).abs() < 1e-12);
        assert!((counts_to_rad(1024, 1, 0) + PI / 2.0).abs() < 1e-12);
    }

    #[test]
    fn sign_and_offset_apply_as_documented() {
        // Mounted reversed: a positive encoder step is a negative joint angle.
        assert!((counts_to_rad(3072, -1, 0) + PI / 2.0).abs() < 1e-12);
        // Joint zero sits 100 counts above the encoder's centre.
        assert_eq!(counts_to_rad(2148, 1, 100), 0.0);
        assert_eq!(rad_to_counts(0.0, 1, 100), 2148);
        assert_eq!(rad_to_counts(0.0, -1, -100), 1948);
    }

    /// What the loop reads back must be what it commanded, for every sign and offset — or a
    /// servo holding perfectly still would look to the policy like it was being pushed.
    #[test]
    fn positions_round_trip() {
        for sign in [1i8, -1] {
            for offset in [-300i16, 0, 57] {
                for raw in [0u16, 1, 1000, 2048, 3333, 4095] {
                    let rad = counts_to_rad(raw, sign, offset);
                    assert_eq!(
                        rad_to_counts(rad, sign, offset),
                        raw,
                        "{sign} {offset} {raw}"
                    );
                }
            }
        }
    }

    /// A target past the end of travel is commanded as the end of travel. Wrapping would send
    /// the joint the long way round.
    #[test]
    fn targets_clamp_to_the_single_turn_range() {
        assert_eq!(rad_to_counts(10.0, 1, 0), 4095);
        assert_eq!(rad_to_counts(-10.0, 1, 0), 0);
        assert_eq!(rad_to_counts(f64::NAN, 1, 0), 2048);
    }

    #[test]
    fn a_block_decodes_field_by_field() {
        let mut block = [0u8; READ_LEN as usize];
        block[0..2].copy_from_slice(&3072u16.to_le_bytes()); // position
        block[2..4].copy_from_slice(&(0x8000u16 | 100).to_le_bytes()); // speed: −73.2 rpm
        block[6] = 112; // 11.2 V
        block[7] = 35; // °C
        block[9] = 0x20; // overload
        block[13..15].copy_from_slice(&(0x8000u16 | 100).to_le_bytes()); // −650 mA
        let s = decode_block(&block, 1, 0);
        assert!((s.position - PI / 2.0).abs() < 1e-12);
        assert!((s.velocity + 73.2 * 2.0 * PI / 60.0).abs() < 1e-9);
        assert!((s.volts - 11.2).abs() < 1e-12);
        assert_eq!(s.temp_c, 35.0);
        assert_eq!(s.status, 0x20);
        assert!((s.current_ma - 650.0).abs() < 1e-12, "magnitude only");
    }

    /// The measured unit: 20 counts of present speed is ~1000 encoder counts a second.
    #[test]
    fn present_speed_is_in_goal_speed_units() {
        let rad_s = 20.0 * RAD_PER_SEC_PER_COUNT;
        let counts_s = rad_s * COUNTS_PER_REV / (2.0 * PI);
        assert!((counts_s - 999.4).abs() < 1.0, "{counts_s}");
    }

    /// Velocity must follow the joint's sign, or a reversed joint reports motion backwards.
    #[test]
    fn velocity_follows_the_joint_sign() {
        let mut block = [0u8; READ_LEN as usize];
        block[0..2].copy_from_slice(&2048u16.to_le_bytes());
        block[2..4].copy_from_slice(&1024u16.to_le_bytes());
        assert!(decode_block(&block, -1, 0).velocity < 0.0);
    }

    /// The block has to reach the current register at 69..70 and hold the whole IMU block.
    #[test]
    fn the_read_block_covers_current_and_the_imu() {
        assert_eq!(READ_ADDR as usize + OFF_CURRENT, 69);
        assert!(OFF_CURRENT + 2 <= READ_LEN as usize);
        assert!(IMU_BLOCK_LEN <= READ_LEN as usize);
    }

    /// The running gain maps to the configured Kp, and the safety layer's ratios survive:
    /// limp is a quarter of running, as on the XL330.
    #[test]
    fn gain_scales_from_the_reference() {
        assert_eq!(kp_for_gain(KP_REFERENCE as u16, 8), 8);
        assert_eq!(kp_for_gain(50, 8), 2);
        assert_eq!(kp_for_gain(160, 8), 6, "standing at 0.8 of running");
        assert_eq!(
            kp_for_gain(0, 8),
            1,
            "never zero: a zero Kp is a servo that will not hold"
        );
        assert_eq!(kp_for_gain(KP_REFERENCE as u16, 32), 32);
    }

    #[test]
    fn a_zero_kp_is_refused() {
        let mut c = config();
        c.position_kp[4] = 0;
        assert!(c.validate().is_err());
    }

    #[test]
    fn current_limit_converts_to_counts() {
        assert_eq!(config().current_limit_counts(), 450);
    }
}
