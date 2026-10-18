//! The robot's servo bus, whichever servos it carries.
//!
//! The original alpha runs Dynamixel XL330s ([`crate::bus`]); this build runs Feetech HL-2915s
//! ([`crate::feetech`]). The control loop does not care which — it talks to [`RobotIo`] — but
//! `robotd` has to open one or the other from its configuration, and `init` needs the handful
//! of bring-up operations that are not on the trait. This enum is that choice, made once.

use std::time::Duration;

use crate::bus::DynamixelIo;
use crate::feetech::{FeetechConfig, FeetechIo};
use crate::io::{ImuStale, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::NUM_JOINTS;

/// What to open.
#[derive(Debug, Clone, PartialEq)]
pub enum BusConfig {
    /// XL330s on protocol 2.0 at 1 Mbit/s, IDs from [`crate::model::JOINT_IDS`].
    Dynamixel,
    Feetech(FeetechConfig),
}

// One per process, never moved after opening: a few hundred bytes of padding cost nothing.
#[allow(clippy::large_enum_variant)]
pub enum ServoBus {
    Dynamixel(DynamixelIo),
    Feetech(FeetechIo),
}

macro_rules! each {
    ($self:expr, $io:ident => $body:expr) => {
        match $self {
            ServoBus::Dynamixel($io) => $body,
            ServoBus::Feetech($io) => $body,
        }
    };
}

impl ServoBus {
    pub fn open(port: &str, config: &BusConfig) -> Result<Self> {
        Ok(match config {
            BusConfig::Dynamixel => ServoBus::Dynamixel(DynamixelIo::open(port)?),
            BusConfig::Feetech(c) => ServoBus::Feetech(FeetechIo::open(port, c.clone())?),
        })
    }

    /// Verify (and where the backend does so, correct) the registers the loop depends on.
    /// Returns how many were corrected.
    pub fn check_registers(&mut self) -> Result<usize> {
        each!(self, io => io.check_registers())
    }

    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        each!(self, io => io.present_positions())
    }

    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        each!(self, io => io.set_torque(on))
    }

    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        each!(self, io => io.interpolate_to(target, duration, step))
    }
}

impl RobotIo for ServoBus {
    fn read(&mut self) -> Result<Sensors> {
        each!(self, io => io.read())
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        each!(self, io => RobotIo::write(io, targets))
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        each!(self, io => io.set_gain(kp))
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        each!(self, io => RobotIo::set_torque(io, on))
    }

    fn set_limp(&mut self, limp: [bool; NUM_JOINTS]) -> Result<()> {
        each!(self, io => RobotIo::set_limp(io, limp))
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        each!(self, io => io.slow_sensors())
    }

    fn imu_stale(&self) -> ImuStale {
        each!(self, io => io.imu_stale())
    }

    fn imu_ready(&self) -> bool {
        each!(self, io => io.imu_ready())
    }
}
