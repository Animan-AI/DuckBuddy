//! The robot control core: everything between reading the bus and writing it.
//!
//! Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
//! owns all of that. The boundary is enforced by the compiler rather than by discipline,
//! which is what stops process concerns leaking into the code that drives motors.
//!
//! The control path it holds — model, bus, [`io::RobotIo`], observations, policy, safety — is
//! designed in `docs/design/robotd-design.md` §2.

pub mod bus;
pub mod fall;
pub mod feetech;
pub mod heading;
pub mod imu;
pub mod io;
pub mod model;
pub mod obs;
pub mod policy;
pub mod rest;
pub mod safety;
pub mod servo_bus;
pub mod thermal;

pub use imu::ImuData;
pub use io::{FakeIo, IoError, JointTargets, RobotIo, Sensors, SlowSensors};
pub use model::{
    Chemistry, DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, LEG_JOINTS, LIE_GRAVITY, LIE_POSITION,
    NUM_JOINTS, Pack, SIT_POSITION, identify_pack, leg_distance,
};
pub use obs::{ACTION_LEN, Command, OBS_LEN, Observation};
