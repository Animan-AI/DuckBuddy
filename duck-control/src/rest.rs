//! Resting to cool down: sit, let the legs go, stand again once cool.
//!
//! A servo that holds anything heats ~13 °C a minute however light the load, and cools ~4 °C a
//! minute once it holds nothing (HL-2915 robot, 2026-10-12). Sitting alone does not help — the
//! sitstand policy holds the sit with the knees folded, and the legs warmed 58 → 61 °C in a
//! minute of it (2026-10-11). What cools is a seated robot whose legs hold nothing: the trunk
//! rests on the floor, the leg servos have their torque off, the head can stay alive.
//!
//! Pure: the control loop feeds it what it sees each tick and carries out the [`RestAction`]s
//! it returns (sit toggle on the controller, limp mask on the bus). No timers of its own, no IO.

/// Why the robot is resting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestReason {
    /// The hottest servo reached the rest threshold.
    Hot,
    /// A client asked (`robot.rest`).
    Asked,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RestPhase {
    Awake,
    /// Asked to rest; waiting for the robot to be standing still to sit.
    Pending,
    /// Sitting down / settling before the legs let go.
    Sitting {
        settled_s: f64,
    },
    /// Seated, legs limp.
    Resting {
        elapsed_s: f64,
    },
    /// Powered again, the sit being restored before standing: seconds since, and seconds seated
    /// and still so far.
    Bracing {
        elapsed_s: f64,
        settled_s: f64,
    },
}

/// What the control loop should do this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RestAction {
    /// Toggle the sitstand policy into the sit.
    Sit,
    /// Cut (true) or restore (false) the leg servos' torque.
    LegsLimp(bool),
    /// Get up. `settled`: the sit was restored (seated and still for `brace_s`), so the sitstand
    /// rise can take it; otherwise it gave up waiting after `brace_max_s`, and the caller should
    /// get up some sturdier way.
    Stand { settled: bool },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RestConfig {
    /// Stand again once the hottest servo is at or below this, °C.
    pub resume_c: f64,
    /// Seated and still this long before the legs let go, s.
    pub settle_s: f64,
    /// Rest at least this long once limp, s — a reading bouncing on `resume_c` must not make
    /// the robot bob up and down.
    pub min_rest_s: f64,
    /// Powered again, seated and still this long before standing, s. Let go entirely (the head
    /// too), the robot comes back leaning and moving; rising before the sit was restored threw it
    /// on its back (2026-10-14) — first sit properly, then stand (the user).
    pub brace_s: f64,
    /// Give up waiting for the sit to be restored after this long, s.
    pub brace_max_s: f64,
    /// Cooling rate used for the time-left estimate, °C per second (measured ~4 °C/min).
    pub cool_c_per_s: f64,
}

impl Default for RestConfig {
    fn default() -> Self {
        Self {
            resume_c: 45.0,
            settle_s: 1.0,
            min_rest_s: 30.0,
            brace_s: 1.0,
            brace_max_s: 8.0,
            cool_c_per_s: 4.0 / 60.0,
        }
    }
}

/// What the loop sees, once per tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RestInputs {
    pub dt: f64,
    /// The policy is driving an upright robot that may be told to sit: enabled, ready, not
    /// fallen, no scripted move running, not walking.
    pub can_sit: bool,
    /// The sitstand policy holds the robot in the sit (not rising, not standing).
    pub seated: bool,
    /// Trunk still and level enough to let the legs go.
    pub still: bool,
    /// Hottest servo, °C, if known.
    pub max_c: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct Rest {
    config: RestConfig,
    phase: RestPhase,
    reason: Option<RestReason>,
    /// A client asked to wake; honoured once cool enough.
    wake_asked: bool,
}

impl Rest {
    pub fn new(config: RestConfig) -> Self {
        Self {
            config,
            phase: RestPhase::Awake,
            reason: None,
            wake_asked: false,
        }
    }

    pub fn phase(&self) -> RestPhase {
        self.phase
    }

    pub fn reason(&self) -> Option<RestReason> {
        self.reason
    }

    /// Resting in any form — the twist should be held at zero.
    pub fn active(&self) -> bool {
        self.phase != RestPhase::Awake
    }

    /// Ask to rest. Idempotent; a thermal reason overrides an asked one (it cannot be woken early).
    pub fn request(&mut self, reason: RestReason) {
        if self.phase == RestPhase::Awake {
            self.phase = RestPhase::Pending;
        }
        if self.reason != Some(RestReason::Hot) {
            self.reason = Some(reason);
        }
        self.wake_asked = false;
    }

    /// Ask to get up. A rest the robot asked for itself (too hot) still waits for `resume_c`.
    pub fn wake(&mut self) {
        match self.phase {
            RestPhase::Awake => {}
            // Not seated yet: simply don't.
            RestPhase::Pending => self.abort(),
            _ => self.wake_asked = true,
        }
    }

    /// The robot was taken out of its rest by something else (fall, relax, shutdown): forget it.
    /// The caller restores the legs' torque mask itself.
    pub fn abort(&mut self) {
        self.phase = RestPhase::Awake;
        self.reason = None;
        self.wake_asked = false;
    }

    fn cool(&self, max_c: Option<f64>) -> bool {
        max_c.is_some_and(|c| c <= self.config.resume_c)
    }

    /// Seconds until the servos should be cool enough to stand, from the measured cooling rate.
    pub fn eta_s(&self, max_c: Option<f64>) -> Option<f64> {
        if !self.active() {
            return None;
        }
        let to_cool = max_c.map_or(0.0, |c| {
            ((c - self.config.resume_c) / self.config.cool_c_per_s).max(0.0)
        });
        let min_left = match self.phase {
            RestPhase::Resting { elapsed_s } => (self.config.min_rest_s - elapsed_s).max(0.0),
            RestPhase::Bracing { .. } => 0.0,
            _ => self.config.min_rest_s,
        };
        Some(to_cool.max(min_left))
    }

    pub fn tick(&mut self, input: RestInputs) -> Option<RestAction> {
        let RestInputs {
            dt,
            can_sit,
            seated,
            still,
            max_c,
        } = input;
        match self.phase {
            RestPhase::Awake => None,
            RestPhase::Pending => {
                if seated {
                    self.phase = RestPhase::Sitting { settled_s: 0.0 };
                    None
                } else if can_sit {
                    self.phase = RestPhase::Sitting { settled_s: 0.0 };
                    Some(RestAction::Sit)
                } else {
                    None
                }
            }
            RestPhase::Sitting { settled_s } => {
                let settled_s = if seated && still { settled_s + dt } else { 0.0 };
                if settled_s >= self.config.settle_s {
                    self.phase = RestPhase::Resting { elapsed_s: 0.0 };
                    Some(RestAction::LegsLimp(true))
                } else {
                    self.phase = RestPhase::Sitting { settled_s };
                    None
                }
            }
            RestPhase::Resting { elapsed_s } => {
                let elapsed_s = elapsed_s + dt;
                let rested = elapsed_s >= self.config.min_rest_s;
                let hot_done = self.reason == Some(RestReason::Hot) && rested && self.cool(max_c);
                let woken = self.wake_asked
                    && (self.reason != Some(RestReason::Hot) || (rested && self.cool(max_c)));
                if hot_done || woken {
                    self.phase = RestPhase::Bracing {
                        elapsed_s: 0.0,
                        settled_s: 0.0,
                    };
                    Some(RestAction::LegsLimp(false))
                } else {
                    self.phase = RestPhase::Resting { elapsed_s };
                    None
                }
            }
            RestPhase::Bracing {
                elapsed_s,
                settled_s,
            } => {
                let elapsed_s = elapsed_s + dt;
                let settled_s = if seated && still { settled_s + dt } else { 0.0 };
                if settled_s >= self.config.brace_s {
                    self.abort();
                    Some(RestAction::Stand { settled: true })
                } else if elapsed_s >= self.config.brace_max_s {
                    self.abort();
                    Some(RestAction::Stand { settled: false })
                } else {
                    self.phase = RestPhase::Bracing {
                        elapsed_s,
                        settled_s,
                    };
                    None
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.02;

    fn run(rest: &mut Rest, secs: f64, input: RestInputs) -> Vec<RestAction> {
        (0..(secs / DT).round() as usize)
            .filter_map(|_| rest.tick(input))
            .collect()
    }

    fn standing(max_c: f64) -> RestInputs {
        RestInputs {
            dt: DT,
            can_sit: true,
            seated: false,
            still: true,
            max_c: Some(max_c),
        }
    }

    fn seated(max_c: f64) -> RestInputs {
        RestInputs {
            dt: DT,
            can_sit: false,
            seated: true,
            still: true,
            max_c: Some(max_c),
        }
    }

    #[test]
    fn a_hot_rest_sits_lets_go_and_stands_once_cool() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Hot);
        assert_eq!(run(&mut rest, 0.1, standing(66.0)), vec![RestAction::Sit]);
        assert_eq!(
            run(&mut rest, 1.2, seated(65.0)),
            vec![RestAction::LegsLimp(true)]
        );
        // still hot after the minimum rest: keeps resting
        assert!(run(&mut rest, 60.0, seated(50.0)).is_empty());
        assert!(rest.active());
        // cool: legs back, then stand
        assert_eq!(
            run(&mut rest, 0.1, seated(44.0)),
            vec![RestAction::LegsLimp(false)]
        );
        assert_eq!(
            run(&mut rest, 1.2, seated(44.0)),
            vec![RestAction::Stand { settled: true }]
        );
        assert!(!rest.active());
    }

    #[test]
    fn it_waits_for_the_robot_to_be_still_before_sitting_and_letting_go() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Hot);
        let walking = RestInputs {
            can_sit: false,
            ..standing(66.0)
        };
        assert!(run(&mut rest, 2.0, walking).is_empty());
        assert_eq!(run(&mut rest, 0.1, standing(66.0)), vec![RestAction::Sit]);
        // seated but rocking: the settle clock keeps restarting
        let rocking = RestInputs {
            still: false,
            ..seated(66.0)
        };
        assert!(run(&mut rest, 3.0, rocking).is_empty());
        assert_eq!(
            run(&mut rest, 1.2, seated(66.0)),
            vec![RestAction::LegsLimp(true)]
        );
    }

    #[test]
    fn a_hot_rest_cannot_be_woken_early() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Hot);
        run(&mut rest, 0.1, standing(66.0));
        run(&mut rest, 1.2, seated(66.0));
        rest.wake();
        assert!(
            run(&mut rest, 40.0, seated(60.0)).is_empty(),
            "still too hot"
        );
        assert_eq!(
            run(&mut rest, 0.1, seated(45.0)),
            vec![RestAction::LegsLimp(false)]
        );
    }

    #[test]
    fn an_asked_rest_wakes_when_asked() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Asked);
        run(&mut rest, 0.1, standing(50.0));
        run(&mut rest, 1.2, seated(50.0));
        assert!(
            run(&mut rest, 5.0, seated(50.0)).is_empty(),
            "rests until woken"
        );
        rest.wake();
        assert_eq!(
            run(&mut rest, 0.1, seated(50.0)),
            vec![RestAction::LegsLimp(false)]
        );
        assert_eq!(
            run(&mut rest, 1.2, seated(50.0)),
            vec![RestAction::Stand { settled: true }]
        );
    }

    #[test]
    fn waking_before_sitting_cancels_and_already_seated_skips_the_sit() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Asked);
        rest.wake();
        assert!(!rest.active());
        rest.request(RestReason::Asked);
        assert!(
            run(&mut rest, 0.1, seated(50.0)).is_empty(),
            "already sitting: no toggle"
        );
        assert_eq!(
            run(&mut rest, 1.2, seated(50.0)),
            vec![RestAction::LegsLimp(true)]
        );
    }

    #[test]
    fn woken_it_restores_the_sit_before_standing_and_gives_up_waiting_eventually() {
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Asked);
        run(&mut rest, 0.1, standing(50.0));
        run(&mut rest, 1.2, seated(50.0));
        rest.wake();
        assert_eq!(
            run(&mut rest, 0.1, seated(50.0)),
            vec![RestAction::LegsLimp(false)]
        );
        // powered again but still settling: no rise yet
        let moving = RestInputs {
            still: false,
            ..seated(50.0)
        };
        assert!(
            run(&mut rest, 3.0, moving).is_empty(),
            "not before the sit is restored"
        );
        assert_eq!(
            run(&mut rest, 1.2, seated(50.0)),
            vec![RestAction::Stand { settled: true }]
        );
        // never settles: stands anyway, flagged, after brace_max_s
        let mut rest = Rest::new(RestConfig::default());
        rest.request(RestReason::Asked);
        run(&mut rest, 0.1, standing(50.0));
        run(&mut rest, 1.2, seated(50.0));
        rest.wake();
        run(&mut rest, 0.1, seated(50.0));
        let acts = run(
            &mut rest,
            9.0,
            RestInputs {
                seated: false,
                ..seated(50.0)
            },
        );
        assert_eq!(acts, vec![RestAction::Stand { settled: false }]);
    }

    #[test]
    fn the_eta_counts_down_to_cool_and_the_minimum_rest() {
        let mut rest = Rest::new(RestConfig::default());
        assert_eq!(rest.eta_s(Some(60.0)), None);
        rest.request(RestReason::Hot);
        let eta = rest.eta_s(Some(57.0)).unwrap();
        assert!((eta - 180.0).abs() < 1e-6, "12 C at 4 C/min, {eta}");
        assert_eq!(
            rest.eta_s(Some(44.0)),
            Some(30.0),
            "cool already: the minimum rest"
        );
    }
}
