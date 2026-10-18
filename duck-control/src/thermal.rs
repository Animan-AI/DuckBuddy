//! Servo temperature guard: warn, stop walking, and optionally sit down before a servo cuts
//! its own torque.
//!
//! The servos latch an overheat shutdown on their own (the HL-2915s on the robot read register 13
//! = 80 °C, with temperature in their unload conditions), and a servo that cuts torque mid-stride
//! drops the robot. On the HL-2915 robot (2026-10-10) one
//! walking trial — a minute of standing plus 4–8 s of walking — warmed the hottest servo by
//! 6–8 °C, and two or three back to back reached 58–61 °C; resting with torque off brought
//! 61 °C back to 48 °C in 1.5 minutes. So the guard acts on the hottest servo, once per thermal
//! sample, with hysteresis so a reading hovering on a threshold does not flap.

/// Something the caller should act on or report, from one [`ThermalGuard::update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThermalEvent {
    /// The hottest servo reached the warning threshold.
    Warn,
    /// It reached the walking limit: walking is refused until it cools.
    WalkBlocked,
    /// It cooled below the walking limit by the hysteresis: walking is allowed again.
    WalkCleared,
    /// It reached the sit threshold: sit down (once per crossing).
    Sit,
}

#[derive(Debug, Clone)]
pub struct ThermalGuard {
    warn_c: f64,
    walk_max_c: f64,
    sit_c: f64,
    hysteresis_c: f64,
    warned: bool,
    walk_blocked: bool,
    sit_armed: bool,
}

impl ThermalGuard {
    /// Thresholds in °C; `0` turns that action off. `hysteresis_c` is how far below a
    /// threshold the hottest servo must cool before the threshold clears.
    pub fn new(warn_c: f64, walk_max_c: f64, sit_c: f64, hysteresis_c: f64) -> Self {
        Self {
            warn_c,
            walk_max_c,
            sit_c,
            hysteresis_c: hysteresis_c.max(0.0),
            warned: false,
            walk_blocked: false,
            sit_armed: true,
        }
    }

    /// Whether walking is refused right now.
    pub fn walk_blocked(&self) -> bool {
        self.walk_blocked
    }

    /// Whether the hottest servo is still within the hysteresis of the sit threshold — a sit
    /// asked for but not yet carried out is stale once this goes false.
    pub fn sit_hot(&self) -> bool {
        self.sit_c > 0.0 && !self.sit_armed
    }

    /// One thermal sample: the hottest servo's temperature. Returns what changed.
    pub fn update(&mut self, max_c: f64) -> Vec<ThermalEvent> {
        let mut events = Vec::new();
        if !max_c.is_finite() {
            return events;
        }
        let on = |threshold: f64| threshold > 0.0;
        let cooled = |threshold: f64| max_c < threshold - self.hysteresis_c;

        if on(self.warn_c) {
            if !self.warned && max_c >= self.warn_c {
                self.warned = true;
                events.push(ThermalEvent::Warn);
            } else if self.warned && cooled(self.warn_c) {
                self.warned = false;
            }
        }
        if on(self.walk_max_c) {
            if !self.walk_blocked && max_c >= self.walk_max_c {
                self.walk_blocked = true;
                events.push(ThermalEvent::WalkBlocked);
            } else if self.walk_blocked && cooled(self.walk_max_c) {
                self.walk_blocked = false;
                events.push(ThermalEvent::WalkCleared);
            }
        } else {
            self.walk_blocked = false;
        }
        if on(self.sit_c) {
            if self.sit_armed && max_c >= self.sit_c {
                self.sit_armed = false;
                events.push(ThermalEvent::Sit);
            } else if !self.sit_armed && cooled(self.sit_c) {
                self.sit_armed = true;
            }
        }
        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn guard() -> ThermalGuard {
        ThermalGuard::new(55.0, 65.0, 60.0, 5.0)
    }

    #[test]
    fn cool_servos_raise_nothing() {
        let mut g = guard();
        assert!(g.update(40.0).is_empty());
        assert!(!g.walk_blocked());
    }

    /// Each threshold fires once on the way up, in the order they are reached.
    #[test]
    fn heating_warns_then_sits_then_stops_the_walk() {
        let mut g = guard();
        assert_eq!(g.update(55.0), vec![ThermalEvent::Warn]);
        assert!(g.update(57.0).is_empty());
        assert_eq!(g.update(61.0), vec![ThermalEvent::Sit]);
        assert!(g.update(62.0).is_empty(), "the sit is asked for once");
        assert_eq!(g.update(65.0), vec![ThermalEvent::WalkBlocked]);
        assert!(g.walk_blocked());
    }

    /// A reading hovering on the limit does not flap the walk on and off: it clears only
    /// once the servo is the hysteresis below it.
    #[test]
    fn the_walk_limit_clears_with_hysteresis() {
        let mut g = guard();
        g.update(66.0);
        assert!(g.update(64.0).is_empty());
        assert!(g.walk_blocked());
        assert!(g.update(60.5).is_empty());
        assert_eq!(g.update(59.9), vec![ThermalEvent::WalkCleared]);
        assert!(!g.walk_blocked());
    }

    /// A sample can cross several thresholds at once — the first sample after a long rest
    /// read on a robot that was already hot, say.
    #[test]
    fn one_sample_can_cross_everything() {
        let mut g = guard();
        assert_eq!(
            g.update(70.0),
            vec![
                ThermalEvent::Warn,
                ThermalEvent::WalkBlocked,
                ThermalEvent::Sit
            ]
        );
    }

    #[test]
    fn the_sit_rearms_only_after_cooling() {
        let mut g = guard();
        assert!(!g.sit_hot());
        assert_eq!(g.update(60.0), vec![ThermalEvent::Warn, ThermalEvent::Sit]);
        assert!(
            g.update(56.0).is_empty(),
            "56 is within the hysteresis of 60"
        );
        assert!(g.sit_hot());
        g.update(54.0);
        assert!(!g.sit_hot());
        assert_eq!(g.update(60.0), vec![ThermalEvent::Sit]);
    }

    #[test]
    fn zero_turns_an_action_off() {
        let mut g = ThermalGuard::new(0.0, 0.0, 0.0, 5.0);
        assert!(g.update(90.0).is_empty());
        assert!(!g.walk_blocked());
    }

    #[test]
    fn a_reading_that_is_not_a_number_changes_nothing() {
        let mut g = guard();
        g.update(66.0);
        assert!(g.update(f64::NAN).is_empty());
        assert!(g.walk_blocked());
    }
}
