# -*- coding: utf-8 -*-
"""BAM actuator model for the Feetech HL-2915 (HLS memory map) as the rev-a robot drives it.

BAM ships an STS3215 class; the HL-2915 differs in supply (3S, 9.9–13.3 V), gearing and
firmware, and the robot runs it with a goal-current cap (register 44) and a fixed Kd
(register 51 = 8). Modelled as BAM's voltage-controlled servo:

    duty = clip(kp * (q_target - q) * error_gain * error_gain_ratio, current window, ±max_pwm)
    tau  = kt * V / R - kt² * dq / R          (+ the BAM model's friction terms)

* ``kp`` is the live register-50 value, read from each log (8 / 16 / 24 / 32).
* ``error_gain`` is a first guess from the bench (Kp 16 held 0.1 N·m at ~5° of error);
  ``error_gain_ratio`` is fitted, as BAM does for the STS3215.
* The Kd the robot runs with is the same in every log, so its damping is absorbed by the
  fitted viscous friction: the fit is only valid at register 51 = 8.
* ``max_current`` is the register-44 cap, taken from the log (1.3 A on the robot).

Register it before loading models or fitting::

    import hl2915  # noqa: F401  (adds "hl2915" to bam.actuators.actuators)
"""
from __future__ import annotations

import numpy as np

from bam.actuator import VoltageControlledActuator
from bam.actuators import actuators
from bam.parameter import Parameter
from bam.testbench import Pendulum

# Fitted error gain is error_gain * error_gain_ratio; see the module docstring.
ERROR_GAIN = 0.05
# The robot's register-44 cap.
DEFAULT_MAX_CURRENT = 1.3


class HL2915Actuator(VoltageControlledActuator):
    def __init__(self, testbench_class=Pendulum):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=16,
            error_gain=ERROR_GAIN,
            max_pwm=1.0,
            max_current=DEFAULT_MAX_CURRENT,
        )

    def load_log(self, log: dict):
        super().load_log(log)
        if "current_limit_ma" in log:
            self.max_current = log["current_limit_ma"] / 1000.0

    def initialize(self):
        # Datasheet: 1.39 N·m stall / 1.5 A at 12 V, 0.91 N·m/A measured on the robot.
        self.model.kt = Parameter(0.9, 0.3, 3.0)
        self.model.R = Parameter(8.0, 2.0, 20.0)
        self.model.armature = Parameter(0.005, 0.0001, 0.05)
        self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value

    def compute_control(self, q_target, q, dq, dt):
        duty = (q_target - q) * self.kp * self.error_gain * self.model.error_gain_ratio.value
        if self.max_current is not None:
            back_emf = self.model.kt.value * dq
            span = self.model.R.value * self.max_current / self.vin
            centre = back_emf / self.vin
            duty = self.backend.clamp(duty, centre - span, centre + span)
        duty = self.backend.clamp(duty, -self.max_pwm, self.max_pwm)
        return self.vin * duty

    def to_mujoco(self):
        # The base class's linearisation, with the fitted gain ratio folded in.
        saved = self.error_gain
        self.error_gain = saved * self.model.error_gain_ratio.value
        try:
            super().to_mujoco()
        finally:
            self.error_gain = saved


actuators["hl2915"] = lambda: HL2915Actuator(Pendulum)
