"""HL-2915 (rev-a) variants of the MicroDuck tasks.

Installed into microduck_rl as `src/mjlab_microduck/tasks/hl2915.py` and imported at the
end of `tasks/__init__.py`. Each task is the stock one with the robot entity swapped for
the rev-a model + HL-2915 BAM actuator (see `robot/hl2915_constants.py`), DR widened for
what the rev-a robot does not pin down, robotd's target low-pass applied in the action
term, and a few task fixes found on the robot (below). Observations and action layout are
untouched, so the exported ONNX is still obs[1,61] → actions[1,14]. Ids insert "-HL2915"
before "-MicroDuck".
"""

from dataclasses import dataclass, fields, replace

import torch
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers import CurriculumTermCfg, EventTermCfg, RewardTermCfg
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.registry import register_mjlab_task

from mjlab_microduck.robot.hl2915_constants import (
    HL2915_BACKLASH_ROBOT_CFG,
    HL2915_STANDUP_ROBOT_CFG,
    HL2915_WALK_BACKLASH_ROBOT_CFG,
    HL2915_WALK_ROBOT_CFG,
)

from . import mdp as microduck_mdp
from .symmetry import SYMMETRY_CFG
from . import (
    MicroduckGroundPickRlCfg,
    MicroduckOnPolicyRunner,
    MicroduckRlCfg,
    MicroduckRouladeRlCfg,
    MicroduckSitStandRlCfg,
    MicroduckStandUpRlCfg,
    MicroduckVelStandRlCfg,
    make_backlash_variant,
    make_microduck_ground_pick_env_cfg,
    make_microduck_roulade_env_cfg,
    make_microduck_sitstand_env_cfg,
    make_microduck_standup_env_cfg,
    make_microduck_velocity_env_cfg,
    make_microduck_velstand_env_cfg,
)


import math
import re

# Domain randomization widened for what the rev-a robot does not pin down (applied on top
# of the stock task's DR; the command delay lives in the actuator cfg):
#   friction — one servo was benched, gearboxes vary;
#   encoder bias — zeros were hand-posed, ±2–3° (stock ±0.86° assumes a jig);
#   mass — only the total was weighed, so per-link masses are a uniform scale (±10%);
#   armature — fitted on one servo.
HL2915_DR = {
    "encoder_bias": ("bias_range", (-0.04, 0.04)),
    "randomize_mass_inertia": ("alpha_range", (0.5 * math.log(0.90), 0.5 * math.log(1.10))),
    "randomize_armature": ("ranges", (0.8, 1.2)),
}


# Per-joint actuator spread (see `PerJointBamActuator`), re-drawn each episode. Gain is
# skewed weak: on the robot the hip pitch servos delivered 0.6-0.8 of the model's amplitude
# under walking load, and only with every leg joint at 0.4-0.5 of the fitted gain does the
# simulation reproduce what the robot did (lean forward, throw the head forward and down,
# fall) — the bench fit only saw loads up to 0.18 N·m. Low gain is learnable: the XL330
# policies walked at the equivalent of 0.26. Friction is wide: one servo was benched.
JOINT_KP_RANGE = (0.4, 1.1)
JOINT_FRICTION_RANGE = (0.7, 2.0)

# Head mass, scaled with its inertia (pseudo-inertia, CoM kept). Centred on the model since
# the head was weighed (2026-10-07): 257.1 g off the head-pitch servo against the model's
# 258.0 g, CoM within 1 mm. Was skewed heavy (0.95-1.30) on the guess that the rev-a
# electronics made it heavier.
HEAD_BODIES = ("neck", "neck_pitch", "yaw_roll_motion", "jaw_soft")
#
# v4 puts back v2's heavy skew (0.95-1.30, with the +5 mm CoM in hl2915_constants) although
# the weighed head matches the model: v3 changed this and the low-pass together and walked
# worse, so v4 changes only the backlash against the v2bl recipe that walked best.
HEAD_MASS_RANGE = (0.95, 1.30)


def randomize_joint_actuators(env, env_ids, kp_range, friction_range, kp_base=None,
                              asset_cfg=SceneEntityCfg("robot")):
    """Fresh per-joint gain and friction scales for these envs (non-accumulating: set, not
    multiplied). Actuators without per-joint scales are left alone.

    `kp_base` ({joint-name regex: factor}) fixes a joint's gain below the actuator's kp_fw
    before the draw: BAM's firmware law is linear in Kp (fitted stiffness 1.04 / 2.07 / 3.11 /
    4.14 N·m/rad at Kp 8 / 16 / 24 / 32), so 0.5 on a joint trained at kp_fw 16 is Kp 8."""
    import re
    from mjlab_microduck.robot.hl2915_constants import (
        PerJointBacklashBamActuator,
        PerJointBamActuator,
    )

    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    for actuator in asset.actuators:
        if not isinstance(actuator, (PerJointBamActuator, PerJointBacklashBamActuator)):
            continue
        n, j = len(env_ids), len(actuator.ctrl_ids)

        def draw(lo_hi):
            lo, hi = lo_hi
            return torch.rand(n, j, device=env.device) * (hi - lo) + lo

        kp = draw(kp_range)
        for pattern, factor in (kp_base or {}).items():
            for i, name in enumerate(actuator._target_names):
                if re.match(pattern, name):
                    kp[:, i] *= factor
        actuator.kp_scale[env_ids] = kp
        actuator.friction_scale[env_ids] = draw(friction_range)


def _widen_dr(cfg):
    for event, (param, value) in HL2915_DR.items():
        term = cfg.events.get(event)
        if term is not None:
            term.params[param] = value
    # Per-joint draws replace the stock per-env friction scale (both write friction_scale).
    cfg.events.pop("randomize_joint_friction", None)
    cfg.events["randomize_joint_actuators"] = EventTermCfg(
        func=randomize_joint_actuators,
        mode="reset",
        params={"kp_range": JOINT_KP_RANGE, "friction_range": JOINT_FRICTION_RANGE},
    )
    cfg.events["randomize_head_mass"] = EventTermCfg(
        func=dr.pseudo_inertia,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=HEAD_BODIES),
            "alpha_range": tuple(0.5 * math.log(x) for x in HEAD_MASS_RANGE),
        },
    )
    return cfg


# Trunk height (terrain-relative) below which feet air time pays nothing: VelStand's
# TERM_GATE_Z — under every wobbling stand measured (≥0.084), over a sit.
AIR_TIME_GATE_Z = 0.08


def feet_air_time_standing(env, gate_tilt_above_deg=40.0, gate_z_below=AIR_TIME_GATE_Z,
                           asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG, **air_time_kwargs):
    """`feet_air_time_upright`, also zeroed while the trunk is low.

    The stock term gates on tilt only, so a robot SITTING — trunk upright, z ≈ 0.047 on this
    model — kicks its free legs through the air-time window and is paid for it. Measured in
    the VelStand-HL2915 sim: the run-1 policy, sat down, collected 136/s of air time, which
    put sitting (349/s total) level with standing perfectly still (356/s) and well above a
    policy still learning to walk. It sat down by iteration 500 and never got up. Paying air
    time only in the standing envelope removes that without paying for any fallen state.
    """
    reward = microduck_mdp.feet_air_time_upright(
        env, gate_tilt_above_deg=gate_tilt_above_deg, asset_cfg=asset_cfg, **air_time_kwargs)
    asset = env.scene[asset_cfg.name]
    return reward * (1.0 - microduck_mdp._fallen_mask(env, asset, gate_z_below, gate_tilt_above_deg))


def _gate_air_time(cfg):
    term = cfg.rewards.get("air_time")
    if term is not None and term.func is microduck_mdp.feet_air_time_upright:
        term.func = feet_air_time_standing
    return cfg


# robotd's target low-pass (`[policy] head_lowpass` / `legs_lowpass`), applied here exactly
# as robotd applies it: target = a * new + (1 - a) * previous target, per control tick, the
# first tick after a reset passing through. On the robot it is not optional — without it a
# policy standing still shook 50x harder (2026-10-07: target steps 0.5-1.6° per tick against
# 0.02 with it), the servo readings being far noisier than the simulation's. Trained without
# it, the filter is lag the policy never saw; trained with it, both sides run the same loop.
HEAD_JOINT_NAMES = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
LOWPASS_HEAD = 0.5
LOWPASS_LEGS = 0.7
# Off since v4: on the robot (2026-10-08, one floor, one hour) the policies trained with the
# filter walked worse than those without, in both backlash arms of the 2x2 — v3bl 2.5 s
# against v2bl's full 4 s, v3 1.2 s against v2's 2.4 s — while robotd keeps it on.
TRAIN_WITH_LOWPASS = False


class LowPassJointPositionAction(JointPositionAction):
    def __init__(self, cfg, env):
        super().__init__(cfg=cfg, env=env)
        alpha = torch.full((self.action_dim,), cfg.legs_alpha, device=self.device)
        for i, name in enumerate(self._target_names):
            if name in HEAD_JOINT_NAMES:
                alpha[i] = cfg.head_alpha
        self._alpha = alpha
        self._previous = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._fresh = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)

    def process_actions(self, actions):
        super().process_actions(actions)
        filtered = self._alpha * self._processed_actions + (1.0 - self._alpha) * self._previous
        self._processed_actions = torch.where(self._fresh[:, None], self._processed_actions, filtered)
        self._previous = self._processed_actions.clone()
        self._fresh[:] = False

    def reset(self, env_ids=None):
        super().reset(env_ids)
        self._fresh[slice(None) if env_ids is None else env_ids] = True


@dataclass(kw_only=True)
class LowPassJointPositionActionCfg(JointPositionActionCfg):
    head_alpha: float = LOWPASS_HEAD
    legs_alpha: float = LOWPASS_LEGS

    def build(self, env):
        return LowPassJointPositionAction(self, env)


# v6 (2026-10-10, training/HL2915-V6-TRAINING-PLAN-20261010-CN.md). On the robot the walk rocked
# and threw its head because of IMU latency: in the BAM simulation the walks stay calm up to
# ~20 ms of it and self-oscillate from 25-30 ms, and the robot sits at ~20-25 ms once robotd's
# median filter is off. The stock observation delay is 0-1 control ticks (0-20 ms), at the edge.
V6_IMU_DELAY_LAG = (0, 2)
IMU_TERMS = ("base_ang_vel", "projected_gravity", "raw_accelerometer")
# Leg gain spread centred on the fit. The weak skew (0.4-1.1) was chosen so a simulation without
# IMU latency would fall forward as the robot did; the latency explains that now, and the skew
# taught the policy to drive its targets hard: on the robot a deploy-time action scale of 0.8 is
# what calmed it (neck target swing 10-17 deg -> 4-7, three 8 s walks without a fall).
V6_JOINT_KP_RANGE = (0.6, 1.2)
# Per-episode gain on the whole action (the target's offset from the default pose), so the loop
# gain the policy leans on is not a single number; robotd's 0.8 lies inside.
V6_ACTION_GAIN_RANGE = (0.75, 1.05)


class Hl2915JointPositionAction(JointPositionAction):
    """Joint targets with a per-episode action gain and, optionally, robotd's low-pass.

    target = default + gain * scale * action, then the low-pass as LowPassJointPositionAction
    applies it (alpha 1.0 = off). The raw action kept for the term is the policy's own, as
    robotd feeds its last action back unscaled."""

    def __init__(self, cfg, env):
        super().__init__(cfg=cfg, env=env)
        alpha = torch.full((self.action_dim,), cfg.legs_alpha, device=self.device)
        for i, name in enumerate(self._target_names):
            if name in HEAD_JOINT_NAMES:
                alpha[i] = cfg.head_alpha
        self._alpha = alpha
        self._gain_range = cfg.gain_range
        self._gain = torch.ones(self.num_envs, 1, device=self.device)
        self._previous = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._fresh = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)

    def process_actions(self, actions):
        super().process_actions(actions * self._gain)
        self._raw_actions[:] = actions
        filtered = self._alpha * self._processed_actions + (1.0 - self._alpha) * self._previous
        self._processed_actions = torch.where(self._fresh[:, None], self._processed_actions, filtered)
        self._previous = self._processed_actions.clone()
        self._fresh[:] = False

    def reset(self, env_ids=None):
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        self._fresh[ids] = True
        lo, hi = self._gain_range
        self._gain[ids] = torch.empty_like(self._gain).uniform_(lo, hi)[ids]


@dataclass(kw_only=True)
class Hl2915JointPositionActionCfg(JointPositionActionCfg):
    head_alpha: float = 1.0
    legs_alpha: float = 1.0
    gain_range: tuple[float, float] = (1.0, 1.0)

    def build(self, env):
        return Hl2915JointPositionAction(self, env)


def _v6(cfg, lowpass, gain_dr=True):
    """IMU observed 0-40 ms late, leg gain centred, a per-episode action gain; `lowpass` also
    trains robotd's target low-pass (the -V6LP- tasks, the A/B against -V6-). `gain_dr` off
    (-NoGainDR-) keeps the action gain at 1: VelStand-V6LP got up from the floor far worse than
    v3 (2026-10-11: fallen_too_long 2.83 against 0.79 per batch), and a gain drawn down to 0.75
    takes torque from exactly the push a stand-up needs."""
    for name in IMU_TERMS:
        term = cfg.observations["actor"].terms.get(name)
        if term is not None:
            term.delay_min_lag, term.delay_max_lag = V6_IMU_DELAY_LAG
    cfg.events["randomize_joint_actuators"].params["kp_range"] = V6_JOINT_KP_RANGE
    old = cfg.actions["joint_pos"]
    assert type(old) is JointPositionActionCfg, type(old)
    cfg.actions["joint_pos"] = Hl2915JointPositionActionCfg(
        **{f.name: getattr(old, f.name) for f in fields(old) if f.init},
        head_alpha=LOWPASS_HEAD if lowpass else 1.0,
        legs_alpha=LOWPASS_LEGS if lowpass else 1.0,
        gain_range=V6_ACTION_GAIN_RANGE if gain_dr else (1.0, 1.0))
    return cfg


def _low_pass(cfg):
    if not TRAIN_WITH_LOWPASS:
        return cfg
    old = cfg.actions["joint_pos"]
    assert type(old) is JointPositionActionCfg
    cfg.actions["joint_pos"] = LowPassJointPositionActionCfg(
        **{f.name: getattr(old, f.name) for f in fields(old) if f.init})
    return cfg


# StandUp: the rise ends with the head thrown from the bottom of the neck's range to the top
# (2026-10-07 on the robot: neck -89° → target +85° in 0.3 s), and the head's momentum then
# throws the robot onto its back. Nothing in the task charged for it. Head joint speed is
# taxed once the rises exist — introduced late, like the stock task's other smoothness
# terms, since a tax during discovery stops the flips being found at all.
HEAD_SPEED_STAGES = [
    {"step": 0, "weight": 0.0},
    {"step": 3000 * 24, "weight": -0.01},
    {"step": 4000 * 24, "weight": -0.03},
]


def _tax_head_speed(cfg):
    from mjlab.envs.mdp import rewards as mjlab_rewards

    cfg.rewards["head_joint_vel"] = RewardTermCfg(
        func=mjlab_rewards.joint_vel_l2, weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HEAD_JOINT_NAMES)})
    cfg.curriculum["head_joint_vel_weight"] = CurriculumTermCfg(
        func=microduck_mdp.reward_weight,
        params={"reward_name": "head_joint_vel", "weight_stages": HEAD_SPEED_STAGES})
    return cfg


def make_hl2915_variant(cfg, robot_cfg):
    cfg.scene.entities = {**cfg.scene.entities, "robot": robot_cfg}
    return _low_pass(_gate_air_time(_widen_dr(cfg)))


# ── LieDown (2026-10-14, training/LIEDOWN-DESIGN-20261013-CN.md) ──────────────────────────────
# Resting the head needs the robot lying down first: seated, the head let go anywhere tips it over
# (2026-10-13: backwards with the head at the sit's pose, onto the beak with it lowered). The sitstand
# task with the sit swapped for LIE — kneeling on shins and feet, the soft jaw on the floor, the trunk
# pitched 26 deg nose-down — the pose a limp robot falls into and stays in (rev-a model and this
# model: perturbed x10 it ends within 7-8 deg; the jaw carries 2.1 N of the 8.3). The posture flag
# stays in the vx slot (0 stand, 1 lie). The head is part of the LIE target (not commandable here),
# the upright terms become terms on the orientation the posture asks for, and two terms are new:
# limp-ready (lying, the servos should hold nothing, so cutting the torque changes nothing) and a
# cap on the head's contact force (the camera and ToF are in it).

# Servo-index (14-joint) LIE pose; hip yaws stay at HOME (0).
LIE_OVERRIDES = {
    1: 0.0, 2: -0.782, 3: 1.485, 4: 0.227,
    5: -0.940, 6: 0.466, 7: 0.0, 8: 0.0,
    10: 0.0, 11: 0.784, 12: -1.484, 13: -0.229,
}
LIE_JOINTS = list(range(14))
LIE_Z = 0.058
LIE_STAND_Z = 0.115
# Gravity in the trunk frame lying (settled limp in this model: [0.442, 0.002, -0.897]).
LIE_GRAVITY_B = (0.442, 0.0, -0.897)
LIE_RAMP_S = 2.5
LIE_DWELL_S = (4.0, 7.0)
LIE_EPISODE_S = 14.0
# Head on the floor at rest: 2.1 N. Above this it is an impact.
HEAD_CONTACT_FREE_N = 3.5
LIE_RESET_PROB = 0.3


def _lie_target_gravity(env, command_name):
    blend = microduck_mdp._posture_blend(env, command_name)
    up = torch.tensor((0.0, 0.0, -1.0), device=env.device)
    lie = torch.tensor(LIE_GRAVITY_B, device=env.device)
    g = up + blend.unsqueeze(-1) * (lie - up)
    return g / g.norm(dim=-1, keepdim=True)


def _gravity_b(env, asset_cfg):
    return torch.nan_to_num(env.scene[asset_cfg.name].data.projected_gravity_b, nan=0.0)


def posture_orientation_linear(env, command_name, asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """cos of the angle between the trunk's gravity and the commanded posture's: 1 on target, 0 a
    quarter turn off, -1 inverted. `body_upright_linear` with the target moved, gradient everywhere."""
    return (_gravity_b(env, asset_cfg) * _lie_target_gravity(env, command_name)).sum(dim=-1)


def _orientation_err_sq(env, command_name, asset_cfg):
    return ((_gravity_b(env, asset_cfg) - _lie_target_gravity(env, command_name)) ** 2).sum(dim=-1)


def _lie_targets(env, asset, command_name, sit_overrides, head_lag):
    """`_posture_targets`, the head (servo joints 5-8) optionally lagging (v3): it follows
    smoothstep((blend - head_lag) / (1 - head_lag)), so going down the legs and trunk are on their
    way before the head moves, and getting up the head is up before the trunk is."""
    blend, target = microduck_mdp._posture_targets(env, asset, command_name, sit_overrides)
    if head_lag > 0:
        stand = microduck_mdp._servo_default_joint_pos(env, asset)
        u = torch.clamp((blend - head_lag) / (1.0 - head_lag), 0.0, 1.0)
        hb = (u * u * (3 - 2 * u)).unsqueeze(-1)
        lie = stand.clone()
        for idx, val in sit_overrides.items():
            lie[:, idx] = val
        target = target.clone()
        target[:, 5:9] = stand[:, 5:9] + hb * (lie[:, 5:9] - stand[:, 5:9])
    return blend, target


def lie_pose_match(env, command_name, sit_overrides, joint_indices, std=0.5, head_lag=0.0,
                   asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    asset = env.scene[asset_cfg.name]
    _, target = _lie_targets(env, asset, command_name, sit_overrides, head_lag)
    q = microduck_mdp._servo_joint_pos(env, asset)[:, joint_indices]
    return torch.exp(-((q - target[:, joint_indices]) / std) ** 2).mean(dim=-1)


def lie_pose_l1(env, command_name, sit_overrides, joint_indices, head_lag=0.0,
                asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    asset = env.scene[asset_cfg.name]
    _, target = _lie_targets(env, asset, command_name, sit_overrides, head_lag)
    q = microduck_mdp._servo_joint_pos(env, asset)[:, joint_indices]
    return -torch.abs(q - target[:, joint_indices]).mean(dim=-1)


def lie_soft_landing(env, command_name, rest_z, band, free_vz, body_name="jaw_soft",
                     asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """-(downward speed of the jaw past `free_vz`)+, m/s, while lying is commanded and the jaw is
    within `band` of where it rests lying: slow down before the floor (v3). The landing-force cap
    alone charges only the hit, after the motion that caused it."""
    asset = env.scene[asset_cfg.name]
    cache = env.__dict__.setdefault("_lie_jaw_body", {})
    if body_name not in cache:
        ids, _ = asset.find_bodies(body_name)
        cache[body_name] = ids[0]
    b = cache[body_name]
    z = torch.nan_to_num(asset.data.body_link_pos_w[:, b, 2] - env.scene.terrain.env_origins[:, 2], nan=1.0)
    vz = torch.nan_to_num(asset.data.body_link_lin_vel_w[:, b, 2], nan=0.0)
    flag = env.command_manager.get_command(command_name)[:, 0]
    near = (z < rest_z + band).float()
    return -torch.clamp(-vz - free_vz, min=0.0) * near * flag


def posture_composite_oriented(env, command_name, sit_overrides, joint_indices, sit_z, stand_z,
                               height_std=0.03, orient_std=0.40, pose_std=0.40, head_lag=0.0,
                               asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """`posture_composite` (height x orientation x pose, a product) with the orientation measured
    against the commanded posture's instead of upright; head inside `joint_indices`."""
    asset = env.scene[asset_cfg.name]
    _, target = _lie_targets(env, asset, command_name, sit_overrides, head_lag)
    target_z, z = microduck_mdp._posture_height(env, command_name, sit_z, stand_z)
    height = torch.exp(-((z - target_z) / height_std) ** 2)
    orient = torch.exp(-_orientation_err_sq(env, command_name, asset_cfg) / orient_std**2)
    joint_pos = microduck_mdp._servo_joint_pos(env, asset)[:, joint_indices]
    pose = torch.exp(-((joint_pos - target[:, joint_indices]) ** 2).mean(dim=-1) / pose_std**2)
    return height * orient * pose


def _at_posture(env, command_name, sit_z, stand_z, band_full, band_zero, orient_full, orient_zero,
                asset_cfg):
    """1 resting at the commanded posture (ramp done, height and orientation in band), 0 away."""
    flag = env.command_manager.get_command(command_name)[:, 0]
    ramp_done = ((flag - microduck_mdp._posture_blend(env, command_name)).abs() < 0.02).float()
    target_z, z = microduck_mdp._posture_height(env, command_name, sit_z, stand_z)
    t = torch.clamp((band_zero - (z - target_z).abs()) / (band_zero - band_full), 0.0, 1.0)
    err = _orientation_err_sq(env, command_name, asset_cfg).sqrt()
    u = torch.clamp((orient_zero - err) / (orient_zero - orient_full), 0.0, 1.0)
    return ramp_done * t * t * (3 - 2 * t) * u * u * (3 - 2 * u)


def posture_stillness_oriented(env, command_name, sit_z, stand_z, band_full=0.012, band_zero=0.03,
                               vel_std=0.05, orient_full=0.25, orient_zero=0.6,
                               asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """`posture_stillness` gated on the commanded orientation (|g - g_target| 0.25 ≈ 14 deg full,
    0.6 ≈ 35 deg zero) instead of on upright."""
    v = torch.nan_to_num(env.scene[asset_cfg.name].data.root_link_lin_vel_w, nan=0.0).norm(dim=-1)
    gate = _at_posture(env, command_name, sit_z, stand_z, band_full, band_zero, orient_full,
                       orient_zero, asset_cfg)
    return torch.exp(-((v / vel_std) ** 2)) * gate


def lie_limp_ready(env, command_name, sit_z, stand_z, asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """-sum(actuator torque^2), N·m, only lying at rest: the pose has to rest on the floor, not on
    the servos, so that robotd cutting their torque changes nothing (and they cool)."""
    flag = env.command_manager.get_command(command_name)[:, 0]
    gate = _at_posture(env, command_name, sit_z, stand_z, 0.012, 0.03, 0.25, 0.6, asset_cfg) * flag
    tau = torch.nan_to_num(env.scene[asset_cfg.name].data.actuator_force, nan=0.0)
    return -(tau**2).sum(dim=-1) * gate


def head_contact_force(env, sensor_name, free_n=HEAD_CONTACT_FREE_N, command_name=None):
    """-(|net head-floor force| - free_n)+, N: lying on the jaw is free, landing on it is not. With
    `command_name`, only while lying is commanded: getting up may push on the jaw."""
    force = torch.nan_to_num(env.scene[sensor_name].data.force, nan=0.0)
    excess = torch.clamp(force.norm(dim=-1).amax(dim=-1) - free_n, min=0.0)
    if command_name is not None:
        excess = excess * env.command_manager.get_command(command_name)[:, 0]
    return -excess


def set_lie_or_stand_state(env, env_ids, lie_prob, lie_overrides, tilt_rad,
                           asset_cfg=microduck_mdp._DEFAULT_ASSET_CFG):
    """`set_random_ground_state` with its sitting bucket as the LIE pose: knees folded to the LIE
    angles, then pitched to the lying orientation (body-frame rotation about y)."""
    microduck_mdp.set_random_ground_state(
        env, env_ids, asset_cfg=asset_cfg, face_down_prob=0.0, face_up_prob=0.0,
        sitting_prob=lie_prob, standing_prob=1.0 - lie_prob,
        sitting_joint_overrides=lie_overrides, sitting_joint_noise_std=0.08,
        sitting_tilt_max=math.radians(5), sitting_z_min=0.060, sitting_z_max=0.070,
        standing_z_min=0.11, standing_z_max=0.12)
    if env_ids is None or len(env_ids) == 0:
        return
    env_ids = env_ids.to(env.device, dtype=torch.long)
    asset = env.scene[asset_cfg.name]
    knee = 7 + microduck_mdp._servo_joint_ids(env, asset)[3]
    lying = env_ids[env.sim.data.qpos[env_ids, knee] > 0.7]
    if len(lying) == 0:
        return
    pitch = math.radians(LIE_PITCH_DEG) + (torch.rand(len(lying), device=env.device) * 2 - 1) * tilt_rad
    c, s = torch.cos(pitch / 2), torch.sin(pitch / 2)
    q = env.sim.data.qpos[lying, 3:7].clone()
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # q ⊗ [c, 0, s, 0]
    env.sim.data.qpos[lying, 3:7] = torch.stack(
        (w * c - y * s, x * c - z * s, y * c + w * s, z * c + x * s), dim=1)


LIE_PITCH_DEG = math.degrees(math.atan2(LIE_GRAVITY_B[0], -LIE_GRAVITY_B[2]))

# v2 (-HeadLate-): v1 charged the head's landing from step 0. In the robot-matched simulation its
# iteration-750 policy lay down 23 times of 24 — in 1.4-2.0 s, the jaw hitting at 40-54 N, the pose
# then still to 1 deg with the torque off — but never got up again, and by iteration 1000 it no longer
# lay down at all: the tax on the landing was the attempt-tax the sitstand notes warn about, and it
# taxed the jaw-braced rise as well. So: charged only while lying is commanded, from iteration 1000,
# and half the resets start lying (more rises to learn from).
HEAD_CONTACT_STAGES = [
    {"step": 0, "weight": 0.0},
    {"step": 1000 * 24, "weight": 0.1},
    {"step": 1500 * 24, "weight": 0.3},
    {"step": 2000 * 24, "weight": 0.5},
]
LIE_RESET_PROB_V2 = 0.5

# v3 (-SoftLand-): v2 on the robot (2026-10-14) folded everything at once and the head's tip hit
# the floor hard (sim: jaw 34-64 N). The ramp slowed 2.5 -> 3.5 s, the head's target lagging the
# rest (moves in the second half of the ramp), and the jaw's downward speed past 5 cm/s charged in
# its last 3 cm (jaw_soft's origin rests 0.0905 m up lying), from iteration 500.
LIE_RAMP_S_V3 = 3.5
HEAD_LAG_V3 = 0.5
JAW_REST_Z, JAW_BAND, JAW_FREE_VZ = 0.0905, 0.03, 0.05
SOFT_LAND_STAGES = [
    {"step": 0, "weight": 0.0},
    {"step": 500 * 24, "weight": 2.0},
    {"step": 1000 * 24, "weight": 5.0},
]
# v4 (-SoftLandLate-): in the robot-matched simulation v3's iteration-500 policy lay down (in 1 s,
# still fast) and from iteration 750 on none did — they crouch 5 mm and stay standing: the jaw-speed
# charge from iteration 500 was one more attempt-tax. Charged only once lying is solid (v2 kept lying
# with the head-contact cap from 1000), ramped gently.
SOFT_LAND_STAGES_V4 = [
    {"step": 0, "weight": 0.0},
    {"step": 1500 * 24, "weight": 0.5},
    {"step": 2000 * 24, "weight": 1.0},
    {"step": 2500 * 24, "weight": 2.0},
    {"step": 3000 * 24, "weight": 3.0},
]

# Limp-ready only once lying exists (a tax during discovery stops the skill being found).
LIMP_READY_STAGES = [
    {"step": 0, "weight": 0.0},
    {"step": 1000 * 24, "weight": 0.5},
    {"step": 1500 * 24, "weight": 1.0},
]


def make_hl2915_liedown_env_cfg(play=False, rough=False, v=1):
    from mjlab.sensor import ContactMatch, ContactSensorCfg

    cfg = make_microduck_sitstand_env_cfg(play=play, rough=rough)
    cfg.episode_length_s = LIE_EPISODE_S

    head_contact = ContactSensorCfg(
        name="head_ground_contact",
        primary=ContactMatch(mode="subtree", pattern="neck", entity="robot"),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("force",), reduce="netforce", num_slots=1)
    cfg.scene.sensors = (*cfg.scene.sensors, head_contact)

    # The command: sitstand's with the LIE height.
    twist = cfg.commands["twist"]
    twist.sit_z, twist.stand_z = LIE_Z, LIE_STAND_Z
    twist.ramp_s = LIE_RAMP_S_V3 if v >= 3 else LIE_RAMP_S
    twist.resampling_time_range = LIE_DWELL_S

    # The head is part of the pose: no head commands, every joint in the pose terms.
    head = cfg.commands["head_pose"]
    head.ranges = tuple((0.0, 0.0) for _ in head.ranges)
    cfg.curriculum.pop("head_pose_range", None)
    cfg.rewards.pop("head_pose_tracking", None)
    pose = {"sit_overrides": LIE_OVERRIDES, "joint_indices": LIE_JOINTS}
    cfg.rewards["posture_pose_legs"].params.update(pose)
    cfg.rewards["posture_pose_l1"].params.update(pose)
    if v >= 3:
        cfg.rewards["posture_pose_legs"].func = lie_pose_match
        cfg.rewards["posture_pose_legs"].params["head_lag"] = HEAD_LAG_V3
        cfg.rewards["posture_pose_l1"].func = lie_pose_l1
        cfg.rewards["posture_pose_l1"].params["head_lag"] = HEAD_LAG_V3
    for name in ("posture_height", "posture_height_sharp", "posture_height_l1"):
        cfg.rewards[name].params.update(sit_z=LIE_Z, stand_z=LIE_STAND_Z)

    # Upright → the commanded orientation.
    cfg.rewards.pop("upright_while_tall", None)
    cfg.rewards["upright_linear"] = RewardTermCfg(
        func=posture_orientation_linear, weight=2.5, params={"command_name": "twist"})
    cfg.rewards["posture_composite"] = RewardTermCfg(
        func=posture_composite_oriented, weight=3.0,
        params={"command_name": "twist", **pose, "sit_z": LIE_Z, "stand_z": LIE_STAND_Z,
                "head_lag": HEAD_LAG_V3 if v >= 3 else 0.0})
    cfg.rewards["posture_stillness"] = RewardTermCfg(
        func=posture_stillness_oriented, weight=2.0,
        params={"command_name": "twist", "sit_z": LIE_Z, "stand_z": LIE_STAND_Z})

    # New: limp-ready and the head's landing.
    cfg.rewards["limp_ready"] = RewardTermCfg(
        func=lie_limp_ready, weight=0.0,
        params={"command_name": "twist", "sit_z": LIE_Z, "stand_z": LIE_STAND_Z})
    cfg.curriculum["limp_ready_weight"] = CurriculumTermCfg(
        func=microduck_mdp.reward_weight,
        params={"reward_name": "limp_ready", "weight_stages": LIMP_READY_STAGES})
    if v == 1:
        cfg.rewards["head_contact"] = RewardTermCfg(
            func=head_contact_force, weight=0.5, params={"sensor_name": "head_ground_contact"})
    else:
        cfg.rewards["head_contact"] = RewardTermCfg(
            func=head_contact_force, weight=0.0,
            params={"sensor_name": "head_ground_contact", "command_name": "twist"})
        cfg.curriculum["head_contact_weight"] = CurriculumTermCfg(
            func=microduck_mdp.reward_weight,
            params={"reward_name": "head_contact", "weight_stages": HEAD_CONTACT_STAGES})

    if v >= 3:
        cfg.rewards["soft_landing"] = RewardTermCfg(
            func=lie_soft_landing, weight=0.0,
            params={"command_name": "twist", "rest_z": JAW_REST_Z, "band": JAW_BAND, "free_vz": JAW_FREE_VZ})
        cfg.curriculum["soft_landing_weight"] = CurriculumTermCfg(
            func=microduck_mdp.reward_weight,
            params={"reward_name": "soft_landing",
                    "weight_stages": SOFT_LAND_STAGES_V4 if v >= 4 else SOFT_LAND_STAGES})

    # Starts: mostly standing, the rest already lying.
    cfg.events["set_ground_state"] = EventTermCfg(
        func=set_lie_or_stand_state, mode="reset",
        params={"lie_prob": LIE_RESET_PROB if v == 1 else LIE_RESET_PROB_V2, "lie_overrides": LIE_OVERRIDES,
                "tilt_rad": math.radians(5)})
    return cfg


# (task id, make fn, kwargs, rl cfg, robot, backlash robot or None). Each keeps its base
# task's collision model: Velocity → walk model, the rest → allcollisions.
_TASKS = (
    ("Mjlab-Velocity-Flat-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {}, MicroduckRlCfg,
     HL2915_WALK_ROBOT_CFG, None),
    ("Mjlab-Velocity-Rough-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {"rough": True}, MicroduckRlCfg,
     HL2915_WALK_ROBOT_CFG, None),
    ("Mjlab-VelStand-Flat-HL2915-MicroDuck", make_microduck_velstand_env_cfg, {}, MicroduckVelStandRlCfg,
     HL2915_STANDUP_ROBOT_CFG, None),
    ("Mjlab-VelStand-Rough-HL2915-MicroDuck", make_microduck_velstand_env_cfg, {"rough": True}, MicroduckVelStandRlCfg,
     HL2915_STANDUP_ROBOT_CFG, None),
    ("Mjlab-StandUp-Flat-HL2915-MicroDuck", make_microduck_standup_env_cfg, {}, MicroduckStandUpRlCfg,
     HL2915_STANDUP_ROBOT_CFG, None),
    ("Mjlab-SitStand-Flat-HL2915-MicroDuck", make_microduck_sitstand_env_cfg, {}, MicroduckSitStandRlCfg,
     HL2915_STANDUP_ROBOT_CFG, None),
    ("Mjlab-Roulade-Flat-HL2915-MicroDuck", make_microduck_roulade_env_cfg, {}, MicroduckRouladeRlCfg,
     HL2915_STANDUP_ROBOT_CFG, None),
    # Backlash: ±1° serial gear play, encoder read through it (the HL-2915 is rated ≤1°).
    ("Mjlab-Velocity-Flat-Backlash-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {}, MicroduckRlCfg,
     None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-VelStand-Flat-Backlash-HL2915-MicroDuck", make_microduck_velstand_env_cfg, {}, MicroduckVelStandRlCfg,
     None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-VelStand-Rough-Backlash-HL2915-MicroDuck", make_microduck_velstand_env_cfg, {"rough": True},
     MicroduckVelStandRlCfg, None, HL2915_BACKLASH_ROBOT_CFG),
    # On the robot (2026-10-07, same floor, same hour) only the backlash walk policy walked
    # its 4 s; both plain ones fell inside 1.5 s. So every skill gets a backlash variant.
    ("Mjlab-StandUp-Flat-Backlash-HL2915-MicroDuck", make_microduck_standup_env_cfg, {}, MicroduckStandUpRlCfg,
     None, HL2915_BACKLASH_ROBOT_CFG),
    # Slippery floors: the same walk with the foot friction drawn from LOW_FRICTION_RANGE.
    ("Mjlab-Velocity-Flat-Backlash-LowFriction-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-SitStand-Flat-Backlash-HL2915-MicroDuck", make_microduck_sitstand_env_cfg, {}, MicroduckSitStandRlCfg,
     None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-Roulade-Flat-Backlash-HL2915-MicroDuck", make_microduck_roulade_env_cfg, {}, MicroduckRouladeRlCfg,
     None, HL2915_BACKLASH_ROBOT_CFG),
    # v5: the v4 walk with the command delay widened (LONG_DELAY_LAG), flat and rough. Rough
    # uses the same feet-only backlash model, as upstream's Velocity-Rough-Backlash does.
    ("Mjlab-Velocity-Flat-Backlash-Delay50-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Rough-Backlash-Delay50-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {"rough": True},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # The v5 flat walk with the leg servos at Kp 8 (head stays 16), the A/B for LEGS_KP_BASE.
    ("Mjlab-Velocity-Flat-Backlash-Delay50-LegsKp8-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # v6: the v4 low-friction walk with `_v6` (IMU latency, centred leg gain, action gain), with
    # and without robotd's low-pass in training, and the same as one walk-and-stand policy.
    ("Mjlab-Velocity-Flat-Backlash-LowFriction-V6-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-LowFriction-V6LP-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-VelStand-Flat-Backlash-LowFriction-V6LP-HL2915-MicroDuck", make_microduck_velstand_env_cfg, {},
     MicroduckVelStandRlCfg, None, HL2915_BACKLASH_ROBOT_CFG),
    # 2026-10-11: the v6lp walk with the leg servos at Kp 8 and 12 (head stays 16) — whether 16,
    # the gain the robot runs, is the right one — and VelStand-V6LP without the action-gain draw.
    ("Mjlab-Velocity-Flat-Backlash-LowFriction-V6LP-LegsKp8-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-LowFriction-V6LP-LegsKp12-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-VelStand-Flat-Backlash-LowFriction-V6LP-NoGainDR-HL2915-MicroDuck", make_microduck_velstand_env_cfg,
     {}, MicroduckVelStandRlCfg, None, HL2915_BACKLASH_ROBOT_CFG),
    # The v6lp walk on floors down to friction 0.15 (dusty tile, 2026-10-11).
    ("Mjlab-Velocity-Flat-Backlash-Slippery-V6LP-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # 2026-10-12: floors from dusty tile to silicone pads on clean tile (WIDE_FRICTION_RANGE), and
    # the same with the command delay widened to 10-50 ms (the robot measures ~32 ms).
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-HL2915-MicroDuck", make_microduck_velocity_env_cfg, {},
     MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-Delay50-V6LP-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # v7 (2026-10-12 night, new hip zeros): every v6 walk wandered off its heading 20-55 deg in
    # 6 s, in either direction, and slip3999 walked with the left toe out 4 deg and the right
    # straight. -Sym- adds the left-right mirror loss, -YawTight- tightens the yaw-rate
    # tracking (see `_tight_yaw`); both on the wide-friction walk, one policy for both floors.
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-YawTight-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, MicroduckRlCfg, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawTight-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # 2026-10-13: v7 in the BAM sweep — -Sym- symmetric gait and longer steps on every floor,
    # -YawTight- worse yaw. Walk and stand in one network (the stand-up policy self-oscillates at
    # 4-5 Hz standing and follows only 20-45% of a head command; walking policies stand still
    # calmly), with and without tighter head tracking (`_head_track`). Getting up stays separate.
    ("Mjlab-VelStand-Flat-Backlash-WideFriction-V6LP-Sym-HL2915-MicroDuck", make_microduck_velstand_env_cfg,
     {}, "velstand-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-VelStand-Flat-Backlash-WideFriction-V6LP-Sym-HeadTrack-HL2915-MicroDuck", make_microduck_velstand_env_cfg,
     {}, "velstand-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    # v9 (2026-10-13): wf-sym-2500 walked 0.95 / 1.08 m on the pads but shuffled at 2.3-2.6 Hz on
    # bare tile (0.38 / 0.97 m), as wf3750 did — so the bare-floor walk goes back to -Slippery-
    # (0.15-1.3) with the mirror loss; and every walk drifts by per-stride yaw kicks (`yaw_drift_tracking`).
    ("Mjlab-Velocity-Flat-Backlash-Slippery-V6LP-Sym-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-Slippery-V6LP-Sym-YawDrift-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-HL2915-MicroDuck", make_microduck_velocity_env_cfg,
     {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # v10 (2026-10-14): wf-sym-drift-3750 held its heading on the robot but walked 0.55 m where
    # wf-sym-2500 walked 0.80 the same hour, as the BAM sweep predicted (every -YawDrift- walk swings
    # the hip 21-24 deg, every walk without it 26-34): the drift term bought the heading with stride,
    # because the per-tick speed term still pays 84% for half the commanded speed. -LinTight- makes
    # speed cost something (`_lin_tight`); -YawDriftLite- halves the drift weight. And the walks swing
    # the head 15-25 deg a stride as a counterweight — head_yaw reached 70 C first and stopped the
    # walking — so -HeadCalm- taxes head joint speed once a gait exists (`_head_calm`).
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-LinTight-HeadCalm-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDriftLite-LinTight-HeadCalm-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # LieDown (2026-10-14): stand <-> lie on the jaw, see `make_hl2915_liedown_env_cfg`.
    ("Mjlab-LieDown-Flat-Backlash-V6LP-Sym-HL2915-MicroDuck", make_hl2915_liedown_env_cfg, {}, "sitstand-sym",
     None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-LieDown-Flat-Backlash-V6LP-Sym-HeadLate-HL2915-MicroDuck", make_hl2915_liedown_env_cfg, {"v": 2},
     "sitstand-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-LieDown-Flat-Backlash-V6LP-Sym-SoftLand-HL2915-MicroDuck", make_hl2915_liedown_env_cfg, {"v": 3},
     "sitstand-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-LieDown-Flat-Backlash-V6LP-Sym-SoftLandLate-HL2915-MicroDuck", make_hl2915_liedown_env_cfg, {"v": 4},
     "sitstand-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    # v11 (2026-10-14 night): v10's long stride (hip swing 45 deg) walked unsteadily on the robot on
    # both floors, falling 9 times in 2 minutes; the user: gait and steadiness over speed. v9 (steady
    # and straight on the robot) with the head tax, and the same with the speed term halfway to v10's.
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-HeadCalm-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-LinMid-HeadCalm-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # v12 (2026-10-17): with all eight leg bearings in, v9 walks straight and steady on the robot - at
    # 920 g, past the model's 850 +-5 % (`_mass920`). Fine-tuned from v9 wf-sym-drift model_3750 so the
    # gait the user likes is where it starts. The head still swings ~17 deg peak to peak as a
    # counterweight; v11's head *speed* tax took the small balancing moves with the big swing and fell
    # on the robot (though that robot had no leg bearings yet). -HeadBand- leaves the head free within
    # +-2.9 deg of its command and charges only the swing past it (`_head_band`), strong and soft.
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-Mass920-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-Mass920-HeadBand-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    ("Mjlab-Velocity-Flat-Backlash-WideFriction-V6LP-Sym-YawDrift-Mass920-HeadBandSoft-HL2915-MicroDuck",
     make_microduck_velocity_env_cfg, {}, None, None, HL2915_WALK_BACKLASH_ROBOT_CFG),
    # v5 skills (2026-10-18), at 920 g. StandUp with the left-right mirror loss: standing on
    # standup_v3bl (trained without it) the robot puts its right foot ~1 deg forward and loads the
    # right hip roll (621 mA against the left's 67; 62 / 79 in the zero pose), and its head leans -
    # a habit of the network, not the hardware (2026-10-17). Its 61-dim obs is the walk's, and the
    # body-command signs (roll negated, z / pitch kept) hold for its [z, roll, pitch] slot.
    ("Mjlab-StandUp-Flat-Backlash-Mass920-Sym-HL2915-MicroDuck", make_microduck_standup_env_cfg, {},
     "standup-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    # Roulade (the forward roll; already trained with the mirror loss): v4bl fine-tuned at 920 g.
    ("Mjlab-Roulade-Flat-Backlash-Mass920-HL2915-MicroDuck", make_microduck_roulade_env_cfg, {},
     MicroduckRouladeRlCfg, None, HL2915_BACKLASH_ROBOT_CFG),
    # Picking a cloth up off the floor in the beak (the user, 2026-10-18): upstream's GroundPick -
    # crouch the mouth tip to just above the floor, pointing down, without touching it, then back to
    # standing carrying a payload at the mouth - at 920 g, with the mirror loss (phase kept, see
    # `phase_command_symmetry`), the bottom held 0.6 s instead of 0.2 so the mouth can close on the
    # cloth, and a cloth's weight (`_cloth_pick`). The jaw is passive here; robotd opens the mouth on
    # the way down and closes it at the bottom.
    ("Mjlab-GroundPick-Flat-Backlash-Mass920-Sym-Cloth-HL2915-MicroDuck", make_microduck_ground_pick_env_cfg,
     {}, "groundpick-sym", None, HL2915_BACKLASH_ROBOT_CFG),
    ("Mjlab-GroundPick-Flat-Backlash-Mass920-Sym-Cloth-ClothNear-HL2915-MicroDuck",
     make_microduck_ground_pick_env_cfg, {}, "groundpick-sym", None, HL2915_BACKLASH_ROBOT_CFG),
)

# Gear play per joint, half-width in degrees, drawn per joint and per episode. Measured on the
# robot (2026-10-08, legs held in the air at Kp 32 and moved by hand): 0.6-1.2 deg of travel
# per joint including the servo's own give, about 5 mm at the toe summed over the leg —
# roughly +-0.3-0.4 deg each. The stock model's fixed +-1 deg is two to three times that.
BACKLASH_HALF_DEG = (0.3, 1.0)


BACKLASH_JOINTS = (r"passive_.*_backlash",)


@requires_model_fields("jnt_range")
def randomize_backlash(env, env_ids, half_deg, asset_cfg):
    """Symmetric +-b limits on every backlash hinge, b drawn per joint (the stock joint_limits
    draws each bound on its own, which would make the play lopsided).

    `asset_cfg` must come in through the event's params: the event manager resolves the
    joint selection only there. A default argument stays unresolved and selects every joint
    — the first v4 run (2026-10-08) clamped hips, knees and ankles to +-1 deg that way and
    learned nothing."""
    from mjlab.envs.mdp.dr._core import _get_entity_indices

    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    asset = env.scene[asset_cfg.name]
    joints = _get_entity_indices(asset.indexing, asset_cfg, "joint", False)
    lo, hi = (math.radians(x) for x in half_deg)
    b = torch.rand(len(env_ids), len(joints), device=env.device) * (hi - lo) + lo
    env_grid, joint_grid = torch.meshgrid(env_ids.to(env.device, torch.long), joints.to(torch.long),
                                          indexing="ij")
    env.sim.model.jnt_range[env_grid, joint_grid] = torch.stack((-b, b), dim=-1)


def _randomize_backlash(cfg):
    cfg.events["randomize_backlash"] = EventTermCfg(
        func=randomize_backlash, mode="reset",
        params={"half_deg": BACKLASH_HALF_DEG,
                "asset_cfg": SceneEntityCfg("robot", joint_names=BACKLASH_JOINTS)})
    return cfg


# Foot friction for the "-LowFriction-" tasks. The stock walk draws 0.7-1.3 (narrowed by
# upstream from mjlab's 0.3-1.2 for the prototype's grippy pads); the rev-a soles are printed
# TPU and the floor at home is glazed tile, which may sit below 0.7.
LOW_FRICTION_RANGE = (0.4, 1.3)
# "-Slippery-": lower still. On 2026-10-11 a dusty tiled floor turned the walk into a 3-4 Hz
# shuffle of 3-9 mm steps that turned up to 160 deg in 8 s, where carpet the same hour gave a clean
# 1.9 Hz walk of 32 mm steps; the BAM simulation shuffles the same way from a floor friction of
# 0.35 down, below the 0.4 the low-friction walks trained to.
SLIPPERY_FRICTION_RANGE = (0.15, 1.3)
# "-WideFriction-": up to grippy too. With 0.5 mm silicone pads on clean tile (2026-10-11) the
# robot no longer shuffled but swung its heading 15-110 deg in 6 s with yaw rates to 400 deg/s,
# against 145 on carpet; the BAM simulation at friction 2.0 gives the same (v6lp x0.9: trunk
# pitch-rate RMS 16 -> 39, 8 s heading 8 -> 74 deg, distance 1.25 -> 0.70 m; v4lf falls 6/6).
WIDE_FRICTION_RANGE = (0.15, 2.0)


def _low_friction(cfg, slippery=False, wide=False):
    cfg.events["foot_friction"].params["ranges"] = (
        WIDE_FRICTION_RANGE if wide else SLIPPERY_FRICTION_RANGE if slippery else LOW_FRICTION_RANGE)
    return cfg


# Command delay for the "-Delay50-" tasks, in 5 ms physics steps, redrawn every step like the
# stock one (2-6, 10-30 ms). On the robot, robotd's target low-pass adds a group delay the
# policies never train with — (1 - a) / a ticks: about 9 ms on the legs (0.7) and 20 ms on the
# head (0.5) — on top of a bus and servo latency not yet measured end to end. In the BAM
# simulation every walk policy falls at 60 ms and v4 at iteration 750 already at 40 ms.
# Feetech ports train wider: Open Duck 0-60 ms, the HD1910M port up to 50 ms.
LONG_DELAY_LAG = (2, 10)


def _long_delay(cfg):
    """Widen the actuator delay on this task's own copy: the robot cfgs are shared between
    the registered tasks, so setting the field in place would change every one of them."""
    robot = cfg.scene.entities["robot"]
    lo, hi = LONG_DELAY_LAG
    actuators = tuple(replace(a, delay_min_lag=lo, delay_max_lag=hi) for a in robot.articulation.actuators)
    robot = replace(robot, articulation=replace(robot.articulation, actuators=actuators))
    cfg.scene.entities = {**cfg.scene.entities, "robot": robot}
    return cfg


# "-YawTight-": the yaw-rate tracking reward is exp(-err^2 / std^2) with std^2 = 0.5, which still
# pays 84% for walking 17 deg/s off the commanded turn — a drift the robot shows and heading hold
# cannot keep up with. 0.1, the linear-velocity term's value, pays 41% for it.
YAW_TRACK_STD = math.sqrt(0.1)


def _tight_yaw(cfg):
    cfg.rewards["track_angular_velocity"].params["std"] = YAW_TRACK_STD
    return cfg


class yaw_drift_tracking:
    """exp(-(mean yaw rate - mean commanded yaw rate)^2 / std^2), both averaged over the last
    `window_s` — about one stride.

    The heading the robot wanders off (2026-10-12/13) is a random walk of per-stride kicks:
    net yaw per stride of +-6-21 deg (std) with single strides of -43 / +31, on every v6/v7
    walk, reproduced in the BAM simulation (7-12 deg). The per-tick yaw-rate term cannot see
    it under the gait's own yaw sway (p95 100-170 deg/s), and tightening that term (-YawTight-)
    made the sway the target and the walks worse. A one-stride boxcar cancels the sway and
    leaves the kick."""

    def __init__(self, cfg, env):
        window_s = cfg.params.get("window_s", 1.0)
        self._n = max(1, int(round(window_s / env.step_dt)))
        self._actual = torch.zeros(env.num_envs, self._n, device=env.device)
        self._cmd = torch.zeros(env.num_envs, self._n, device=env.device)
        self._count = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self._i = 0

    def __call__(self, env, std, command_name, window_s=1.0, asset_cfg=SceneEntityCfg("robot")):
        asset = env.scene[asset_cfg.name]
        self._actual[:, self._i] = asset.data.root_link_ang_vel_b[:, 2]
        self._cmd[:, self._i] = env.command_manager.get_command(command_name)[:, 2]
        self._i = (self._i + 1) % self._n
        self._count = torch.clamp(self._count + 1, max=self._n)
        filled = self._count.clamp(min=1).float()
        err = (self._actual.sum(dim=1) - self._cmd.sum(dim=1)) / filled
        return torch.exp(-torch.square(err) / std**2)

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        self._actual[env_ids] = 0.0
        self._cmd[env_ids] = 0.0
        self._count[env_ids] = 0


# "-YawDrift-": 0.15 rad/s (8.6 deg/s) of net turn over a stride costs ~63% of the term.
YAW_DRIFT_STD, YAW_DRIFT_WEIGHT, YAW_DRIFT_WINDOW_S = 0.15, 1.5, 1.0


def _yaw_drift(cfg):
    cfg.rewards["yaw_drift"] = RewardTermCfg(
        func=yaw_drift_tracking, weight=YAW_DRIFT_WEIGHT,
        params={"std": YAW_DRIFT_STD, "command_name": "twist", "window_s": YAW_DRIFT_WINDOW_S},
    )
    return cfg


# "-YawDriftLite-": the same term at half the weight.
YAW_DRIFT_LITE_WEIGHT = 0.75


# "-LinTight-": track_linear_velocity is exp(-err^2 / std^2) with std^2 = 0.1, so walking 0.13 m/s
# against a 0.27 command (the robot's v9) still pays 84%. At 0.04 that miss pays 66% and a 0.2 m/s
# miss 37%; weight 2 -> 3.
LIN_TRACK_STD, LIN_TRACK_WEIGHT = math.sqrt(0.04), 3.0
# "-LinMid-" (v11): halfway - a 0.13 m/s miss pays 75%.
LIN_MID_STD, LIN_MID_WEIGHT = math.sqrt(0.06), 2.5


def _lin_tight(cfg):
    cfg.rewards["track_linear_velocity"].params["std"] = LIN_TRACK_STD
    cfg.rewards["track_linear_velocity"].weight = LIN_TRACK_WEIGHT
    return cfg


# "-HeadCalm-": head joint speed taxed (sum of squares over the four head joints, rad/s). The v9
# swing — yaw ~0.13 rad at the stride, neck and head pitch ~0.1 / 0.07 at the step — is about
# 1.5 (rad/s)^2, so -0.1 costs ~0.15 a step against ~2 for each tracking term. Off while the
# gait bootstraps, like `_tax_head_speed`: a head pinned from the start may stop the walk being found.
HEAD_CALM_STAGES = [
    {"step": 0, "weight": 0.0},
    {"step": 500 * 24, "weight": -0.02},
    {"step": 1000 * 24, "weight": -0.05},
    {"step": 1500 * 24, "weight": -0.1},
]


def _head_calm(cfg):
    from mjlab.envs.mdp import rewards as mjlab_rewards

    cfg.rewards["head_calm"] = RewardTermCfg(
        func=mjlab_rewards.joint_vel_l2, weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HEAD_JOINT_NAMES)})
    cfg.curriculum["head_calm_weight"] = CurriculumTermCfg(
        func=microduck_mdp.reward_weight,
        params={"reward_name": "head_calm", "weight_stages": HEAD_CALM_STAGES})
    return cfg


# "-Mass920-" (v12): the robot weighs 920 g with the leg bearings, the ToF and the camera in (2026-10-17);
# the model is 850 g and its mass DR scales only the trunk, +-5 %. No per-part weights, so the extra
# 70 g is split as it most likely sits: the head's own DR (HEAD_MASS_RANGE 0.95-1.30, up to +78 g)
# already covers the ToF / camera / fan, and the trunk (206 g) takes the eight hip bearings and the
# rest - +15 to +65 g, x1.07-1.32.
MASS920_TRUNK_RANGE = (1.07, 1.32)


def _mass920(cfg):
    lo, hi = MASS920_TRUNK_RANGE
    cfg.events["randomize_mass_inertia"].params["alpha_range"] = (math.log(lo) / 2.0, math.log(hi) / 2.0)
    return cfg


class head_swing_band:
    """Sum over the head yaw and roll of relu(|err| - band)^2, err the output-side head angle (servo
    + backlash, as `head_pose_tracking` measures it) minus default minus the head command: free
    within the band, the swing past it charged. v9 swings head yaw ~17 deg peak to peak (sim);
    a 0.05 rad band leaves ~6 deg peak to peak free."""

    def __init__(self, cfg, env):
        asset = env.scene[cfg.params.get("asset_cfg", SceneEntityCfg("robot")).name]
        ids, names = asset.find_joints_by_actuator_names(microduck_mdp._NECK_JOINT_PATTERNS)
        name_to_id = {n: i for i, n in enumerate(asset.joint_names)}
        bl = [name_to_id.get(f"passive_{n}_backlash") for n in names]
        self._ids = torch.tensor(ids, device=env.device, dtype=torch.long)
        self._bl = torch.tensor([0 if b is None else b for b in bl], device=env.device, dtype=torch.long)
        self._bl_mask = torch.tensor([0.0 if b is None else 1.0 for b in bl], device=env.device)
        # the command's order is [neck_pitch, head_pitch, head_yaw, head_roll]
        self._cols = torch.tensor([i for i, n in enumerate(names) if n in ("head_yaw", "head_roll")],
                                  device=env.device, dtype=torch.long)
        assert len(self._cols) == 2, names

    def __call__(self, env, command_name="head_pose", band=0.05, asset_cfg=SceneEntityCfg("robot")):
        asset = env.scene[asset_cfg.name]
        cmd = env.command_manager.get_command(command_name)
        q = asset.data.joint_pos
        measured = q[:, self._ids] + q[:, self._bl] * self._bl_mask
        err = (measured - asset.data.default_joint_pos[:, self._ids] - cmd)[:, self._cols]
        return torch.square(torch.clamp(torch.abs(err) - band, min=0.0)).sum(dim=1)

    def reset(self, env_ids=None):
        pass


HEAD_BAND_RAD = 0.05
# Fine-tuning from v9: the env step counter starts again at 0; a short ramp, the gait already exists.
# (a term at weight 0 is skipped by mjlab's reward manager: a small weight from step 0 keeps it computed
# and logged from the first iteration)
HEAD_BAND_STAGES = [
    {"step": 0, "weight": -0.5},
    {"step": 100 * 24, "weight": -15.0},
    {"step": 400 * 24, "weight": -30.0},
    {"step": 700 * 24, "weight": -50.0},
]
HEAD_BAND_SOFT_STAGES = [
    {"step": 0, "weight": -0.3},
    {"step": 100 * 24, "weight": -8.0},
    {"step": 500 * 24, "weight": -20.0},
]


def _head_band(cfg, stages):
    cfg.rewards["head_band"] = RewardTermCfg(
        func=head_swing_band, weight=stages[0]["weight"], params={"command_name": "head_pose", "band": HEAD_BAND_RAD})
    cfg.curriculum["head_band_weight"] = CurriculumTermCfg(
        func=microduck_mdp.reward_weight,
        params={"reward_name": "head_band", "weight_stages": stages})
    return cfg


def phase_command_symmetry(env, obs, actions):
    """The walk's mirror (`symmetry.microduck_vel_symmetry`) for tasks whose twist slot carries a
    phase clock (GroundPick: [cos 2pi phi, sin 2pi phi, 0]) instead of a twist: the walk's table
    negates the slot's y and yaw, which would hand the mirrored robot a different phase. The mirrored
    half keeps the original's."""
    from .symmetry import microduck_vel_symmetry
    aug_obs, aug_actions = microduck_vel_symmetry(env, obs, actions)
    if aug_obs is not None:
        a = aug_obs["actor"]
        b = a.shape[0] // 2
        a[b:, 48:51] = a[:b, 48:51]
    return aug_obs, aug_actions


PHASE_SYMMETRY_CFG = {**SYMMETRY_CFG,
                      "data_augmentation_func": "mjlab_microduck.tasks.hl2915:phase_command_symmetry"}


# "-Cloth-": GroundPick for a cloth on the floor. Upstream's bottom lasts 0.2 s of the 4 s period
# (phase 0.375-0.425) - the mouth servo needs ~0.3 s to close - so 0.6 s (to 0.525), the rise kept
# at ~1.3 s (to 0.85); and a payload of 20-120 g at the mouth on the way up (a sock to a small towel;
# upstream 10-40 g, sized for the XL330).
CLOTH_PHASES = {"descent_end": 0.375, "hold_end": 0.525, "rise_end": 0.85}
CLOTH_PAYLOAD_KG = (0.02, 0.12)
# "-ClothNear-" (v2, 2026-10-18): v1 held the mouth tip 3.2 cm over the floor in the BAM sim (1.4-1.7 cm
# with weak legs) - too high for a cloth. Upstream's proximity term, exp(-(z / 0.10)^2) x 3, still pays
# 90 % at 3.2 cm, and touching costs from 1 N: so a narrow one besides it (0.5 cm target, 1.5 cm wide:
# 1 % at 3.2 cm, 64 % at 1 cm), and a light graze of the jaw allowed (2 N).
CLOTH_NEAR = {"std": 0.015, "target_height": 0.005, "weight": 4.0}
CLOTH_IMPACT_N = 2.0


def _cloth_pick(cfg):
    for term in cfg.rewards.values():
        for k, v in CLOTH_PHASES.items():
            if k in term.params:
                term.params[k] = v
    cfg.events["sample_mouth_payload"].params["min_kg"] = CLOTH_PAYLOAD_KG[0]
    cfg.events["sample_mouth_payload"].params["max_kg"] = CLOTH_PAYLOAD_KG[1]
    return cfg


def _cloth_near(cfg):
    base = cfg.rewards["mouth_ground_proximity"]
    cfg.rewards["mouth_floor_near"] = RewardTermCfg(
        func=base.func, weight=CLOTH_NEAR["weight"],
        params={**base.params, "std": CLOTH_NEAR["std"], "target_height": CLOTH_NEAR["target_height"]})
    cfg.rewards["head_impact_penalty"].params["threshold"] = CLOTH_IMPACT_N
    return cfg


def _sym_rl_cfg(base=MicroduckRlCfg):
    """`base` with the left-right mirror loss on (`symmetry.py`, checked against the rev-a model
    2026-10-12: mirrored joint angles put the feet and head exactly at the mirror image). Its own
    dict: rsl_rl writes the env into it."""
    return replace(base, algorithm=replace(base.algorithm, symmetry_cfg=dict(SYMMETRY_CFG)))


# "-HeadTrack-": head_pose_tracking is exp(-err^2 / std^2) per head joint at std 0.5 rad, weight 2 —
# a 0.3 rad miss still pays 70%. On the robot the stand-up policy reached 20-45% of a gesture.
HEAD_TRACK_STD, HEAD_TRACK_WEIGHT = 0.3, 3.0


def _head_track(cfg):
    cfg.rewards["head_pose_tracking"].params["std"] = HEAD_TRACK_STD
    cfg.rewards["head_pose_tracking"].weight = HEAD_TRACK_WEIGHT
    return cfg


# Leg gain for the "-LegsKp8-" tasks, as a factor on HL2915_KP (16). Every Feetech Microduck
# port runs its servos soft — P 3-6, about 1.1-1.9x the stiffness the XL330 policies trained
# on — where Kp 16 is about 3.8x (2.07 against 0.55 N·m/rad). Stiff servos behind a 20-50 ms
# loop fit what the robot showed: the stand-up policy's 5 Hz self-oscillation, standing that
# shakes 50x harder without the low-pass. The bench fit holds at Kp 8 (MAE 0.55°, as at 16).
# Deploy with [bus] position_kp = 8, position_kp_head = 16.
LEGS_KP_BASE = {r"^(left|right)_": 0.5}


def legs_kp_base(kp):
    """`kp_base` for the leg servos at register-50 value `kp` (the head stays at HL2915_KP).
    Deploy with [bus] position_kp = kp, position_kp_head = 16."""
    from mjlab_microduck.robot.hl2915_constants import HL2915_KP
    return {r"^(left|right)_": kp / HL2915_KP}


def _build(task_id, make, kw, robot, backlash, play):
    cfg = make(play=play, **kw) if play else make(**kw)
    if backlash is None:
        cfg = make_hl2915_variant(cfg, robot)
    else:
        cfg = _low_pass(_gate_air_time(_widen_dr(make_backlash_variant(cfg, backlash))))
        cfg = _randomize_backlash(cfg)
    if "-StandUp-" in task_id:
        cfg = _tax_head_speed(cfg)
    if "-LowFriction-" in task_id or "-Slippery-" in task_id or "-WideFriction-" in task_id:
        cfg = _low_friction(cfg, slippery="-Slippery-" in task_id, wide="-WideFriction-" in task_id)
    if "-Delay50-" in task_id:
        cfg = _long_delay(cfg)
    legs_kp = re.search(r"-LegsKp(\d+)-", task_id)
    if legs_kp:
        cfg.events["randomize_joint_actuators"].params["kp_base"] = legs_kp_base(int(legs_kp.group(1)))
    if "-V6" in task_id:
        cfg = _v6(cfg, lowpass="-V6LP-" in task_id, gain_dr="-NoGainDR-" not in task_id)
    if "-YawTight-" in task_id:
        cfg = _tight_yaw(cfg)
    if "-HeadTrack-" in task_id:
        cfg = _head_track(cfg)
    if "-YawDrift-" in task_id:
        cfg = _yaw_drift(cfg)
    if "-YawDriftLite-" in task_id:
        cfg = _yaw_drift(cfg)
        cfg.rewards["yaw_drift"].weight = YAW_DRIFT_LITE_WEIGHT
    if "-LinTight-" in task_id:
        cfg = _lin_tight(cfg)
    if "-LinMid-" in task_id:
        cfg.rewards["track_linear_velocity"].params["std"] = LIN_MID_STD
        cfg.rewards["track_linear_velocity"].weight = LIN_MID_WEIGHT
    if "-HeadCalm-" in task_id:
        cfg = _head_calm(cfg)
    if "-Mass920-" in task_id:
        cfg = _mass920(cfg)
    if "-HeadBand-" in task_id:
        cfg = _head_band(cfg, HEAD_BAND_STAGES)
    if "-HeadBandSoft-" in task_id:
        cfg = _head_band(cfg, HEAD_BAND_SOFT_STAGES)
    if "-Cloth-" in task_id:
        cfg = _cloth_pick(cfg)
    if "-ClothNear-" in task_id:
        cfg = _cloth_near(cfg)
    return cfg


for _id, _make, _kw, _rl, _robot, _backlash in _TASKS:
    env = _build(_id, _make, _kw, _robot, _backlash, play=False)
    play = _build(_id, _make, _kw, _robot, _backlash, play=True)
    if _rl is None:  # the -Sym- tasks
        _rl = _sym_rl_cfg()
    elif _rl == "velstand-sym":
        _rl = _sym_rl_cfg(MicroduckVelStandRlCfg)
    elif _rl == "sitstand-sym":
        _rl = _sym_rl_cfg(MicroduckSitStandRlCfg)
    elif _rl == "standup-sym":
        _rl = _sym_rl_cfg(MicroduckStandUpRlCfg)
    elif _rl == "groundpick-sym":
        _rl = replace(MicroduckGroundPickRlCfg, algorithm=replace(
            MicroduckGroundPickRlCfg.algorithm, symmetry_cfg=dict(PHASE_SYMMETRY_CFG)))
    register_mjlab_task(task_id=_id, env_cfg=env, play_env_cfg=play, rl_cfg=_rl,
                        runner_cls=MicroduckOnPolicyRunner)
