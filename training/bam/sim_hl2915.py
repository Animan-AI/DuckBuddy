# -*- coding: utf-8 -*-
"""在 MuJoCo 里用 BAM 辨识出的 HL-2915 执行器回放策略 —— 复现 / 预测实机行为。

和 `training/sim_walk.py` 用同一套观测 / 动作约定（直接 import 它），区别只在执行器：
MJCF 里的 position 执行器换成 motor，每个物理步由 BAM 的 MujocoController 按
"舵机内部 P 环 + 限流 + 反电动势 + 摩擦" 算力矩，Kp 用寄存器 50 的值（腿 / 头分开）。

    # 需要 bam（better_actuator_models，与服务器同版本）在 PYTHONPATH 上
    python training/bam/sim_hl2915.py --params m4.json --policy policies/alpha_stand.onnx \
        --kp 32 --kp-head 32 --action-scale 1.0 --duration 6 --video kp32.mp4
    python training/bam/sim_hl2915.py ... --position     # 对照：原版 XL330 position 执行器

指标里 `jitter` 是相邻两拍动作差的均方根（踢腿 / 抽搐时会很大），`lean` 是后 2/3 时段
躯干俯仰均值（正 = 前倾）。
"""
import argparse
import os
import sys
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "training"))

import sim_walk as sw  # noqa: E402

HEAD = {"neck_pitch", "head_pitch", "head_yaw", "head_roll"}


HEAD_BODIES = ("neck", "neck_pitch", "yaw_roll_motion", "jaw_soft")

# Starting poses for --init other than "stand" / "sit": trunk (pitch, roll), radians.
# Positive pitch tips the nose down, so face_down lies on the belly.
INIT_TILT = {
    "face_down": (np.pi / 2, 0.0),
    "face_up": (-np.pi / 2, 0.0),
    "face_up_rolled": (-np.pi / 2 + 0.3, 0.6),
    "side_left": (0.0, -np.pi / 2),
    "side_right": (0.0, np.pi / 2),
}


def backlash_halves(text, seed=0):
    """`--backlash-deg`: "" / "0" = off, "0.4" = +-0.4° on every servo, "0.3:1.0" = each joint's
    half-width drawn from that range (seeded), as the v4 training draws it per episode.
    Returns per-joint half-widths in radians (POLICY_JOINTS order), or None."""
    if not text or float(text.split(":")[-1]) == 0.0:
        return None
    if ":" in text:
        lo, hi = (float(x) for x in text.split(":"))
        deg = np.random.default_rng(seed).uniform(lo, hi, len(sw.POLICY_JOINTS))
    else:
        deg = np.full(len(sw.POLICY_JOINTS), float(text))
    return np.radians(deg)


def add_backlash(spec, halves):
    """Gear play as the training's backlash models have it (microduck_rl add_backlash.py): an
    unactuated `passive_<joint>_backlash` hinge right after each servo joint in the same body,
    range +-half, near-rigid limit. Keyframes get a 0 inserted for each new joint."""
    import mujoco
    old = spec.compile()
    after = sorted(old.jnt_qposadr[old.joint(n).id] + 1 for n in sw.POLICY_JOINTS)
    keys = [np.insert(np.asarray(k.qpos), after, 0.0) for k in spec.keys]
    for name, half in zip(sw.POLICY_JOINTS, halves):
        body = next(b for b in spec.bodies if any(j.name == name for j in b.joints))
        servo = next(j for j in body.joints if j.name == name)
        assert body.joints[-1].name == name, f"{name} is not the last joint of {body.name}"
        body.add_joint(name=f"passive_{name}_backlash", type=mujoco.mjtJoint.mjJNT_HINGE,
                       axis=servo.axis, pos=servo.pos, limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                       range=[-half, half], damping=0.01, frictionloss=0.0, armature=0.001,
                       solref_limit=[0.01, 1.0], solimp_limit=[0.95, 0.999, 0.0001, 0.5, 2.0])
    for k, q in zip(spec.keys, keys):
        k.qpos = q


def load_scene(path, motor, timestep, head_mass_scale=1.0, head_com_forward=0.0, backlash=None, friction=None,
               edit=None):
    """`edit(spec)`, if given, runs first (e.g. `replay_delay_hl2915.py` fixes the base)."""
    import mujoco
    spec = mujoco.MjSpec.from_file(path)
    spec.option.timestep = timestep
    if edit is not None:
        edit(spec)
    if backlash is not None:
        add_backlash(spec, backlash)
    if friction is not None:
        # a contact takes the larger of its two geoms' friction, so set them all
        for g in spec.geoms:
            g.friction[0] = friction
    # Head variations, as the HL-2915 training randomizes them: mass and inertia scaled
    # together, and the big head body's CoM moved forward (its -z is forward at home).
    for b in spec.bodies:
        if b.name in HEAD_BODIES and head_mass_scale != 1.0:
            b.mass = b.mass * head_mass_scale
            b.fullinertia = [x * head_mass_scale for x in b.fullinertia]
        if b.name == "jaw_soft" and head_com_forward:
            ipos = list(b.ipos)
            ipos[2] -= head_com_forward
            b.ipos = ipos
    if motor:
        for a in spec.actuators:
            a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            a.gainprm[:] = 0
            a.gainprm[0] = 1.0
            a.biastype = mujoco.mjtBias.mjBIAS_NONE
            a.biasprm[:] = 0
            a.ctrllimited = 0
            a.forcelimited = 0
    model = spec.compile()
    return model, mujoco.MjData(model)


def run(a, scene, policy, cmd):
    import mujoco
    import onnxruntime as ort

    backlash = backlash_halves(getattr(a, "backlash_deg", ""), getattr(a, "backlash_seed", 0))
    model, data = load_scene(scene, not a.position, a.timestep, a.head_mass_scale,
                             a.head_com_forward_mm / 1000.0, backlash, getattr(a, "friction", None))
    jid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in sw.POLICY_JOINTS]
    aid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in sw.POLICY_JOINTS]
    qadr = np.array([model.jnt_qposadr[i] for i in jid])
    vadr = np.array([model.jnt_dofadr[i] for i in jid])
    # The encoder sits on the output side of the gear play: the policy, robot.state and the
    # servo's own P loop all see servo + backlash (`enc_q`), back-EMF stays motor-side.
    # A zero-calibration error: the joint physically sits `zero_err` past what its encoder
    # reads, so the policy sees q - zero_err and every target lands zero_err further on.
    zero_err = np.zeros(len(sw.POLICY_JOINTS))
    for item in filter(None, getattr(a, "zero_error_deg", "").split(",")):
        name, value = item.split("=")
        zero_err[sw.POLICY_JOINTS.index(name.strip())] = np.radians(float(value))
    if backlash is not None:
        bid = [model.joint(f"passive_{n}_backlash").id for n in sw.POLICY_JOINTS]
        bqadr = np.array([model.jnt_qposadr[i] for i in bid])
        bvadr = np.array([model.jnt_dofadr[i] for i in bid])

        def enc_q():
            return data.qpos[qadr] + data.qpos[bqadr] - zero_err

        def enc_dq():
            return data.qvel[vadr] + data.qvel[bvadr]
    else:
        def enc_q():
            return data.qpos[qadr] - zero_err

        def enc_dq():
            return data.qvel[vadr]
    imu_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
    trunk = model.site_bodyid[imu_site]
    mouth_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "mouth_tip")  # -1 if the model has none
    gyro_adr = next(model.sensor_adr[s] for s in range(model.nsensor)
                    if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, s) == "imu_ang_vel")

    ctrl = None
    if not a.position:
        import hl2915  # noqa: F401
        from bam.model import load_model
        from bam.mujoco import MujocoController
        bam_model = load_model(a.params)
        act = bam_model.actuator
        act.vin = a.vin
        act.max_current = a.current_ma / 1000.0
        act.kp = np.array([a.kp_head if n in HEAD else a.kp for n in sw.POLICY_JOINTS], dtype=float)
        # Per-joint gain multipliers, e.g. "left_hip_pitch=0.6,right_hip_pitch=0.8".
        for item in filter(None, a.kp_scale.split(",")):
            name, value = item.split("=")
            act.kp[sw.POLICY_JOINTS.index(name.strip())] *= float(value)
        # 构造时会 mj_setConst，把 data 重置到 qpos0 —— 所以必须在摆好姿态之前建
        ctrl = MujocoController(bam_model, sw.POLICY_JOINTS, model, data)
        if backlash is not None:
            # same as microduck_rl's BacklashEncoderBamActuator: only the P loop's position
            # reads through the play (MujocoController.update hands it the servo joint alone)
            control = act.compute_control
            act.compute_control = lambda target, q, dq, dt: control(target, q + data.qpos[bqadr], dq, dt)

    mujoco.mj_resetData(model, data)
    key = next((k for k in range(model.nkey) if np.allclose(model.key_qpos[k][qadr], sw.HOME_14, atol=1e-4)), None)
    if key is not None:
        mujoco.mj_resetDataKeyframe(model, data, key)
    else:
        data.qpos[qadr] = sw.HOME_14
        data.qpos[2] = 0.12
    init = getattr(a, "init", "stand")
    if init == "sit":
        mujoco.mj_resetDataKeyframe(model, data, model.key("SIT").id)
    elif init != "stand":
        # Lying: the trunk turned onto its front / back / side, dropped from just above the
        # floor; the settle phase below then holds the standing joint pose, as robotd's
        # limp-fall sequence ramps a landed robot to it before handing over.
        pitch, roll = INIT_TILT[init]
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, [roll, pitch, 0.0], "xyz")
        data.qpos[3:7] = quat
        data.qpos[2] = 0.09
    mujoco.mj_forward(model, data)
    if ctrl is not None:
        ctrl.reset(data.qpos)

    sess = ort.InferenceSession(policy, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    substeps = int(round((1.0 / sw.CONTROL_HZ) / model.opt.timestep))
    delay = int(round(a.delay_ms / 1000.0 / model.opt.timestep))
    pending = deque([sw.HOME_14.copy()] * (delay + 1), maxlen=delay + 1)
    # The robot's IMU path: the reading the policy gets is `imu_lag_ms` old (bus timing, the chip's
    # fusion), and robotd takes a component-wise median of it with the previous two ticks'
    # (duck-control imu.rs). Neither is in the training beyond its 0-20 ms observation delay.
    imu_lag = int(round(getattr(a, "imu_lag_ms", 0.0) / 1000.0 / model.opt.timestep))
    imu_hist = deque(maxlen=imu_lag + 1)
    hold_ref = hold_yaw = None
    hold_ema = 0.0
    med_hist = deque(maxlen=3)
    # robotd's start of a walk: the `stand` network holds the robot at standing gain for
    # `pre_s`, then the walk network takes over mid-stance (last action carried across).
    pre = None
    if getattr(a, "pre_policy", None):
        pre = ort.InferenceSession(a.pre_policy, providers=["CPUExecutionProvider"])
    pre_ticks = int(round(getattr(a, "pre_s", 0.0) * sw.CONTROL_HZ)) if pre is not None else 0
    kp_walk = act.kp.copy() if ctrl is not None else None
    last = np.zeros(sw.ACTION_LEN, dtype=np.float32)
    prev_target = None

    # Pushes as the training gives them (mdp.push_by_setting_velocity, +-0.3 m/s every 3-6 s):
    # at time t the trunk's world-frame linear velocity jumps by (vx, vy).
    pushes = {int(round(t * sw.CONTROL_HZ)): (vx, vy) for t, vx, vy in parse_pushes(getattr(a, "push", ""))}
    shown = None  # (vx, vy, ticks left) of the push arrow drawn in the video

    renderer = frames = None
    if a.video:
        w, h = (int(x) for x in getattr(a, "video_size", "640x480").split("x"))
        model.vis.global_.offwidth, model.vis.global_.offheight = max(w, 640), max(h, 480)
        renderer = mujoco.Renderer(model, height=h, width=w)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = (getattr(a, "cam_distance", 0.7), getattr(a, "cam_elevation", -10.0),
                                                    getattr(a, "cam_azimuth", 120.0))
        frames = []
        cam_yaw = None

    # 先像 robotd init 那样用硬刚度在 home pose 站稳，再交给策略
    if ctrl is not None and a.settle > 0:
        kp_run = act.kp.copy()
        act.kp = np.full_like(kp_run, a.settle_kp, dtype=float)
        hold = data.qpos[qadr].copy() if init == "sit" else sw.HOME_14 + zero_err
        for n, q in zip(sw.POLICY_JOINTS, hold):
            ctrl.set_q_target(n, q)
        for _ in range(int(round(a.settle / model.opt.timestep))):
            ctrl.update()
            mujoco.mj_step(model, data)
        act.kp = kp_run
    elif ctrl is None and a.settle > 0:
        data.ctrl[aid] = sw.HOME_14 + zero_err
        for _ in range(int(round(a.settle / model.opt.timestep))):
            mujoco.mj_step(model, data)
    x0 = float(data.qpos[0])

    log = {k: [] for k in ("t", "pitch", "roll", "yaw", "z", "x", "y", "jitter", "act", "tau", "err", "target",
                           "q", "dq", "gz", "gyro", "g", "head_f", "tau_sq")}
    # Torque off on a schedule (`a.limp_fn(t) -> bool`, robotd's rest): P gain zero, the servo's
    # friction left. On the way back robotd blends the targets from where the joints lie over
    # `a.limp_blend_s` (REST_LEG_BLEND_S).
    limp_fn = getattr(a, "limp_fn", None)
    limp_blend_s = getattr(a, "limp_blend_s", 0.8)
    was_limp, blend_from, blend_t = False, None, 0.0
    # Head-floor contact force (sum of normal forces, N): every body under the neck.
    head_bodies = set()
    neck = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "neck")
    if neck >= 0:
        for b in range(model.nbody):
            p_ = b
            while p_ > 0 and p_ != neck:
                p_ = model.body_parentid[p_]
            if p_ == neck:
                head_bodies.add(b)
    cforce = np.zeros(6)
    fell = None
    for step in range(int(round(a.duration * sw.CONTROL_HZ))):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, data.xquat[trunk])
        R = R.reshape(3, 3)
        g = R.T @ np.array([0.0, 0.0, -1.0])
        if a.ramp_s > 0:
            k = min(1.0, (max(0, step - pre_ticks) / sw.CONTROL_HZ) / a.ramp_s)
            cmd = dict(cmd, twist=[a.vx * k, 0.0, 0.0])
        if step < pre_ticks:
            cmd = dict(cmd, twist=[0.0, 0.0, 0.0])
        if getattr(a, "cmd_fn", None) is not None:
            cmd = a.cmd_fn(step / sw.CONTROL_HZ)
        # robotd's heading hold (`[control] heading_hold_*`, duck_control::heading): while walking
        # straight, vyaw = -gain * wrap(filtered yaw - yaw at the start), clamped, then robotd's
        # twist EMA (cmd_alpha 0.2). `heading_hold_tau` low-passes the heading first (0 = off).
        hold_gain = getattr(a, "heading_hold_gain", 0.0)
        if hold_gain > 0 and step >= pre_ticks and abs(cmd["twist"][0]) > 0.01:
            yaw_now = np.arctan2(R[1, 0], R[0, 0])
            if hold_ref is None:
                hold_ref = hold_yaw = yaw_now
            tau = getattr(a, "heading_hold_tau", 0.0)
            err_raw = (yaw_now - hold_yaw + np.pi) % (2 * np.pi) - np.pi
            hold_yaw = hold_yaw + (err_raw if tau <= 0 else err_raw * min(1.0, 1.0 / (sw.CONTROL_HZ * tau)))
            err = (hold_yaw - hold_ref + np.pi) % (2 * np.pi) - np.pi
            dead = np.radians(getattr(a, "heading_hold_deadband_deg", 0.0))
            err = np.sign(err) * max(0.0, abs(err) - dead)
            limit = getattr(a, "heading_hold_max", 0.5)
            target = float(np.clip(-hold_gain * err, -limit, limit))
            hold_ema += 0.2 * (target - hold_ema)
            cmd = dict(cmd, twist=[cmd["twist"][0], cmd["twist"][1], hold_ema])
            log.setdefault("hold_vyaw", []).append(hold_ema)
        if not imu_hist:
            imu_hist.append((data.sensordata[gyro_adr:gyro_adr + 3].copy(), g.copy()))
        gyro_obs, g_obs = imu_hist[0]
        if getattr(a, "imu_median", False):
            med_hist.append((gyro_obs, g_obs))
            gyro_obs = np.median([m[0] for m in med_hist], axis=0) if len(med_hist) == 3 else gyro_obs
            g_obs = np.median([m[1] for m in med_hist], axis=0) if len(med_hist) == 3 else g_obs
        off = np.radians(getattr(a, "imu_pitch_offset_deg", 0.0))
        if off:
            # an IMU reading `off` further back (nose up) than the trunk really is
            g_obs = np.array([g_obs[0] * np.cos(off) + g_obs[2] * np.sin(off), g_obs[1],
                              -g_obs[0] * np.sin(off) + g_obs[2] * np.cos(off)])
        obs = sw.build_obs(gyro_obs, g_obs, enc_q(), enc_dq(), last, cmd)
        net = pre if step < pre_ticks else sess
        if ctrl is not None and pre is not None:
            act.kp = kp_walk * (getattr(a, "pre_kp_ratio", 1.0) if step < pre_ticks else 1.0)
        action = net.run(None, {net.get_inputs()[0].name: obs[None, :]})[0][0].astype(np.float32)
        jit = float(np.sqrt(np.mean((action - last) ** 2)))
        last = action
        target = sw.HOME_14 + a.action_scale * action
        # robotd's target low-pass ([policy] head_lowpass / legs_lowpass), 1.0 = off
        if prev_target is not None:
            alpha = np.array([getattr(a, "head_lowpass", 1.0) if n in HEAD else getattr(a, "legs_lowpass", 1.0)
                              for n in sw.POLICY_JOINTS])
            target = alpha * target + (1.0 - alpha) * prev_target
        prev_target = target.copy()
        limp = bool(limp_fn(step / sw.CONTROL_HZ)) if limp_fn is not None else False
        if was_limp and not limp:
            blend_from, blend_t = enc_q().copy(), 0.0
        was_limp = limp
        if blend_from is not None and not limp:
            k = min(1.0, blend_t / limp_blend_s)
            target = blend_from + k * (target - blend_from)
            blend_t += 1.0 / sw.CONTROL_HZ
            if k >= 1.0:
                blend_from = None
        if ctrl is not None and limp_fn is not None:
            act.kp = np.zeros_like(kp_walk) if limp else kp_walk.copy()
        if step in pushes:
            # (vx, vy) is along the robot's heading / to its left, so "forward" stays forward after it turns
            c, s_ = R[0, 0] / np.hypot(R[0, 0], R[1, 0]), R[1, 0] / np.hypot(R[0, 0], R[1, 0])
            bx, by = pushes[step]
            dv = (c * bx - s_ * by, s_ * bx + c * by)
            data.qvel[0:2] += dv
            log.setdefault("push", []).append((step / sw.CONTROL_HZ, bx, by))
            shown = (*dv, int(0.4 * sw.CONTROL_HZ))
        taus, head_f, tau_sq = [], 0.0, 0.0
        for _ in range(substeps):
            pending.append(target)
            cur = pending[0]
            if ctrl is None:
                data.ctrl[aid] = cur + zero_err
            else:
                for i, n in enumerate(sw.POLICY_JOINTS):
                    ctrl.set_q_target(n, cur[i] + zero_err[i])
                ctrl.update()
            mujoco.mj_step(model, data)
            taus.append(np.abs(data.actuator_force[aid]).max())
            tau_sq = max(tau_sq, float((data.actuator_force[aid] ** 2).sum()))
            if head_bodies:
                f_head = 0.0
                for ci in range(data.ncon):
                    c = data.contact[ci]
                    b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
                    if (b1 == 0 and b2 in head_bodies) or (b2 == 0 and b1 in head_bodies):
                        mujoco.mj_contactForce(model, data, ci, cforce)
                        f_head += abs(cforce[0])
                head_f = max(head_f, f_head)
            Rs = np.zeros(9)
            mujoco.mju_quat2Mat(Rs, data.xquat[trunk])
            imu_hist.append((data.sensordata[gyro_adr:gyro_adr + 3].copy(),
                             Rs.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])))
        log["err"].append(np.abs(target - enc_q()))
        log["target"].append(target.copy())
        log["q"].append(enc_q())
        log["dq"].append(enc_dq())
        log["gz"].append(float(g[2]))
        log["g"].append(g.copy())
        log["head_f"].append(head_f)
        log["tau_sq"].append(tau_sq)
        log["gyro"].append(data.sensordata[gyro_adr:gyro_adr + 3].copy())
        if mouth_site >= 0:
            log.setdefault("mouth_z", []).append(float(data.site_xpos[mouth_site][2]))
        pitch = np.arctan2(g[0], np.hypot(g[1], g[2]))
        roll = np.arctan2(-g[1], -g[2])
        t = step / sw.CONTROL_HZ
        for k, v in (("t", t), ("pitch", pitch), ("roll", roll), ("yaw", np.arctan2(R[1, 0], R[0, 0])),
                     ("z", data.qpos[2]), ("x", data.qpos[0]), ("y", data.qpos[1]), ("jitter", jit), ("act", np.abs(action).max()),
                     ("tau", max(taus))):
            log[k].append(float(v))
        if fell is None and (data.qpos[2] < 0.05 or g[2] > -0.5):
            fell = t
        if renderer is not None:
            # follow the trunk on the floor plane only: the gait's bob doesn't shake the camera
            cam.lookat[:] = (data.qpos[0], data.qpos[1], 0.11)
            if getattr(a, "cam_follow_yaw", False):
                # keep the same side of the robot in view as it turns; smoothed (~0.4 s) so the
                # gait's yaw wobble doesn't swing the camera
                yaw_now = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                cam_yaw = yaw_now if cam_yaw is None else cam_yaw + 0.05 * ((yaw_now - cam_yaw + 180) % 360 - 180)
                cam.azimuth = getattr(a, "cam_azimuth", 120.0) + cam_yaw
            renderer.update_scene(data, camera=cam)
            if shown is not None:
                vx, vy, left = shown
                # arrow into the trunk along the push, 1 m/s = 0.5 m long
                end = data.xpos[trunk] + np.array([0.0, 0.0, 0.03])
                start = end - 0.5 * np.array([vx, vy, 0.0])
                scn = renderer.scene
                if scn.ngeom < scn.maxgeom:
                    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                        np.zeros(3), np.eye(3).ravel(), np.array([1.0, 0.2, 0.1, 1.0], dtype=np.float32))
                    mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.008, start, end)
                    scn.ngeom += 1
                shown = (vx, vy, left - 1) if left > 1 else None
            frame = renderer.render()
            if getattr(a, "caption", None):
                frame = caption_frame(frame, a.caption, t, pitch, log.get("push"), fell)
            frames.append(frame)
    if renderer is not None:
        renderer.close()
        import imageio.v2 as iio
        iio.mimsave(a.video, frames, fps=int(sw.CONTROL_HZ * getattr(a, "video_speed", 1.0)), macro_block_size=1)
    pushes_done = log.pop("push", [])
    log.pop("hold_vyaw", None)
    return {k: np.asarray(v) for k, v in log.items()} | {"fell": fell, "pushes": pushes_done}


def parse_pushes(text):
    """`--push "3:0.3,0;5:0,-0.3"` -> [(3.0, 0.3, 0.0), (5.0, 0.0, -0.3)]: time s, dv m/s (forward, left)."""
    out = []
    for item in filter(None, (text or "").split(";")):
        t, v = item.split(":")
        vx, vy = (float(x) for x in v.split(","))
        out.append((float(t), vx, vy))
    return out


_FONT = None


def caption_frame(frame, title, t, pitch, pushes, fell):
    """Title, clock, trunk pitch and the last push burned into the frame's top-left corner."""
    global _FONT
    from PIL import Image, ImageDraw, ImageFont
    if _FONT is None:
        for name in ("msyh.ttc", "simhei.ttf", "DejaVuSans.ttf", "arial.ttf"):
            try:
                _FONT = ImageFont.truetype(name, max(14, frame.shape[0] // 30))
                break
            except OSError:
                continue
        else:
            _FONT = ImageFont.load_default()
    img = Image.fromarray(frame)
    d = ImageDraw.Draw(img)
    lines = [title, f"t {t:4.1f} s   俯仰 {np.degrees(pitch):+5.1f}°"]
    if pushes and t - pushes[-1][0] < 1.0:
        pt, vx, vy = pushes[-1]
        lines.append(f"推 {np.hypot(vx, vy):.2f} m/s  " + {(1, 0): "向前", (-1, 0): "向后", (0, 1): "向左", (0, -1): "向右"}
                     .get((int(np.sign(round(vx, 3))), int(np.sign(round(vy, 3)))), "斜向"))
    if fell is not None:
        lines.append(f"摔倒 @ {fell:.1f} s")
    y = 8
    for line in lines:
        d.text((10, y), line, font=_FONT, fill=(255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0))
        y += int(_FONT.size * 1.3) if hasattr(_FONT, "size") else 16
    return np.asarray(img)


def tracking_report(log):
    """Mean |target - joint| per joint, degrees, after the first second (same as robot.state)."""
    err = np.degrees(log["err"][int(sw.CONTROL_HZ):].mean(0))
    return " ".join(f"{n.replace('left_', 'L').replace('right_', 'R')}:{e:.1f}" for n, e in zip(sw.POLICY_JOINTS, err))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default=os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"))
    ap.add_argument("--policy", default=os.path.join(ROOT, "policies/alpha_stand.onnx"))
    ap.add_argument("--params", help="BAM params JSON（hl2915）")
    ap.add_argument("--position", action="store_true", help="用 MJCF 自带的 position 执行器（原版对照）")
    ap.add_argument("--kp", type=int, default=8, help="腿的寄存器 50")
    ap.add_argument("--kp-head", type=int, default=16, help="颈 / 头的寄存器 50")
    ap.add_argument("--vin", type=float, default=12.0)
    ap.add_argument("--current-ma", type=float, default=1300.0)
    ap.add_argument("--delay-ms", type=float, default=0.0, help="指令到执行器的纯延迟")
    ap.add_argument("--timestep", type=float, default=0.001)
    ap.add_argument("--settle", type=float, default=1.0, help="策略接管前用 --settle-kp 在 home pose 站稳的秒数")
    ap.add_argument("--settle-kp", type=int, default=32)
    ap.add_argument("--action-scale", type=float, default=1.0)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--video")
    ap.add_argument("--imu-lag-ms", type=float, default=0.0, help="age of the IMU reading the policy gets")
    ap.add_argument("--imu-pitch-offset-deg", type=float, default=0.0,
                    help="IMU reads this much further back (nose up) than the trunk; robot 10-09: ~3.3")
    ap.add_argument("--imu-median", action="store_true", help="robotd's median-of-3 on gyro and gravity")
    ap.add_argument("--pre-policy", help="stand network run before the walk (robotd's `stand` slot)")
    ap.add_argument("--pre-s", type=float, default=0.0, help="how long --pre-policy stands before the walk")
    ap.add_argument("--pre-kp-ratio", type=float, default=1.0, help="robotd standing_gain_ratio while it stands")
    ap.add_argument("--video-size", default="640x480")
    ap.add_argument("--cam-azimuth", type=float, default=120.0, help="90 = side view from the robot's right")
    ap.add_argument("--cam-elevation", type=float, default=-10.0)
    ap.add_argument("--cam-distance", type=float, default=0.7)
    ap.add_argument("--cam-follow-yaw", action="store_true", help="turn the camera with the robot's heading")
    ap.add_argument("--caption", help="text burned into the video, with the clock, pitch and pushes")
    ap.add_argument("--video-speed", type=float, default=1.0, help="0.5 = half-speed playback")
    ap.add_argument("--push", default="", help='"t:vx,vy;...": add (vx forward, vy left) m/s to the trunk at t s, as training does')
    ap.add_argument("--label", default="")
    ap.add_argument("--tracking", action="store_true", help="also print per-joint tracking error")
    ap.add_argument("--npz", help="save the per-tick log (targets, joints, pitch...) here")
    ap.add_argument("--kp-scale", default="", help='per-joint gain multipliers: "left_hip_pitch=0.6,..."')
    ap.add_argument("--head-mass-scale", type=float, default=1.0)
    ap.add_argument("--head-com-forward-mm", type=float, default=0.0)
    ap.add_argument("--backlash-deg", default="", help='gear play: "0.4" = +-0.4° per servo, "0.3:1.0" = per joint '
                    'from that range (v4 training), "" = none')
    ap.add_argument("--backlash-seed", type=int, default=0)
    ap.add_argument("--friction", type=float, help="sliding friction of floor and soles (MJCF default 1.0)")
    ap.add_argument("--zero-error-deg", default="", help='zero-calibration error: "left_hip_roll=3.4,..." = the joint '
                    'physically sits that far past what it reads')
    ap.add_argument("--ramp-s", type=float, default=0.0, help="ramp the vx command over this long, like robotd's EMA")
    ap.add_argument("--head-lowpass", type=float, default=1.0, help="robotd target low-pass on the head (1.0 = off)")
    ap.add_argument("--legs-lowpass", type=float, default=1.0, help="robotd target low-pass on the legs (1.0 = off)")
    a = ap.parse_args()
    if not a.position and not a.params:
        ap.error("需要 --params，或用 --position")

    cmd = {"twist": [a.vx, 0.0, 0.0],
           # robotd 的默认头部指令是 0（intents.rs），不是 home 角度
           "head": [0.0, 0.0, 0.0, 0.0],
           "body": [0.0, 0.0, 0.0]}
    log = run(a, a.scene, a.policy, cmd)
    tail = slice(len(log["t"]) // 3, None)
    fell = "—" if log["fell"] is None else f"{log['fell']:.2f}s"
    print(f"{a.label:<28} 摔倒 {fell:>6} | lean {np.degrees(log['pitch'][tail].mean()):+5.1f}° "
          f"(max {np.degrees(np.abs(log['pitch']).max()):4.1f}°) | roll rms {np.degrees(np.sqrt((log['roll'][tail]**2).mean())):4.1f}° "
          f"| z {log['z'][tail].mean():.3f} | jitter {log['jitter'][tail].mean():.3f} | |a|max {log['act'].max():.2f} "
          f"| τmax {log['tau'].max():.2f} N·m | Δx {log['x'][-1] - log['x'][0]:+.2f} m "
          f"| yaw {np.degrees(np.unwrap(log['yaw'])[-1] - log['yaw'][0]):+.0f}°", flush=True)
    if a.tracking:
        print("   tracking err deg:", tracking_report(log), flush=True)
    if a.npz:
        np.savez(a.npz, **{k: v for k, v in log.items() if k != "fell"}, joints=np.array(sw.POLICY_JOINTS))


if __name__ == "__main__":
    main()
