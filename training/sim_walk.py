# -*- coding: utf-8 -*-
"""在本机 MuJoCo 里回放训练出来的 ONNX 策略，看步态并出量化指标。

观测向量的组装**逐字照抄** `duck-control/src/obs.rs` 的布局表 —— 那 61 个下标
是整条链路里最容易错又最不会报错的地方：错了不会崩，只会走出一个看起来
"没训好"的步态。所以这里不重新推导，只搬运：

    0..3     3   gyro，躯干系，rad/s
    3..6     3   投影重力，躯干系，单位向量（直立时 [0,0,-1]）
    6..20   14   关节角 − home pose（不含嘴）
    20..34  14   关节角速度（不含嘴）
    34..48  14   上一次的动作（策略原始输出，未乘 action_scale）
    48..61  13   指令 [vx,vy,vyaw, neck_pitch,head_pitch,head_yaw,head_roll,
                        0,0, body_z, body_roll, body_pitch, 0]

动作施加方式同样照抄 `robotd/src/control.rs`：

    目标角 = home pose + action_scale × action        (action_scale 默认 0.9)

用法:
    python training/sim_walk.py --vx 0.15 --duration 8 --video walk.mp4
    python training/sim_walk.py --sweep                 # 扫多个速度指令
"""
import argparse
import json
import os
import sys

import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ── 从 duck-control 搬来的常量（改这里之前先去改 Rust，再同步过来）──────────
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
MOUTH_INDEX = 9
DEFAULT_POSITION = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
])
# 策略看到的 14 个关节：15 个里去掉嘴
POLICY_JOINTS = [n for i, n in enumerate(JOINT_NAMES) if i != MOUTH_INDEX]
HOME_14 = np.delete(DEFAULT_POSITION, MOUTH_INDEX)

OBS_LEN, ACTION_LEN = 61, 14
CONTROL_HZ = 50.0
ACTION_SCALE = 0.9          # robotd/src/control.rs 的 walking 默认值


def build_obs(gyro, gravity, qpos14, qvel14, last_action, cmd):
    obs = np.zeros(OBS_LEN, dtype=np.float32)
    obs[0:3] = gyro
    obs[3:6] = gravity
    obs[6:20] = qpos14 - HOME_14
    obs[20:34] = qvel14
    obs[34:48] = last_action
    obs[48:51] = cmd["twist"]
    obs[51:55] = cmd["head"]
    obs[55:57] = 0.0                     # body x,y —— 训练里没约束，恒零
    obs[57] = cmd["body"][0]             # body z
    obs[58] = cmd["body"][1]             # body roll
    obs[59] = cmd["body"][2]             # body pitch
    obs[60] = 0.0                        # body yaw —— 恒零
    return obs


def run(scene, policy_path, cmd, duration, video=None, seed=0, action_scale=ACTION_SCALE,
        width=640, height=480, fps=50):
    import mujoco
    import onnxruntime as ort

    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    # 关节 / 执行器索引：按策略顺序取，不靠 MJCF 的声明顺序碰运气
    jid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in POLICY_JOINTS]
    aid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in POLICY_JOINTS]
    if -1 in jid:
        missing = [n for n, i in zip(POLICY_JOINTS, jid) if i == -1]
        raise SystemExit(f"MJCF 里找不到关节: {missing}")
    if -1 in aid:
        missing = [n for n, i in zip(POLICY_JOINTS, aid) if i == -1]
        raise SystemExit(f"MJCF 里找不到执行器: {missing}")
    qadr = np.array([model.jnt_qposadr[i] for i in jid])
    vadr = np.array([model.jnt_dofadr[i] for i in jid])

    imu_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
    trunk_bid = model.site_bodyid[imu_site]
    gyro_adr = None
    for s in range(model.nsensor):
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, s) == "imu_ang_vel":
            gyro_adr = model.sensor_adr[s]
    if gyro_adr is None:
        raise SystemExit("MJCF 里没有 imu_ang_vel 传感器")

    foot_bid = {}
    for side in ("left", "right"):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_foot")
        foot_bid[side] = model.site_bodyid[sid] if sid != -1 else -1

    # 从 home pose 关键帧起步；找不到就手动摆
    mujoco.mj_resetData(model, data)
    key = None
    for k in range(model.nkey):
        if np.allclose(model.key_qpos[k][7:], HOME_14, atol=1e-4):
            key = k
    if key is not None:
        mujoco.mj_resetDataKeyframe(model, data, key)
    else:
        data.qpos[qadr] = HOME_14
        data.qpos[2] = 0.12
    mujoco.mj_forward(model, data)

    sess = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    substeps = int(round((1.0 / CONTROL_HZ) / model.opt.timestep))
    n_ctrl = int(round(duration * CONTROL_HZ))
    last_action = np.zeros(ACTION_LEN, dtype=np.float32)

    renderer = None
    frames = []
    if video:
        renderer = mujoco.Renderer(model, height=height, width=width)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 0.75, -12.0, 135.0

    log = {k: [] for k in ("t", "x", "y", "z", "vx", "vy", "vyaw",
                           "roll", "pitch", "gz", "act_absmax", "lc", "rc", "q")}
    fell_at = None
    every = max(1, int(round(CONTROL_HZ / fps)))

    for step in range(n_ctrl):
        quat = data.xquat[trunk_bid].copy()               # world←body
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        gravity = R.T @ np.array([0.0, 0.0, -1.0])        # 躯干系投影重力
        gyro = data.sensordata[gyro_adr:gyro_adr + 3].copy()

        qpos14 = data.qpos[qadr].copy()
        qvel14 = data.qvel[vadr].copy()
        obs = build_obs(gyro, gravity, qpos14, qvel14, last_action, cmd)

        action = sess.run(None, {in_name: obs[None, :]})[0][0].astype(np.float32)
        last_action = action
        data.ctrl[aid] = HOME_14 + action_scale * action

        for _ in range(substeps):
            mujoco.mj_step(model, data)

        vworld = data.qvel[0:3].copy()
        vbody = R.T @ vworld
        # roll/pitch 由投影重力反解，和机器人上的判据一致
        roll = np.arctan2(-gravity[1], -gravity[2])
        pitch = np.arctan2(gravity[0], np.hypot(gravity[1], gravity[2]))

        contacts = {"left": 0, "right": 0}
        for c in range(data.ncon):
            b1 = model.geom_bodyid[data.contact.geom1[c]]
            b2 = model.geom_bodyid[data.contact.geom2[c]]
            for side, bid in foot_bid.items():
                if bid != -1 and (b1 == bid or b2 == bid):
                    contacts[side] += 1

        t = step / CONTROL_HZ
        log["t"].append(t)
        log["x"].append(float(data.qpos[0]))
        log["y"].append(float(data.qpos[1]))
        log["z"].append(float(data.qpos[2]))
        log["vx"].append(float(vbody[0]))
        log["vy"].append(float(vbody[1]))
        log["vyaw"].append(float(data.qvel[5]))
        log["roll"].append(float(roll))
        log["pitch"].append(float(pitch))
        log["gz"].append(float(gravity[2]))
        log["q"].append(data.qpos[qadr].copy())
        log["act_absmax"].append(float(np.abs(action).max()))
        log["lc"].append(contacts["left"])
        log["rc"].append(contacts["right"])

        if fell_at is None and (data.qpos[2] < 0.05 or gravity[2] > -0.5):
            fell_at = t

        if renderer is not None and step % every == 0:
            cam.lookat[:] = data.qpos[0:3]
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render())

    if renderer is not None:
        renderer.close()
        import imageio.v2 as iio
        iio.mimsave(video, frames, fps=fps // every, macro_block_size=1)

    return {k: np.asarray(v) for k, v in log.items()} | {"fell_at": fell_at}


def summarise(log, cmd, label):
    n = len(log["t"])
    tail = slice(n // 3, n)               # 丢掉起步的前 1/3，看稳态
    vx, vy = log["vx"][tail], log["vy"][tail]
    z, roll, pitch = log["z"][tail], log["roll"][tail], log["pitch"][tail]
    # 步频：左脚触地的上升沿
    lc = (log["lc"] > 0).astype(int)
    edges = int(((lc[1:] == 1) & (lc[:-1] == 0)).sum())
    dur = log["t"][-1] - log["t"][0] if n > 1 else 1.0
    out = {
        "label": label,
        "cmd_vx": cmd["twist"][0], "cmd_vy": cmd["twist"][1], "cmd_vyaw": cmd["twist"][2],
        "vx_mean": float(vx.mean()), "vx_std": float(vx.std()),
        "vy_mean": float(vy.mean()), "vyaw_mean": float(log["vyaw"][tail].mean()),
        "z_mean": float(z.mean()), "z_std": float(z.std()),
        "roll_rms_deg": float(np.degrees(np.sqrt((roll ** 2).mean()))),
        "pitch_rms_deg": float(np.degrees(np.sqrt((pitch ** 2).mean()))),
        "step_hz": edges / dur if dur > 0 else 0.0,
        "act_absmax": float(log["act_absmax"].max()),
        "dist_x": float(log["x"][-1] - log["x"][0]),
        "drift_y": float(log["y"][-1] - log["y"][0]),
        "fell_at": log["fell_at"],
    }
    return out


def fmt(r):
    fell = "—" if r["fell_at"] is None else f"{r['fell_at']:.1f}s ❌"
    return ("%-12s cmd_vx=%+.2f | vx=%+.3f±%.3f  vy=%+.3f vyaw=%+.3f | z=%.3f±%.3f | "
            "roll%.1f° pitch%.1f° | 步频%.2fHz | |a|max=%.2f | Δx=%+.2fm Δy=%+.2fm | 摔倒 %s"
            % (r["label"], r["cmd_vx"], r["vx_mean"], r["vx_std"], r["vy_mean"], r["vyaw_mean"],
               r["z_mean"], r["z_std"], r["roll_rms_deg"], r["pitch_rms_deg"],
               r["step_hz"], r["act_absmax"], r["dist_x"], r["drift_y"], fell))


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--policy", default=os.path.join(root, "training/policies/walk-baseline.onnx"))
    ap.add_argument("--scene", default=os.path.join(root, "hardware/original/mjcf/scene_walk.xml"))
    ap.add_argument("--vx", type=float, default=0.15)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--vyaw", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--action-scale", type=float, default=ACTION_SCALE)
    ap.add_argument("--video", default=None)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    def command(vx, vy, vyaw):
        return {"twist": [vx, vy, vyaw],
                "head": [DEFAULT_POSITION[5], DEFAULT_POSITION[6], 0.0, 0.0],
                "body": [0.0, 0.0, 0.0]}

    cases = []
    if a.sweep:
        cases = [("stand", 0.0, 0.0, 0.0),
                 ("fwd 0.05", 0.05, 0.0, 0.0),
                 ("fwd 0.10", 0.10, 0.0, 0.0),
                 ("fwd 0.15", 0.15, 0.0, 0.0),
                 ("fwd 0.20", 0.20, 0.0, 0.0),
                 ("fwd 0.30", 0.30, 0.0, 0.0),
                 ("side 0.10", 0.0, 0.10, 0.0),
                 ("yaw 0.50", 0.0, 0.0, 0.50)]
    else:
        cases = [("run", a.vx, a.vy, a.vyaw)]

    results = []
    for i, (label, vx, vy, vyaw) in enumerate(cases):
        cmd = command(vx, vy, vyaw)
        vid = a.video if (a.video and len(cases) == 1) else (
            a.video.replace(".mp4", f"-{label.replace(' ', '')}.mp4") if a.video else None)
        log = run(a.scene, a.policy, cmd, a.duration, video=vid,
                  action_scale=a.action_scale)
        r = summarise(log, cmd, label)
        results.append(r)
        print(fmt(r), flush=True)

    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(results, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
