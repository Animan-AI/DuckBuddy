# -*- coding: utf-8 -*-
"""Check the leg zeros against the floor: where the model puts the feet, from the robot's joint
readings and IMU, in a recording (robot.state jsonl) of the robot held still.

Calibration check (`--hand-posed`): torque off, both soles pressed flat on a flat table, both feet
turned to point straight ahead, parallel to a straight edge. Then the true feet are flat and
parallel, and whatever the model makes of them is calibration error:
  - the yaw difference between the feet is the hip-yaw zeros (toe-in the software does not see);
  - the roll difference is the hip-roll zeros (one leg splayed more than the model thinks);
  - the common roll / pitch is the IMU trims (or all of one leg chain - the two can't be told
    apart from one pose; the pitch chains were trimmed on purpose, see deploy's robotd.toml);
  - the pitch difference is the left vs right sagittal chain.
The trunk's own yaw against the straight edge is unknown, so only the yaw *difference* is used.

Without --hand-posed it reports what the model thinks of the feet while the robot stands (the
policy's stand before the first walk), as a reference.

    python training/bam/feet_check_hl2915.py recs/feet_1.jsonl recs/feet_2.jsonl --hand-posed
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "training"))

import sim_hl2915 as sh  # noqa: E402
import sim_walk as sw  # noqa: E402

STATE_TO_POLICY = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14]
# robot.state joint index of each hip, and its [bus] joint_signs entry (deploy/robotd.toml)
HIP = {"left_hip_yaw": (0, -1), "left_hip_roll": (1, -1), "right_hip_yaw": (10, -1), "right_hip_roll": (11, -1)}
COUNTS_PER_RAD = 4096 / (2 * np.pi)
HOME = dict(left_hip_yaw=0, left_hip_roll=-0.0873, left_hip_pitch=-0.4579, left_knee=-0.0049, left_ankle=0.4530,
            right_hip_yaw=0, right_hip_roll=0.0873, right_hip_pitch=0.4579, right_knee=0.0049, right_ankle=-0.4530,
            neck_pitch=0.3491, head_pitch=0.3491, head_yaw=0, head_roll=0)


class Feet:
    def __init__(self):
        import mujoco
        self.mj = mujoco
        self.model, self.data = sh.load_scene(os.path.join(ROOT, "hardware/rev-a/mjcf/scene_walk.xml"), False, 0.001)
        self.qadr = [self.model.jnt_qposadr[self.model.joint(n).id] for n in sw.POLICY_JOINTS]
        self.home = np.array([HOME[n] for n in sw.POLICY_JOINTS])
        self.ref = self._mats(self.home, (0, 0, -1))
        # the soles' own heading at home (the meshes are not quite square to the trunk)
        self.home_yaw = {s: self._sole_heading(s) for s in ("left", "right")}

    def _mats(self, q, gravity):
        g = np.asarray(gravity, float)
        pitch, roll = np.arctan2(g[0], np.hypot(g[1], g[2])), np.arctan2(-g[1], -g[2])
        quat = np.zeros(4)
        self.mj.mju_euler2Quat(quat, [roll, pitch, 0.0], "xyz")
        d = self.data
        d.qpos[:] = self.model.qpos0
        d.qpos[3:7] = quat
        d.qpos[self.qadr] = q
        self.mj.mj_kinematics(self.model, d)
        return {s: d.geom_xmat[self.model.geom(f"{s}_foot_collision").id].reshape(3, 3).copy()
                for s in ("left", "right")}

    def _sole_heading(self, side):
        m, d = self.model, self.data
        g = m.geom(f"{side}_foot_collision").id
        mesh = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T
        _, _, vt = np.linalg.svd(w[:, :2] - w[:, :2].mean(0))
        ax = vt[0] * np.sign(vt[0][0])
        return np.degrees(np.arctan2(ax[1], ax[0]))

    def feet(self, joints, gravity):
        """Per foot (yaw, roll, pitch) in degrees, world frame with the trunk's yaw at 0: yaw + =
        toe pointing left, roll / pitch 0 = sole flat."""
        mats = self._mats(np.asarray(joints)[STATE_TO_POLICY], gravity)
        out = {}
        for s, m in mats.items():
            r = m @ self.ref[s].T
            out[s] = np.array([np.degrees(np.arctan2(r[1, 0], r[0, 0])) + self.home_yaw[s],
                               np.degrees(np.arctan2(r[2, 1], r[2, 2])),
                               np.degrees(np.arctan2(-r[2, 0], np.hypot(r[2, 1], r[2, 2])))])
        return out


def load(path):
    rows = []
    for line in open(path):
        if '"robot.state"' in line:
            try:
                rows.append(json.loads(line)["params"])
            except json.JSONDecodeError:
                pass
    return [r for r in rows if "joints" in r and "safety" in r]


def still(rows):
    """The rows of the stillest second (the hand-posed robot moves while being set down)."""
    j = np.array([r["joints"] for r in rows])
    if len(j) <= 50:
        return rows
    spread = [np.ptp(j[k:k + 50], axis=0).max() for k in range(0, len(j) - 50, 5)]
    k = 5 * int(np.argmin(spread))
    return rows[k:k + 50]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--hand-posed", action="store_true", help="torque off, soles flat and feet parallel")
    a = ap.parse_args()
    feet = Feet()
    print(f"model home pose: sole heading L {feet.home_yaw['left']:+.1f}  R {feet.home_yaw['right']:+.1f} deg "
          f"(toe-in {feet.home_yaw['right'] - feet.home_yaw['left']:+.1f})")
    results = []
    for path in a.recordings:
        rows = load(path)
        if a.hand_posed:
            seg = still(rows)
        else:
            vx = np.array([r.get("move", {}).get("applied", [0])[0] for r in rows])
            i0 = int(np.argmax(vx > 0.0)) if (vx > 0).any() else len(rows)
            seg = [r for r in rows[max(0, i0 - 150):i0] if r.get("policy") == "stand"]
        if len(seg) < 20:
            print(f"{os.path.basename(path)}: no still segment")
            continue
        f = [feet.feet(r["joints"], r["safety"]["gravity"]) for r in seg]
        L = np.mean([x["left"] for x in f], axis=0)
        R = np.mean([x["right"] for x in f], axis=0)
        jitter = np.ptp(np.array([r["joints"] for r in seg]), axis=0).max()
        results.append((L, R))
        print(f"{os.path.basename(path):<24} yaw L {L[0]:+5.1f} R {R[0]:+5.1f} (toe-in {R[0] - L[0]:+5.1f}) | "
              f"roll L {L[1]:+5.1f} R {R[1]:+5.1f} | pitch L {L[2]:+5.1f} R {R[2]:+5.1f} deg | "
              f"joints still to {np.degrees(jitter):.1f} deg")
    if not (a.hand_posed and results):
        return
    L = np.mean([r[0] for r in results], axis=0)
    R = np.mean([r[1] for r in results], axis=0)
    toe_in, splay = R[0] - L[0], (L[1] - R[1]) / 2
    print(f"\nfeet really flat and parallel, so the model is off by:")
    print(f"  toe-in, real minus model {-toe_in:+.1f} deg  (split between the hip yaws)")
    print(f"  roll, left vs right      {L[1] - R[1]:+.1f} deg  (hip rolls)")
    print(f"  roll, both feet          {(L[1] + R[1]) / 2:+.1f} deg  (IMU roll trim, or both leg chains)")
    print(f"  pitch, left vs right     {L[2] - R[2]:+.1f} deg  (sagittal chains)")
    print(f"  pitch, both feet         {(L[2] + R[2]) / 2:+.1f} deg  (IMU pitch trim + chains, trimmed on purpose)")
    # A joint reading too high by e rad needs the offset moved by sign * e * counts/rad
    # (robotd: angle = sign * (raw - center - offset) * 2 pi / 4096). Foot yaw / roll follow the
    # hip yaw with -1 and the hip roll with +1 (both legs, at the home pose).
    err = {"left_hip_yaw": -(-toe_in / 2), "right_hip_yaw": -(toe_in / 2),
           "left_hip_roll": splay, "right_hip_roll": -splay}
    print("\n  [bus] joint_offsets change, counts (1 deg = 11.4):")
    for name, e_deg in err.items():
        idx, sign = HIP[name]
        print(f"    {name:<15} (index {idx:>2}): {sign * np.radians(e_deg) * COUNTS_PER_RAD:+5.0f}")


if __name__ == "__main__":
    main()
