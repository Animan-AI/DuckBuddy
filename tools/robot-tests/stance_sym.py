#!/usr/bin/env python3
"""Left / right symmetry of the legs while the robot stands (2026-10-17: after a battery swap the user
saw the right hip sit further in). Read-only: robot.state at 50 Hz for a few seconds.

The model mirrors every leg joint with a sign flip (training symmetry.py and the robot's own targets:
hip pitch -31 / +32, knee 74 / -75 seated), so a symmetric stance has left + right = 0 for each pair.
For each pair it prints the mean position and target, left + right of each (the asymmetry), and the
tracking error (position - target) per side; then the trunk's roll and pitch from gravity.
- targets asymmetric: the policy itself leans (IMU roll trim, a shifted centre of mass, ...);
- targets symmetric, positions not: zero offset, assembly or load on one side.
Then per joint the mean |current| and the mean |position change| per frame: a servo hunting in its
gear play (head_yaw 2026-10-15: 0.23 deg a frame against 0.07 commanded) moves a lot on little
current; one holding a steady load (a twist through the foot) draws current and sits still.

    python3 stance_sym.py [seconds]
"""
import json
import math
import socket
import sys

NAMES = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", "neck_pitch",
         "head_pitch", "head_yaw", "head_roll", "mouth", "right_hip_yaw", "right_hip_roll",
         "right_hip_pitch", "right_knee", "right_ankle"]
PAIRS = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"]


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    s = socket.socket(socket.AF_UNIX)
    s.connect("/tmp/robotd.sock")
    s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":50}}\n')
    pos, tgt, grav, cur, policies = [], [], [], [], set()
    for line in s.makefile("r"):
        m = json.loads(line)
        if m.get("method") != "robot.state":
            continue
        p = m["params"]
        pos.append(p["joints"])
        tgt.append(p["targets"])
        grav.append((p.get("safety") or {}).get("gravity") or [0, 0, -1])
        cur.append(p.get("currents_ma") or [0.0] * len(NAMES))
        policies.add(p.get("policy"))
        if len(pos) >= seconds * 50:
            break
    n = len(pos)
    mean = lambda rows, i: math.degrees(sum(r[i] for r in rows) / n)
    print(f"{n} frames, policy {sorted(policies)}")
    print(f"{'pair':10s} {'L pos':>7s} {'R pos':>7s} {'L+R pos':>8s} | {'L tgt':>7s} {'R tgt':>7s} {'L+R tgt':>8s} | "
          f"{'L err':>6s} {'R err':>6s}   (deg)")
    for pair in PAIRS:
        li, ri = NAMES.index("left_" + pair), NAMES.index("right_" + pair)
        lp, rp, lt, rt = mean(pos, li), mean(pos, ri), mean(tgt, li), mean(tgt, ri)
        print(f"{pair:10s} {lp:7.2f} {rp:7.2f} {lp + rp:8.2f} | {lt:7.2f} {rt:7.2f} {lt + rt:8.2f} | "
              f"{lp - lt:6.2f} {rp - rt:6.2f}")
    print()
    print(f"{'joint':16s} {'|I| mA':>7s} {'|dq|/frame':>11s} {'|dtgt|/frame':>12s}")
    for i, name in enumerate(NAMES):
        if name in ("mouth",):
            continue
        amps = sum(abs(c[i]) for c in cur) / n
        dq = sum(abs(pos[k][i] - pos[k - 1][i]) for k in range(1, n)) / (n - 1)
        dt = sum(abs(tgt[k][i] - tgt[k - 1][i]) for k in range(1, n)) / (n - 1)
        print(f"{name:16s} {amps:7.0f} {math.degrees(dq):11.3f} {math.degrees(dt):12.3f}")
    gx = sum(g[0] for g in grav) / n
    gy = sum(g[1] for g in grav) / n
    gz = sum(g[2] for g in grav) / n
    print(f"trunk from gravity: roll {math.degrees(math.atan2(gy, -gz)):+.2f} deg, "
          f"pitch {math.degrees(math.atan2(-gx, math.hypot(gy, gz))):+.2f} deg  (gravity {gx:+.3f} {gy:+.3f} {gz:+.3f})")


if __name__ == "__main__":
    main()
