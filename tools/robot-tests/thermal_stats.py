"""Servo heating from robot.state recordings that carry `temps_c` (robotd 2026-10-10 and later).

    python tools/robot-tests/thermal_stats.py recs/thermal_walk1.jsonl [more.jsonl ...]

Per servo: the temperature at the start and the end, the heating rate over the run (C/min,
fitted on the once-a-second samples), and how long the hottest servo took to reach 55 / 60 /
65 C. Also when robotd's thermal guard stopped the walk (`thermal` in `limited_by`).
"""
import json
import os
import sys

import numpy as np

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]


def load(path):
    rows = []
    for line in open(path):
        if '"robot.state"' in line:
            try:
                rows.append(json.loads(line)["params"])
            except json.JSONDecodeError:
                pass  # a truncated last line
    return [r for r in rows if r.get("temps_c")]


def main():
    for path in sys.argv[1:]:
        rows = load(path)
        if len(rows) < 10:
            print(f"{os.path.basename(path)}: no temps_c in this recording (robotd before 2026-10-10?)")
            continue
        t = np.array([r["t"] for r in rows]) - rows[0]["t"]
        temps = np.array([r["temps_c"] for r in rows])
        walking = np.array([abs(r["move"]["applied"][0]) > 0.05 for r in rows])
        thermal = [r["t"] - rows[0]["t"] for r in rows if "thermal" in r["move"].get("limited_by", [])]
        print(f"== {os.path.basename(path)}: {t[-1]:.0f} s, walking {walking.mean():.0%} of it"
              + (f", walk stopped by the thermal guard at {thermal[0]:.0f} s" if thermal else ""))
        print("   servo             start   end   C/min")
        order = np.argsort(-temps[-1])
        for j in order:
            rate = np.polyfit(t / 60.0, temps[:, j], 1)[0] if t[-1] > 20 else float("nan")
            print(f"   {JOINTS[j]:<16} {temps[0, j]:5.0f} {temps[-1, j]:5.0f}  {rate:+6.1f}")
        hottest = temps.max(axis=1)
        for limit in (55, 60, 65):
            hit = np.argmax(hottest >= limit) if (hottest >= limit).any() else None
            print(f"   hottest servo reached {limit} C " + ("never" if hit is None else f"after {t[hit]:.0f} s"))


if __name__ == "__main__":
    main()
