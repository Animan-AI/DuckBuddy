#!/usr/bin/env python3
"""Cut an odom_log.py recording into walks and say what the odometry made of each (2026-10-17).

A run is a stretch with a twist commanded (|vx|, |vy| or |wz| over a threshold), bounded by the
robot standing still. For each: duration, the odometry's distance and its forward / sideways parts
along the start heading, the heading change, the mean commanded speed, and the hottest joint.
Compare the distance with a tape measure: scale = tape / odometry.

    python3 odom_runs.py ~/robot/recs/odom-cal.jsonl [--since HH:MM]
"""
import argparse
import json
import math
import time


def runs(frames, move=0.02, still_s=0.6):
    """[(start index, end index)] of the walks."""
    out, start, last_move = [], None, None
    for i, f in enumerate(frames):
        req = f.get("req") or [0, 0, 0]
        # a twist commanded, or (frames from before the logger read `move`) the walk policy driving
        moving = any(abs(v) > move for v in req) or (f.get("req") is None and f.get("policy") in ("walk", "settle"))
        if moving:
            if start is None:
                start = i
            last_move = i
        elif start is not None and f["w"] - frames[last_move]["w"] > still_s:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(frames) - 1))
    return out


def describe(frames, a, b):
    # the pose just before the first command, and once it has stood still after the last
    p0, p1 = frames[max(0, a - 1)], frames[b]
    if p0.get("pos") is None or p1.get("pos") is None:
        return None
    dx, dy = p1["pos"][0] - p0["pos"][0], p1["pos"][1] - p0["pos"][1]
    h = p0["yaw"] or 0.0
    fwd = dx * math.cos(h) + dy * math.sin(h)
    side = -dx * math.sin(h) + dy * math.cos(h)
    dyaw = math.degrees((p1["yaw"] - p0["yaw"] + math.pi) % (2 * math.pi) - math.pi)
    seg = frames[a:b]
    vx = sum((f.get("req") or [0])[0] for f in seg) / max(1, len(seg))
    wz = sum((f.get("req") or [0, 0, 0])[2] for f in seg) / max(1, len(seg))
    dur = frames[b]["w"] - frames[a]["w"]
    temps = [f["max_c"] for f in seg if f.get("max_c") is not None]
    return {"at": time.strftime("%H:%M:%S", time.localtime(frames[a]["w"])), "s": round(dur, 1),
            "dist_m": round(math.hypot(dx, dy), 3), "fwd_m": round(fwd, 3), "side_m": round(side, 3),
            "dyaw_deg": round(dyaw, 1), "vx_cmd": round(vx, 3), "wz_cmd": round(wz, 3),
            "max_c": max(temps) if temps else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--since", help="HH:MM, local")
    a = ap.parse_args()
    frames = [json.loads(x) for x in open(a.log, encoding="utf-8") if x.strip()]
    if a.since:
        hh, mm = map(int, a.since.split(":"))
        frames = [f for f in frames if (lambda t: t.tm_hour * 60 + t.tm_min)(time.localtime(f["w"])) >= hh * 60 + mm]
    for i, (s, e) in enumerate(runs(frames), 1):
        d = describe(frames, s, e)
        if d:
            print(i, json.dumps(d))


if __name__ == "__main__":
    main()
