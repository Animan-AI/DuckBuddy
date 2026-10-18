#!/usr/bin/env python3
"""Record robotd's odometry while someone walks the robot (odometry calibration, 2026-10-17).

Read-only: subscribes to robot.state and writes one JSON line per frame (wall time, robot time,
policy, odom position / yaw, commanded and applied twist, hottest joint). Commands nothing.

    python3 odom_log.py ~/robot/recs/odom-cal.jsonl [--hz 20]
then `odom_runs.py` cuts it into runs.
"""
import argparse
import json
import socket
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--sock", default="/tmp/robotd.sock")
    ap.add_argument("--hz", type=int, default=20)
    a = ap.parse_args()
    while True:
        try:
            s = socket.socket(socket.AF_UNIX)
            s.connect(a.sock)
            s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": "robot.subscribe",
                                   "params": {"hz": a.hz}}) + "\n").encode())
            f = s.makefile("r", encoding="utf-8")
            with open(a.out, "a", encoding="utf-8") as out:
                for line in f:
                    m = json.loads(line)
                    if m.get("method") != "robot.state":
                        continue
                    st = m["params"]
                    od = st.get("odom") or {}
                    mv = st.get("move") or st.get("movement") or {}
                    temps = st.get("temps_c") or []
                    out.write(json.dumps({
                        "w": round(time.time(), 3), "t": st.get("t"), "policy": st.get("policy"),
                        "pos": od.get("position"), "yaw": od.get("yaw"),
                        "req": mv.get("requested"), "app": mv.get("applied"),
                        "max_c": max(temps) if temps else None}) + "\n")
                    out.flush()
        except (OSError, ValueError) as e:
            print(time.strftime("%H:%M:%S"), "robotd:", repr(e), flush=True)
            time.sleep(2)


if __name__ == "__main__":
    main()
