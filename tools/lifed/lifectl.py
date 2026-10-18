# -*- coding: utf-8 -*-
"""Talk to lifed — look at the duck's inner state, or play the senses it does not have yet.

    python3 lifectl.py state                 # one snapshot
    python3 lifectl.py watch                 # live, twice a second
    python3 lifectl.py touch | call [yaw] | person [yaw] | sound [yaw]
    python3 lifectl.py say 晚安              # words, through the local intent matcher
    python3 lifectl.py cmd sleep|wake|rest|greet|quiet|unquiet

yaw is in radians, + to the duck's left. From a PC: ssh orangepi@<ip> python3 ~/lifed/lifectl.py watch
"""
import json
import socket
import sys
import time

SOCK = "/tmp/lifed.sock"


def call(method, params=None):
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(3)
    s.connect(SOCK)
    s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}},
                          ensure_ascii=False) + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        buf += s.recv(65536)
    s.close()
    return json.loads(buf.split(b"\n")[0]).get("result")


def show(s):
    bar = lambda v: "#" * int(round(10 * max(0.0, v))) if v is not None else "-"
    print(f"{s['state']:<10} {s['since_s']:6.1f}s  posture {s['posture']:<8} pad {s['pad']:<7}"
          f" head {'on' if s['head_streaming'] else 'off'}  quiet {s['quiet']}")
    print(f"  energy {s['energy']} (head {s['head_energy']})  max {s['max_c']} C  head {s['head_max_c']} C"
          f"  battery {s['battery_pct']}%  falls {s['falls_recent']}")
    print(f"  social {bar(s['social']):<10} curiosity {bar(s['curiosity']):<10}"
          f" arousal {bar(s['arousal']):<10} valence {s['valence']:+.2f}  mood {s['mood']}")
    for line in s.get("log", [])[-4:]:
        print("   ", line)


def main():
    a = sys.argv[1:] or ["state"]
    if a[0] == "state":
        print(json.dumps(call("life.state"), ensure_ascii=False, indent=1))
    elif a[0] == "watch":
        while True:
            print("\033[2J\033[H", end="")
            show(call("life.state"))
            time.sleep(0.5)
    elif a[0] in ("touch", "call", "person", "sound"):
        ev = {"type": a[0]}
        if len(a) > 1:
            ev["yaw"] = float(a[1])
        print(call("life.event", ev))
    elif a[0] == "say":
        print(call("life.event", {"type": "say", "text": " ".join(a[1:])}))
    elif a[0] == "cmd":
        print(call("life.event", {"type": "command", "name": a[1]}))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
