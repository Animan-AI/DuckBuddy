#!/bin/bash
# trial.sh <tag> [vx m/s] [seconds]: one walking trial. Records robot.state to recs/t_<tag>.jsonl:
# enable -> stand 1.5 s -> (only if cool and standing quietly) walk vx (0.2) for the given time (4 s) -> stand 7 s.
#
# Refuses to walk when the CPU is above 80 C or a servo above 70 C (65 until 2026-10-13): on 2026-10-09, without the
# head fan, the OPi got too hot to touch and the bus started failing mid-trial.
cd ~/robot
VX=${2:-0.2}
SECS=${3:-4}
REC=$(python3 -c "print($SECS + 9)")
# QUIET_MAX: the standing check's limit, deg/tick of mean target change. Without robotd's target
# low-pass the stand-up policies chatter 2.5-3.5 deg/tick while standing still (2026-10-10).
QUIET_MAX=${QUIET_MAX:-1.5}
CPU_MAX=80
SERVO_MAX=70

health_ok() {
    python3 - "$CPU_MAX" "$SERVO_MAX" <<'PY'
import json, socket, sys
cpu_max, servo_max = float(sys.argv[1]), float(sys.argv[2])
s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect("/tmp/robotd.sock")
s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.health"}\n'); b = b""
while b"\n" not in b: b += s.recv(65536)
r = json.loads(b.split(b"\n")[0]).get("result", {})
bat, mot = r.get("battery") or {}, r.get("motors") or {}
cpu, servo = r.get("cpu_temp_c") or 0.0, mot.get("max_c") or 0.0
print(f"battery {bat.get('volts', 0):.2f} V  cpu {cpu:.0f} C  servos max {servo:.0f} C ({mot.get('hottest', '?')})")
sys.exit(0 if cpu < cpu_max and servo < servo_max else 1)
PY
}

echo -n "before: "
if ! health_ok; then
    echo "TOO HOT (limits: cpu ${CPU_MAX} C, servos ${SERVO_MAX} C) - let it cool, trial not run"
    exit 1
fi
(python3 /tmp/rpc.py record "$REC" "recs/t_$1.jsonl" > "/tmp/rec_$1.out" 2>&1 &)
sleep 0.3; python3 /tmp/rpc.py enable >/dev/null
# Wait (up to 4 s) for the standing policy to have the robot, then 1 s more: measured on the held
# pose instead, the gate read 0.00 deg/tick and passed a robot no policy was balancing yet (2026-10-11).
for _ in $(seq 1 16); do
    python3 /tmp/rpc.py record 0.25 /tmp/quiet.jsonl >/dev/null
    python3 -c 'import json, sys; r = [json.loads(l)["params"] for l in open("/tmp/quiet.jsonl")]; sys.exit(0 if r and r[-1].get("policy") == "stand" else 1)' && break
done
sleep 1.0
# quiet-standing gate: the standing policy driving, mean |target step| over 0.5 s under QUIET_MAX, upright
python3 /tmp/rpc.py record 0.5 /tmp/quiet.jsonl >/dev/null
if python3 - "$QUIET_MAX" <<'PY'
import json, math, sys
rows = [json.loads(l)["params"] for l in open("/tmp/quiet.jsonl")]
policies = {r.get("policy") for r in rows}
if policies != {"stand"}:
    print("standing check: policy %s, not the standing policy" % sorted(map(str, policies)))
    sys.exit(1)
steps = [abs(math.degrees(a - b)) for r0, r1 in zip(rows, rows[1:]) for a, b in zip(r1["targets"], r0["targets"])]
m = sum(steps) / len(steps); gz = rows[-1]["safety"]["gravity"][2]
print("standing check: mean target step %.2f deg/tick, gravity z %.2f" % (m, gz))
sys.exit(0 if m < float(sys.argv[1]) and gz < -0.94 else 1)
PY
then python3 /tmp/rpc.py move "$VX" "$SECS"
else echo "NOT WALKING: robot not standing quietly"
fi
sleep 7; cat "/tmp/rec_$1.out"
echo -n "after:  "; health_ok >/dev/null; health_ok; sync
