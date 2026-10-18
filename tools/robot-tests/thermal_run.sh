#!/bin/bash
# thermal_run.sh <tag> walk|stand [seconds] [vx m/s] [vyaw rad/s]: how fast the servos heat.
#
# Walks a circle (0.15 m/s at 0.4 rad/s, about 0.75 m across, so it fits a room) or just
# stands, for up to `seconds` (180), recording robot.state to recs/thermal_<tag>.jsonl —
# robotd builds from 2026-10-10 put every servo's temperature in it (`temps_c`, once a second).
# robotd's [thermal] guard ends a walk by itself at walk_max_c (65 C by default): the robot
# stands and the state reports `thermal` in `limited_by`. Analyse with thermal_stats.py.
#
# Start from a standing robot (rpc.py init, set down); hold it when the run ends.
cd ~/robot
MODE=${2:?walk or stand}
SECS=${3:-180}
VX=${4:-0.15}
VYAW=${5:-0.4}
python3 /tmp/rpc.py health
(python3 /tmp/rpc.py record "$(python3 -c "print($SECS + 15)")" "recs/thermal_$1.jsonl" > "/tmp/thermal_$1.out" 2>&1 &)
sleep 0.3
python3 /tmp/rpc.py enable > /dev/null
sleep 5
if [ "$MODE" = walk ]; then
    python3 /tmp/rpc.py move "$VX" "$SECS" "$VYAW"
else
    sleep "$SECS"
fi
sleep 10
cat "/tmp/thermal_$1.out"
python3 /tmp/rpc.py health
sync
