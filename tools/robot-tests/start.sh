#!/bin/bash
# start.sh <config.toml> <tag>: robotd in the background on /tmp/robotd.sock, log to recs/robotd-<tag>.log.
# Refuses if a robotd is already running (stop it by its exact PID first). Torque stays off
# until `python3 /tmp/rpc.py init`.
cd ~/robot
if pgrep -x robotd >/dev/null; then echo "robotd already running: $(pgrep -x robotd)"; exit 1; fi
export ORT_DYLIB_PATH=$HOME/robot/onnxruntime-linux-aarch64-1.28.3/lib/libonnxruntime.so
nohup ./robotd --params "$1" --socket /tmp/robotd.sock > "recs/robotd-$2.log" 2>&1 &
sleep 3; echo "pid $(pgrep -x robotd)"; tail -5 "recs/robotd-$2.log"
