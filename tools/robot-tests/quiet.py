# quiet.py: 3 s of robot.state, reported per half second — mean target step (deg/tick), the
# policy driving, and gravity z (-1 = upright). Is the robot standing still, and on which network?
import json, math, subprocess
subprocess.run(["python3", "/tmp/rpc.py", "record", "3", "/tmp/q.jsonl"], capture_output=True)
rows = [json.loads(l)["params"] for l in open("/tmp/q.jsonl")]
for k in range(0, len(rows) - 25, 25):
    seg = rows[k:k + 26]
    steps = [abs(math.degrees(a - b)) for r0, r1 in zip(seg, seg[1:]) for a, b in zip(r1["targets"], r0["targets"])]
    print(f"{k/50:.1f}-{(k+25)/50:.1f}s  mean step {sum(steps)/len(steps):.2f} deg/tick  policy {seg[-1].get('policy')}  gz {seg[-1]['safety']['gravity'][2]:.3f}")
