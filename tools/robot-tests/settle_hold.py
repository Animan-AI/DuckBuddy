# Wait until the robot has stood upright under the stand policy for 1 s (at most 8 s), then
# disable (hold the pose). Disabling mid-get-up leaves the robot on the floor.
import json, socket, subprocess, time
s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect("/tmp/robotd.sock")
s.sendall(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"robot.subscribe\",\"params\":{\"hz\":10}}\n")
buf, t0, ok_since, state = b"", time.time(), None, ""
while time.time() - t0 < 8:
    buf += s.recv(65536)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1); m = json.loads(line)
        if m.get("method") != "robot.state": continue
        p = m["params"]; g = p["safety"]["gravity"]
        up = p.get("policy") == "stand" and g[2] < -0.94 and not p["safety"].get("fallen")
        state = "%s g_z %.2f" % (p.get("policy"), g[2])
        ok_since = (ok_since or time.time()) if up else None
    if ok_since and time.time() - ok_since > 1.0: break
print("settled" if ok_since else "NOT settled (%s)" % state, "after %.1f s" % (time.time() - t0))
print(subprocess.run(["python3", "/tmp/rpc.py", "disable"], capture_output=True, text=True).stdout.strip()[-60:])
