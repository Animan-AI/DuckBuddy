# robotd IPC 小工具：python3 rpc.py enable|disable|relax|init|health|watch SECONDS|record SECONDS PATH|move VX SECONDS [VYAW]
import socket, json, sys, time
SOCK = "/tmp/robotd.sock"
def call(method, params=None):
    s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect(SOCK)
    req = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params is not None: req["params"] = params
    s.sendall((json.dumps(req) + "\n").encode()); buf = b""
    while b"\n" not in buf: buf += s.recv(65536)
    return json.loads(buf.split(b"\n")[0])
cmd = sys.argv[1]
if cmd == "enable": print(call("robot.enable", {"on": True, "toggle": False}))
elif cmd == "disable": print(call("robot.enable", {"on": False, "toggle": False}))
elif cmd == "relax": print(call("robot.relax"))
elif cmd == "init": print(call("robot.init"))
elif cmd == "health":
    # one line: battery, CPU and hottest-servo temperature (robot.health)
    r = call("robot.health").get("result", {})
    bat, mot = r.get("battery") or {}, r.get("motors") or {}
    print(f"battery {bat.get('volts', float('nan')):.2f} V ({bat.get('percent', float('nan')):.0f}%)"
          f"  cpu {r.get('cpu_temp_c') or float('nan'):.0f} C"
          f"  servos max {mot.get('max_c', float('nan')):.0f} C ({mot.get('hottest', '?')}) mean {mot.get('mean_c', float('nan')):.0f} C")
elif cmd == "watch":
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5
    s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect(SOCK)
    s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":4}}\n')
    buf = b""; t0 = time.time()
    while time.time() - t0 < secs:
        buf += s.recv(65536)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1); m = json.loads(line)
            if m.get("method") != "robot.state": continue
            p = m["params"]; sf = p.get("safety", {})
            cur = p.get("currents_ma") or p.get("motors", {}).get("currents_ma")
            print(f"t={time.time()-t0:4.1f}s policy={p.get('policy')} enabled={p.get('enabled')} "
                  f"bringup={p.get('bringup')} g={[round(v,2) for v in sf.get('gravity',[])]} fallen={sf.get('fallen')} limp={sf.get('limp')}"
                  + (f" maxI={max(cur):.0f}mA" if cur else ""))
elif cmd == "record":
    # every tick of robot.state for SECONDS, as JSON lines, for offline analysis
    secs = float(sys.argv[2]); path = sys.argv[3]
    s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect(SOCK)
    s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{}}\n')
    buf = b""; t0 = time.time(); n = 0
    with open(path, "w") as f:
        while time.time() - t0 < secs:
            buf += s.recv(65536)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if b'"robot.state"' in line:
                    f.write(line.decode() + "\n"); n += 1
    print(f"recorded {n} ticks to {path}")
elif cmd == "move":
    # robot.move at 20 Hz (deadman is 500 ms), then an explicit zero
    vx = float(sys.argv[2]); secs = float(sys.argv[3]); vyaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect(SOCK)
    def send(vx, vyaw):
        s.sendall((json.dumps({"jsonrpc": "2.0", "method": "robot.move",
                               "params": {"vx": vx, "vy": 0.0, "vyaw": vyaw}}) + "\n").encode())
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            send(vx, vyaw); time.sleep(0.05)
    finally:
        for _ in range(5):
            send(0.0, 0.0); time.sleep(0.05)
    print(f"moved vx={vx} for {secs}s")
