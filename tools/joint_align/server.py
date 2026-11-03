# -*- coding: utf-8 -*-
"""关节映射对照网页：实机关节角 → 训练用的 MuJoCo 模型实时摆出同样姿态。

robotd 的 `robot.state` 每拍给出 15 个关节角（已套 joint_ids / signs / offsets，正是策略
看到的那套角度）和 IMU 重力方向。这个工具把它们套到 microduck_rl 训练用的模型上渲染
（`model/` 是从训练服务器 `robot/microduck_hl2915/` 取来的，运动学与 hardware/original 一致），
扭矩关着时用手摆机器人，对照画面里的模型是否同向、同角度。

    python tools/joint_align/server.py                     # 连 OPi，http://localhost:8770
    python tools/joint_align/server.py --demo              # 不连机器人，只看模型（滑块）

OPi 上要有 robotd 在跑；`--start-robotd` 会替你在 OPi 上启动
（~/robot/robotd --params robotd-test.toml）。连上时先发一次 robot.relax 关扭矩，之后只读；
本工具从不发 enable / move。

标定（2026-10-25，小黑）：启动时从 OPi 读 robotd 的配置（--config）里的 joint_signs / joint_offsets。
网页上每个关节可以"反向"（翻 sign），也可以把当前姿势"设为零位"（重算 offset）；改动马上
用在显示和模型上（robot.state 的角度按 robotd 现用的值还原成编码器读数，再按新值换算）。
"写入配置"先备份再改 toml 里这两行，有 --restart-cmd 就接着重启 robotd 让它生效。

精调：robotd 重启不断扭矩，所以可以站在 home 上微调——每个关节的 −/+ 让它在同一指令角下实际
往负/正方向多转 0.5°，"写入并回 home"写入、重启、再执行 --init-cmd 把机器人平滑带到新的 home：

    python tools/joint_align/server.py --host orangepi@duck-1a2b.local \\
        --restart-cmd "systemctl --user restart robotd-dev" \\
        --init-cmd "cd ~/robot && ./robotctl robot init --robot-socket /tmp/robotd.sock"
"""
import argparse
import io
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
MODEL = os.path.join(HERE, "model", "robot_allcollisions.xml")
MESHDIR = os.path.join(ROOT, "hardware", "original", "mjcf", "assets")

# duck-control/src/model.rs 的 JOINT_NAMES 与 home pose（mouth 不在训练模型里）
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
JOINT_CN = {
    "left_hip_yaw": "左髋 偏航", "left_hip_roll": "左髋 横滚", "left_hip_pitch": "左髋 俯仰",
    "left_knee": "左膝", "left_ankle": "左踝",
    "neck_pitch": "颈 俯仰", "head_pitch": "头 俯仰", "head_yaw": "头 偏航", "head_roll": "头 横滚",
    "mouth": "嘴",
    "right_hip_yaw": "右髋 偏航", "right_hip_roll": "右髋 横滚", "right_hip_pitch": "右髋 俯仰",
    "right_knee": "右膝", "right_ankle": "右踝",
}
HOME = [0.0, -0.0873, -0.4579, -0.0049, 0.4530,
        0.3491, 0.3491, 0.0, 0.0, 0.0,
        0.0, 0.0873, 0.4579, 0.0049, -0.4530]

# OPi 端：订阅 robot.state，每拍打印一行精简 JSON
REMOTE = r'''
import socket, json, sys
# 先 relax：HL-2915 上电时扭矩可能是开着的，而 robotd 启动时当它是松的、不会去关。
# 只在第一次连上时：之后的重连多半是"写入并回 home"重启了 robotd，那时机器人正被带回 home。
if "relax" in sys.argv[1:]:
    r = socket.socket(socket.AF_UNIX); r.connect("/tmp/robotd.sock")
    r.sendall(b'{"jsonrpc":"2.0","id":2,"method":"robot.relax"}\n'); r.recv(4096); r.close()
s = socket.socket(socket.AF_UNIX); s.connect("/tmp/robotd.sock")
s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":20}}\n')
buf = b""
while True:
    d = s.recv(65536)
    if not d: break
    buf += d
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        try: m = json.loads(line)
        except Exception: continue
        if m.get("method") != "robot.state": continue
        p = m["params"]
        print(json.dumps({"t": p.get("t"), "joints": p.get("joints"), "targets": p.get("targets"),
                          "gravity": p.get("safety", {}).get("gravity"), "policy": p.get("policy")}), flush=True)
'''


class Live:
    """最新一帧实机状态，由 SSH 读线程更新。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.joints = list(HOME)
        self.gravity = [0.0, 0.0, -1.0]
        self.stamp = 0.0
        self.status = "未连接"
        self.policy = None

    def set(self, joints, gravity, policy):
        with self.lock:
            if joints and len(joints) == 15:
                self.joints = list(joints)
            if gravity and len(gravity) == 3:
                self.gravity = list(gravity)
            self.policy = policy
            self.stamp = time.time()
            self.status = "已连接"

    def get(self):
        with self.lock:
            return list(self.joints), list(self.gravity), self.stamp, self.status, self.policy


def reader(live, host, start_robotd):
    first = True
    while True:
        if start_robotd:
            # 整个启动包进 nohup sh -c 并关掉所有 fd，否则 ssh 会一直等后台进程的输出
            start = ("cd ~/robot && ORT_DYLIB_PATH=$HOME/robot/onnxruntime-linux-aarch64-1.28.3/lib/libonnxruntime.so "
                     "exec ./robotd --params robotd-test.toml --socket /tmp/robotd.sock > /tmp/robotd.log 2>&1")
            try:
                subprocess.run(["ssh", "-o", "BatchMode=yes", host,
                                "pgrep -x robotd >/dev/null || { nohup sh -c '" + start + "' "
                                "> /dev/null 2>&1 < /dev/null & sleep 5; }"],
                               capture_output=True, timeout=30)
            except subprocess.TimeoutExpired:
                live.status = "启动 robotd 超时"
        live.status = "连接中…"
        try:
            # 脚本走 stdin（python3 -），不经过远端 shell 的引号解析
            p = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=3", host,
                                  "python3", "-u", "-"] + (["relax"] if first else []),
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, encoding="utf-8", errors="replace")
            p.stdin.write(REMOTE)
            p.stdin.close()
            for line in p.stdout:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                first = False
                live.set(m.get("joints"), m.get("gravity"), m.get("policy"))
            err = p.stderr.read().strip().splitlines()
            live.status = "断开：" + (err[-1] if err else "robotd 没在运行？")
        except Exception as e:  # noqa: BLE001
            live.status = f"断开：{e}"
        time.sleep(2)


RAD_PER_COUNT = 2 * math.pi / 4096

# OPi 端：读 robotd 配置里的 signs / offsets（tomllib，Python 3.11+）
READ_CONFIG = r'''
import json, os, sys, tomllib
path = os.path.realpath(os.path.expanduser(sys.argv[1]))
bus = tomllib.load(open(path, "rb")).get("bus", {})
print(json.dumps({"path": path, "signs": bus.get("joint_signs") or [1] * 15,
                  "offsets": bus.get("joint_offsets") or [0] * 15}))
'''

# OPi 端：备份后只改 joint_signs / joint_offsets 两行（各自一行的数组），上面加一行注释
WRITE_CONFIG = r'''
import json, re, shutil, sys, time
path, signs, offsets = sys.argv[1], json.loads(sys.argv[2]), json.loads(sys.argv[3])
text = open(path, encoding="utf-8").read()
fmt = lambda v: "[" + ", ".join("%4d" % x for x in v).lstrip() + "]"
for key, vals in (("joint_signs", signs), ("joint_offsets", offsets)):
    pat = re.compile(r"^(%s\s*=\s*)\[[^\]\n]*\]" % key, re.M)
    if not pat.search(text):
        sys.exit("no single-line %s in %s" % (key, path))
    text = pat.sub(lambda m: m.group(1) + fmt(vals), text, count=1)
stamp = time.strftime("%Y%m%d-%H%M%S")
note = "# %s joint_align: joint_signs / joint_offsets set on the robot by hand\n" % time.strftime("%Y-%m-%d")
text = re.sub(r"^# [0-9-]+ joint_align: joint_signs / joint_offsets set on the robot by hand\n", "", text, flags=re.M)
text = re.sub(r"^(joint_signs\s*=)", lambda m: note + m.group(1), text, count=1, flags=re.M)
shutil.copy2(path, path + ".bak-" + stamp)
open(path, "w", encoding="utf-8").write(text)
print(path + ".bak-" + stamp)
'''


class Calib:
    """robotd 现用的 signs / offsets（从 OPi 读）和网页上正在试的那一套。

    robot.state 的角度按现用值算出：a = s0·(raw − 2048 − o0)·k。先还原 raw − 2048 = s0·a/k + o0，
    再按试用值换算 a' = s1·(raw − 2048 − o1)·k。"""

    def __init__(self, host, config, restart_cmd, init_cmd):
        self.host, self.config, self.restart_cmd, self.init_cmd = host, config, restart_cmd, init_cmd
        self.lock = threading.Lock()
        self.loaded = None          # (signs, offsets) robotd 正在用的
        self.signs = self.offsets = None
        self.path = None
        self.msg = "读取配置中…"

    def load(self):
        try:
            out = subprocess.run(["ssh", "-o", "BatchMode=yes", self.host, "python3", "-", self.config],
                                 input=READ_CONFIG, capture_output=True, text=True, timeout=20)
            cfg = json.loads(out.stdout)
        except Exception as e:  # noqa: BLE001
            self.msg = f"读不到配置：{e}"
            return
        with self.lock:
            self.loaded = ([int(s) for s in cfg["signs"]], [int(o) for o in cfg["offsets"]])
            self.signs, self.offsets = list(self.loaded[0]), list(self.loaded[1])
            self.path = cfg["path"]
            self.msg = f"已读配置 {self.path}"

    def _counts(self, i, a):
        return self.loaded[0][i] * a / RAD_PER_COUNT + self.loaded[1][i]

    def apply(self, joints):
        with self.lock:
            if not self.loaded:
                return joints
            return [self.signs[i] * (self._counts(i, a) - self.offsets[i]) * RAD_PER_COUNT
                    for i, a in enumerate(joints)]

    def flip(self, i):
        with self.lock:
            if self.loaded:
                self.signs[i] = -self.signs[i]

    def zero(self, idx, joints, target=None):
        """把这些关节当前的姿势记成 0°；给了 target（如站姿 HOME）就记成 target[i]。
        a' = s·(raw − 2048 − o)·k = t  →  o = (raw − 2048) − s·t/k，s 用试用中的方向。"""
        with self.lock:
            if self.loaded:
                for i in idx:
                    t = target[i] if target else 0.0
                    self.offsets[i] = int(round(self._counts(i, joints[i]) - self.signs[i] * t / RAD_PER_COUNT))

    def nudge(self, i, deg):
        """同一个指令角下，让这个关节实际往正方向多转 deg 度（读数随之减小 deg）。"""
        with self.lock:
            if self.loaded:
                self.offsets[i] += self.signs[i] * int(round(math.radians(deg) / RAD_PER_COUNT))

    def reset(self):
        with self.lock:
            if self.loaded:
                self.signs, self.offsets = list(self.loaded[0]), list(self.loaded[1])

    def save(self, home=False):
        with self.lock:
            if not self.loaded:
                return False, "配置还没读到"
            signs, offsets = list(self.signs), list(self.offsets)
        try:
            out = subprocess.run(["ssh", "-o", "BatchMode=yes", self.host, "python3", "-", self.path,
                                  "'" + json.dumps(signs) + "'", "'" + json.dumps(offsets) + "'"],
                                 input=WRITE_CONFIG, capture_output=True, text=True, timeout=20)
        except Exception as e:  # noqa: BLE001
            return False, f"写入失败：{e}"
        if out.returncode != 0:
            return False, "写入失败：" + (out.stderr.strip().splitlines() or ["?"])[-1]
        msg = f"已写入 {self.path}（备份 {out.stdout.strip()}）"
        if self.restart_cmd:
            r = subprocess.run(["ssh", "-o", "BatchMode=yes", self.host, self.restart_cmd],
                               capture_output=True, text=True, timeout=30)
            if r.returncode != 0:
                return False, msg + "；重启 robotd 失败：" + r.stderr.strip()
            with self.lock:
                self.loaded = (signs, offsets)
            msg += "；robotd 已重启"
            if home and self.init_cmd:
                # robotd 重启不断扭矩；它要先读到一次总线才接 init，所以失败了隔一会儿再试
                for _ in range(5):
                    time.sleep(1.5)
                    r = subprocess.run(["ssh", "-o", "BatchMode=yes", self.host, self.init_cmd],
                                       capture_output=True, text=True, timeout=30)
                    if r.returncode == 0:
                        return True, msg + "；回 home 中"
                return False, msg + "；回 home 失败：" + (r.stderr.strip() or r.stdout.strip())
        else:
            msg += "；robotd 重启后生效（在那之前显示仍按试用值换算）"
        return True, msg

    def snapshot(self):
        with self.lock:
            if not self.loaded:
                return dict(ready=False, msg=self.msg)
            return dict(ready=True, msg=self.msg, path=self.path, signs=self.signs, offsets=self.offsets,
                        can_home=bool(self.restart_cmd and self.init_cmd),
                        loaded_signs=self.loaded[0], loaded_offsets=self.loaded[1])


class Renderer:
    """单线程渲染（GL 上下文不能跨线程），请求排队。"""

    def __init__(self):
        self.q = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()

    def render(self, joints, gravity, view, bump=None, deg=0.0):
        done = queue.Queue(1)
        self.q.put((joints, gravity, view, bump, deg, done))
        return done.get(timeout=10)

    def _run(self):
        import mujoco
        from PIL import Image
        try:
            spec = mujoco.MjSpec.from_file(MODEL)
            spec.meshdir = MESHDIR
            floor = spec.worldbody.add_geom()
            floor.type = mujoco.mjtGeom.mjGEOM_PLANE
            floor.size = [1, 1, 0.01]
            floor.rgba = [0.55, 0.6, 0.66, 1]
            for pos in ([0.6, 0.6, 1.2], [-0.6, -0.4, 1.0]):
                light = spec.worldbody.add_light()
                light.pos = pos
                light.dir = -np.asarray(pos) / np.linalg.norm(pos)
                light.diffuse = [0.6, 0.6, 0.6]
            spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
            model = spec.compile()
        except Exception as e:  # noqa: BLE001
            print("model load failed:", e, file=sys.stderr, flush=True)
            while True:
                *_, done = self.q.get()
                done.put(b"")
        data = mujoco.MjData(model)
        r = mujoco.Renderer(model, height=420, width=420)
        adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in JOINT_NAMES if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0}
        cam = mujoco.MjvCamera()
        while True:
            joints, gravity, view, bump, deg, done = self.q.get()
            try:
                mujoco.mj_resetData(model, data)
                for i, n in enumerate(JOINT_NAMES):
                    if n in adr:
                        data.qpos[adr[n]] = joints[i] + (math.radians(deg) if n == bump else 0.0)
                # 躯干姿态：把躯干系的重力方向转到世界 -z（偏航取 0）
                g = np.asarray(gravity, float)
                g = g / (np.linalg.norm(g) or 1.0)
                down = np.array([0.0, 0.0, -1.0])
                axis = np.cross(g, down)
                s, c = np.linalg.norm(axis), float(np.dot(g, down))
                quat = np.array([1.0, 0, 0, 0]) if s < 1e-9 else np.r_[
                    math.cos(math.atan2(s, c) / 2), axis / s * math.sin(math.atan2(s, c) / 2)]
                data.qpos[0:3] = [0, 0, 0.3]
                data.qpos[3:7] = quat
                mujoco.mj_kinematics(model, data)
                # 全身最低点贴地（躺着、趴着也对）：几何体中心减去其包围半径
                zmin = float(np.min(data.geom_xpos[1:, 2] - model.geom_rbound[1:] * 0.6))
                data.qpos[2] -= zmin
                mujoco.mj_kinematics(model, data)
                cam.lookat[:] = [0.0, 0.0, 0.11]
                cam.distance = 0.55
                cam.elevation = -8.0
                # 机器人面朝 +x：正面 = 从 +x 看过去（机器人左腿在画面右边），侧面 = 从左侧 +y 看（机器人朝画面左边），iso = 左前方
                cam.azimuth = {"front": 180.0, "side": 270.0, "iso": 215.0}.get(view, 180.0)
                r.update_scene(data, camera=cam)
                img = Image.fromarray(r.render())
                buf = io.BytesIO()
                img.save(buf, "JPEG", quality=82)
                done.put(buf.getvalue())
            except Exception as e:  # noqa: BLE001
                print("render error:", e, file=sys.stderr)
                done.put(b"")


def make_handler(live, renderer, demo, calib):
    page = open(os.path.join(HERE, "index.html"), encoding="utf-8").read()
    slider = {"joints": list(HOME)}

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _pose(self):
            if demo:
                return list(slider["joints"]), [0.0, 0.0, -1.0], time.time(), "演示（无机器人）", None
            joints, *rest = live.get()
            return (calib.apply(joints), *rest)

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            if u.path == "/":
                return self._send(page.encode(), "text/html; charset=utf-8")
            if u.path == "/state":
                joints, gravity, stamp, status, policy = self._pose()
                g = np.asarray(gravity, float)
                pitch = math.degrees(math.atan2(g[0], math.hypot(g[1], g[2])))
                roll = math.degrees(math.atan2(-g[1], -g[2]))
                body = dict(names=JOINT_NAMES, cn=[JOINT_CN[n] for n in JOINT_NAMES], joints=joints,
                            home=HOME, gravity=gravity, pitch=pitch, roll=roll, demo=demo,
                            age=time.time() - stamp if stamp else None, status=status, policy=policy,
                            calib=calib.snapshot() if calib else dict(ready=False, msg="演示模式"))
                return self._send(json.dumps(body).encode(), "application/json")
            if u.path == "/frame":
                joints, gravity, *_ = self._pose()
                ref = qs.get("pose", [None])[0]
                if ref in ("zero", "home"):  # 参考姿势：躯干摆正的模型，照着摆实机
                    joints, gravity = ([0.0] * len(HOME) if ref == "zero" else list(HOME)), [0.0, 0.0, -1.0]
                bump = qs.get("bump", [None])[0]
                deg = float(qs.get("deg", ["0"])[0])
                jpg = renderer.render(joints, gravity, qs.get("view", ["front"])[0], bump, deg)
                return self._send(jpg, "image/jpeg")
            self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
            if u.path == "/demo_pose" and demo:
                j = body.get("joints")
                if isinstance(j, list) and len(j) == 15:
                    slider["joints"] = [float(x) for x in j]
                return self._send(b"{}", "application/json")
            if calib and u.path in ("/flip", "/nudge", "/zero", "/reset", "/save"):
                ok, msg = True, ""
                if u.path == "/flip":
                    calib.flip(int(body["i"]))
                elif u.path == "/nudge":
                    calib.nudge(int(body["i"]), float(body["deg"]))
                elif u.path == "/zero":
                    joints, _, stamp, *_ = live.get()
                    if not stamp or time.time() - stamp > 1.0:
                        ok, msg = False, "实机读数不是最新的，没记"
                    else:
                        idx = [int(body["i"])] if body.get("i") is not None else range(len(JOINT_NAMES))
                        calib.zero(idx, joints, HOME if body.get("ref") == "home" else None)
                elif u.path == "/reset":
                    calib.reset()
                else:
                    ok, msg = calib.save(home=bool(body.get("home")))
                calib.msg = msg or calib.msg
                return self._send(json.dumps(dict(ok=ok, msg=msg)).encode(), "application/json")
            self._send(b"{}", "application/json", 404)

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="orangepi@<robot-ip>")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--start-robotd", action="store_true")
    ap.add_argument("--config", default="~/robot/active.toml", help="OPi 上 robotd 读的配置（标定写回这里）")
    ap.add_argument("--restart-cmd", default="", help="写入配置后在 OPi 上执行，让 robotd 重新读配置")
    ap.add_argument("--init-cmd", default="", help="\"写入并回 home\"在重启后执行，让机器人上扭矩回 home")
    a = ap.parse_args()

    live = Live()
    calib = None
    if not a.demo:
        threading.Thread(target=reader, args=(live, a.host, a.start_robotd), daemon=True).start()
        calib = Calib(a.host, a.config, a.restart_cmd, a.init_cmd)
        threading.Thread(target=calib.load, daemon=True).start()
    renderer = Renderer()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(live, renderer, a.demo, calib))
    print(f"http://localhost:{a.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
