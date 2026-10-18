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
# 先 relax：HL-2915 上电时扭矩可能是开着的，而 robotd 启动时当它是松的、不会去关
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
                                  "python3", "-u", "-"],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, encoding="utf-8", errors="replace")
            p.stdin.write(REMOTE)
            p.stdin.close()
            for line in p.stdout:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                live.set(m.get("joints"), m.get("gravity"), m.get("policy"))
            err = p.stderr.read().strip().splitlines()
            live.status = "断开：" + (err[-1] if err else "robotd 没在运行？")
        except Exception as e:  # noqa: BLE001
            live.status = f"断开：{e}"
        time.sleep(2)


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


def make_handler(live, renderer, demo):
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
            return live.get()

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
                            age=time.time() - stamp if stamp else None, status=status, policy=policy)
                return self._send(json.dumps(body).encode(), "application/json")
            if u.path == "/frame":
                joints, gravity, *_ = self._pose()
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
            self._send(b"{}", "application/json", 404)

    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="orangepi@<robot-ip>")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--start-robotd", action="store_true")
    a = ap.parse_args()

    live = Live()
    if not a.demo:
        threading.Thread(target=reader, args=(live, a.host, a.start_robotd), daemon=True).start()
    renderer = Renderer()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(live, renderer, a.demo))
    print(f"http://localhost:{a.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
