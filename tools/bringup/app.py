# -*- coding: utf-8 -*-
"""DuckBuddy 装机/维修工具：电脑上起一个网页，四件事——舵机设置、零位标定、IMU 固件烧录、整机测试。

    python tools/bringup/app.py --host orangepi@duck-1a2b.local             # 浏览器开 http://localhost:8780
    python tools/bringup/app.py --host orangepi@192.168.1.50 --com COM11    # 用 IP，外加飞特 USB 调试板

结构：网页 ⇄ 本程序（电脑）⇄ 两个"代理"（agent.py，一行一个 JSON）：
- 机器人代理：每次连接经 ssh 把 agent.py 从 stdin 送到机器人上跑（机器人上不用装东西），
  管 /dev/ttyS7 舵机总线（500k）和麦克风/喇叭/ToF/摄像头/IMU 的测试；
- USB 代理：在电脑上跑 agent.py --usb，接飞特 USB 调试板（CH343）。出厂舵机是 1M，
  OPi 的 UART7 设 1M 实际 750k，新舵机改 ID / 波特率只能走这里。
零位标定直接复用 tools/joint_align（它的 Calib / Renderer / 页面），在 端口+1 上起，网页里嵌进来。
IMU 固件在电脑上 cargo build + probe-rs 下载（ST-Link 接 PDB J2），烧完经机器人总线读 ID 200 核对。
"""
import argparse
import importlib.util
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
AGENT = os.path.join(HERE, "agent.py")
FIRMWARE = os.path.join(ROOT, "firmware", "imu-feetech")
ELF = os.path.join(FIRMWARE, "target", "thumbv6m-none-eabi", "release", "imu-feetech")
CHIP = "STM32G031F8Px"
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

# 机器人端的引导：第一行是 agent.py 的字节数，接着是源码，之后 stdin 就是请求流
BOOT = "import sys;b=sys.stdin.buffer;n=int(b.readline());exec(b.read(n))"


class Agent:
    """一个 agent.py 进程：请求串行发送（一把锁），应答按 id 取回。断了下次调用时重连。"""

    def __init__(self, name, argv, send_source):
        self.name, self.argv, self.send_source = name, argv, send_source
        self.lock = threading.Lock()
        self.p = None
        self.next_id = 0
        self.replies = queue.Queue()
        self.err_tail = []
        self.hello = None

    def _start(self):
        self.err_tail = []
        self.replies = queue.Queue()
        self.p = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, creationflags=NO_WINDOW)
        if self.send_source:
            src = open(AGENT, "rb").read()
            try:
                self.p.stdin.write(b"%d\n" % len(src) + src)
                self.p.stdin.flush()
            except OSError:
                pass  # ssh 已经退出（连不上）：下面等不到 hello，报 ssh 的错误输出
        threading.Thread(target=self._read_out, args=(self.p, self.replies), daemon=True).start()
        threading.Thread(target=self._read_err, args=(self.p,), daemon=True).start()
        try:
            first = self.replies.get(timeout=25)
        except queue.Empty:
            first = None
        if not first or not first.get("hello"):
            time.sleep(0.2)  # 让 stderr 读线程收完 ssh 的报错
            self.close()
            tail = " / ".join(self.err_tail[-3:])
            raise RuntimeError(f"{self.name} 代理没起来" + (f"：{tail}" if tail else ""))
        self.hello = first

    @staticmethod
    def _read_out(p, q):
        for line in p.stdout:
            try:
                q.put(json.loads(line))
            except ValueError:
                pass
        q.put(None)

    def _read_err(self, p):
        for line in p.stderr:
            self.err_tail = (self.err_tail + [line.decode("utf-8", "replace").strip()])[-20:]

    def alive(self):
        return self.p is not None and self.p.poll() is None

    def call(self, op, args=None, timeout=90):
        with self.lock:
            if not self.alive():
                self._start()
            self.next_id += 1
            rid = self.next_id
            try:
                self.p.stdin.write((json.dumps({"id": rid, "op": op, "args": args or {}}) + "\n").encode())
                self.p.stdin.flush()
            except OSError:
                self.close()
                raise RuntimeError(f"{self.name} 代理断开了，再点一次会重连")
            deadline = time.time() + timeout
            while True:
                try:
                    m = self.replies.get(timeout=max(0.1, deadline - time.time()))
                except queue.Empty:
                    self.close()  # 应答对不上了，重来
                    raise RuntimeError(f"{op} 超时（{timeout} s）")
                if m is None:
                    self.close()
                    tail = " / ".join(self.err_tail[-3:])
                    raise RuntimeError(f"{self.name} 代理断开了" + (f"：{tail}" if tail else ""))
                if m.get("id") == rid:
                    if not m.get("ok"):
                        raise RuntimeError(m.get("error") or "失败")
                    return m["result"]

    def close(self):
        if self.p is not None:
            try:
                self.p.kill()
            except OSError:
                pass
        self.p = None


class Job:
    """电脑上跑的命令（cargo / probe-rs），输出逐行收集给网页轮询。"""
    jobs = {}
    seq = 0

    def __init__(self, title, cmds, cwd):
        Job.seq += 1
        self.id, self.title, self.lines, self.rc, self.done = Job.seq, title, [], None, False
        Job.jobs[self.id] = self
        threading.Thread(target=self._run, args=(cmds, cwd), daemon=True).start()

    def _run(self, cmds, cwd):
        for cmd in cmds:
            exe = shutil.which(cmd[0])
            if not exe:
                self.lines.append(f"找不到 {cmd[0]}（PATH 里没有）")
                self.rc = -1
                break
            self.lines.append("$ " + " ".join(cmd))
            try:
                p = subprocess.Popen([exe] + cmd[1:], cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     creationflags=NO_WINDOW)
                for raw in iter(p.stdout.readline, b""):
                    for part in raw.decode("utf-8", "replace").replace("\r", "\n").split("\n"):
                        if part.strip():
                            self.lines.append(part.rstrip())
                self.rc = p.wait()
            except OSError as e:
                self.lines.append(str(e))
                self.rc = -1
            if self.rc != 0:
                break
        self.lines.append("完成" if self.rc == 0 else f"失败（退出码 {self.rc}）")
        self.done = True

    def snapshot(self, since=0):
        return dict(id=self.id, title=self.title, lines=self.lines[since:], n=len(self.lines), done=self.done,
                    rc=self.rc)


def firmware_info():
    def newest(d):
        t = 0
        for base, _, files in os.walk(d):
            for f in files:
                t = max(t, os.path.getmtime(os.path.join(base, f)))
        return t
    src = max(newest(os.path.join(FIRMWARE, "src")), os.path.getmtime(os.path.join(FIRMWARE, "Cargo.toml")))
    elf = os.path.getmtime(ELF) if os.path.exists(ELF) else None
    git = subprocess.run(["git", "log", "-1", "--format=%h %cs %s", "--", "."], cwd=FIRMWARE, capture_output=True,
                         text=True, encoding="utf-8", creationflags=NO_WINDOW).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "."], cwd=FIRMWARE, capture_output=True, text=True,
                           encoding="utf-8", creationflags=NO_WINDOW).stdout.strip()
    fmt = lambda t: time.strftime("%Y-%m-%d %H:%M", time.localtime(t))  # noqa: E731
    return dict(dir=FIRMWARE, elf=ELF if elf else None, elf_time=fmt(elf) if elf else None, src_time=fmt(src),
                stale=elf is None or elf < src, git=git, dirty=[l for l in dirty.splitlines() if "target/" not in l],
                cargo=shutil.which("cargo"), probe_rs=shutil.which("probe-rs"), chip=CHIP)


class Calibration:
    """零位标定：在 端口+1 上起 tools/joint_align 的服务（同一份代码、同一个页面）。"""

    def __init__(self, a):
        self.a = a
        self.url = None  # 起来后是端口号
        self.lock = threading.Lock()

    def start(self):
        with self.lock:
            if self.url:
                return self.url
            spec = importlib.util.spec_from_file_location("joint_align_server",
                                                          os.path.join(ROOT, "tools", "joint_align", "server.py"))
            ja = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ja)
            live = ja.Live()
            # 第一次连上时 joint_align 会发 robot.relax（网页上已确认机器人躺好/托好）
            threading.Thread(target=ja.reader, args=(live, self.a.host, False), daemon=True).start()
            calib = ja.Calib(self.a.host, self.a.config, self.a.restart_cmd, self.a.init_cmd)
            threading.Thread(target=calib.load, daemon=True).start()
            srv = ThreadingHTTPServer((self.a.listen, self.a.port + 1), ja.make_handler(live, ja.Renderer(), False, calib))
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            self.url = self.a.port + 1
            return self.url


def make_handler(a, robot, usb_box, calibration):
    page_path = os.path.join(HERE, "index.html")

    def usb():
        if usb_box["agent"] is None:
            raise RuntimeError("还没选 USB 调试板的串口")
        return usb_box["agent"]

    class H(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, ctype="application/json", code=200):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            qs = parse_qs(u.query)
            if u.path == "/":
                return self._send(open(page_path, "rb").read(), "text/html; charset=utf-8")
            if u.path == "/api/info":
                return self._send(dict(host=a.host, config=a.config, com=usb_box["port"], unit=a.unit,
                                       calib_port=calibration.url, restart_cmd=a.restart_cmd, init_cmd=a.init_cmd))
            if u.path == "/api/usb/ports":
                try:
                    from serial.tools import list_ports
                    ports = [dict(device=p.device, desc=p.description, vid=p.vid, pid=p.pid)
                             for p in list_ports.comports()]
                except ImportError:
                    ports = []
                return self._send(dict(ports=ports, current=usb_box["port"]))
            if u.path == "/api/flash/info":
                return self._send(firmware_info())
            if u.path == "/api/job":
                job = Job.jobs.get(int(qs.get("id", ["0"])[0]))
                if not job:
                    return self._send(dict(error="没有这个任务"), code=404)
                return self._send(job.snapshot(int(qs.get("since", ["0"])[0])))
            self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            try:
                if u.path == "/api/call":
                    target = robot if body.get("target", "robot") == "robot" else usb()
                    args = dict(body.get("args") or {})
                    if target is robot:
                        args.setdefault("unit", a.unit)
                    return self._send(dict(ok=True, result=target.call(body["op"], args,
                                                                       timeout=float(body.get("timeout", 90)))))
                if u.path == "/api/agent/restart":
                    (robot if body.get("target", "robot") == "robot" else usb()).close()
                    return self._send(dict(ok=True))
                if u.path == "/api/usb/open":
                    port = body.get("port")
                    if usb_box["agent"]:
                        usb_box["agent"].close()
                    usb_box["port"] = port or None
                    usb_box["agent"] = usb_agent(port) if port else None
                    return self._send(dict(ok=True, port=usb_box["port"]))
                if u.path == "/api/calib/start":
                    return self._send(dict(ok=True, port=calibration.start()))
                if u.path == "/api/flash/run":
                    what = body.get("what")
                    probe = ["--chip", CHIP, "--non-interactive"]
                    if what == "build":
                        job = Job("构建固件", [["cargo", "build", "--release"]], FIRMWARE)
                    elif what == "probe":
                        job = Job("找 ST-Link", [["probe-rs", "list"]], FIRMWARE)
                    elif what == "flash":
                        if not os.path.exists(ELF):
                            raise RuntimeError("还没构建（target/…/release/imu-feetech 不存在）")
                        job = Job("烧录", [["probe-rs", "download", *probe, "--disable-progressbars", "--verify", ELF],
                                         ["probe-rs", "reset", *probe]], FIRMWARE)
                    else:
                        raise RuntimeError(f"不认识 {what}")
                    return self._send(dict(ok=True, job=job.id))
            except Exception as e:  # noqa: BLE001
                return self._send(dict(ok=False, error=str(e)))
            self._send(dict(ok=False, error="not found"), code=404)

    return H


def usb_agent(port):
    return Agent("USB 调试板", [sys.executable, "-u", AGENT, "--usb", "--port", port, "--baud", "1000000"],
                 send_source=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="机器人的 ssh 地址（免密），如 orangepi@duck-1a2b.local")
    ap.add_argument("--port", type=int, default=8780, help="网页端口；零位标定用 端口+1")
    ap.add_argument("--listen", default="127.0.0.1", help="0.0.0.0 = 局域网里别的设备也能打开")
    ap.add_argument("--com", default=None, help="飞特 USB 调试板串口（如 COM11），也可在网页里选")
    ap.add_argument("--config", default="~/robot/active.toml", help="机器人上 robotd 的配置")
    ap.add_argument("--unit", default="robotd-dev", help="robotd 的 systemd --user 单元")
    ap.add_argument("--restart-cmd", default="systemctl --user restart robotd-dev",
                    help="零位写入后在机器人上执行，让 robotd 重读配置")
    ap.add_argument("--init-cmd", default="cd ~/robot && ./robotctl robot init --robot-socket /tmp/robotd.sock",
                    help="\"写入并回 home\" 重启后执行")
    a = ap.parse_args()

    robot = Agent("机器人", ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5",
                          a.host, f"python3 -u -c '{BOOT}' --config {a.config}"], send_source=True)
    usb_box = {"port": a.com, "agent": None}
    handler = make_handler(a, robot, usb_box, Calibration(a))
    if a.com:
        usb_box["agent"] = usb_agent(a.com)
    srv = ThreadingHTTPServer((a.listen, a.port), handler)
    print(f"http://localhost:{a.port}  （机器人 {a.host}）", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        if usb_box["agent"]:
            usb_box["agent"].close()


if __name__ == "__main__":
    main()
