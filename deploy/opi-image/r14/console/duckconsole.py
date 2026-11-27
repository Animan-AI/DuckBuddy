"""New DuckBuddy console and the original conversation/settings page."""

import base64
import html
import json
import os
import socket
import subprocess
from pathlib import Path
from urllib.parse import urlsplit

import duckweb
from robot_control import CommandError, RobotControl

HERE = Path(__file__).resolve().parent
ROBOT = os.environ.get("DUCK_CONSOLE_ROBOT_SOCKET", "/tmp/robotd.sock")
CONFIG = os.environ.get("DUCK_CONSOLE_ROBOT_CONFIG", str(Path.home() / "robot/active.toml"))
ASSETS = {"/console.css": "text/css; charset=utf-8", "/console.js": "text/javascript; charset=utf-8"}


def live_robot(method, params=None):
    state = method == "robot.state"
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.7)
            sock.connect(ROBOT)
            payload = {"jsonrpc": "2.0", "id": 1,
                       "method": "robot.subscribe" if state else method,
                       "params": {"hz": 10} if state else params or {}}
            sock.sendall(json.dumps(payload).encode() + b"\n")
            with sock.makefile("rb") as stream:
                response = json.loads(stream.readline(256 * 1024))
                if state:
                    if response.get("result", {}).get("accepted"):
                        packet = json.loads(stream.readline(256 * 1024))
                        if packet.get("method") == "robot.state":
                            return packet.get("params")
                    return None
            if "result" in response:
                return response["result"]
            if "error" in response:
                return {"accepted": False, "reason": response["error"].get("message", "控制接口拒绝了请求")}
    except (OSError, ValueError):
        return None


CONTROL = RobotControl(live_robot, CONFIG)


class ConsoleHandler(duckweb.Handler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            page = (HERE / "console.html").read_text(encoding="utf-8")
            return self._send(200, page.replace("{{NAME}}", html.escape(duckweb.NAME)).encode(),
                              "text/html; charset=utf-8")
        if path == "/dialog":
            page = duckweb.PAGE.replace("小白", html.escape(duckweb.NAME)).replace(
                "<header>", '<header><a href="/" style="color:inherit">← 驾驶舱</a>', 1)
            return self._send(200, page.encode(), "text/html; charset=utf-8")
        if path in ASSETS:
            return self._send(200, (HERE / path[1:]).read_bytes(), ASSETS[path])
        if path == "/api/console":
            return self._json(CONTROL.snapshot())
        if path == "/api/console/preview":
            return self.camera(False)
        return super().do_GET()

    def camera(self, photo):
        result = duckweb.rpc(self.server.args.vision, "vision.console_frame", {"photo": photo}, timeout=10 if photo else 7)
        if not result or not result.get("ok"):
            reason = (result or {}).get("reason") or "摄像头服务未就绪"
            return self._json(dict(ok=False, reason=reason), 503)
        try:
            image = base64.b64decode(result["jpeg_b64"], validate=True)
        except (KeyError, ValueError):
            return self._json(dict(ok=False, reason="摄像头没有返回有效照片"), 502)
        return self._send(200, image, "image/jpeg")

    def do_POST(self):
        path = urlsplit(self.path).path
        if not path.startswith("/api/console/"):
            return super().do_POST()
        origin = self.headers.get("Origin")
        host = self.headers.get("Host")
        if origin not in (f"http://{host}", f"https://{host}") or self.headers.get_content_type() != "application/json":
            return self._json(dict(ok=False, reason="请求来源不正确"), 403)
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 <= size <= 4096:
                raise ValueError
            data = json.loads(self.rfile.read(size) or b"{}")
            if not isinstance(data, dict):
                raise ValueError
        except ValueError:
            return self._json(dict(ok=False, reason="请求内容无效"), 400)
        if path == "/api/console/camera":
            # Start only the existing camera service; this also works before calibration.
            try:
                result = subprocess.run(["systemctl", "--user", "start", "visiond-dev.service"],
                                        capture_output=True, text=True, timeout=6)
            except (OSError, subprocess.TimeoutExpired):
                return self._json(dict(ok=False, reason="摄像头服务启动失败"), 503)
            if result.returncode:
                return self._json(dict(ok=False, reason="摄像头服务启动失败"), 503)
            return self._json(dict(ok=True))
        if path == "/api/console/photo":
            return self.camera(True)
        if path != "/api/console/action" or not isinstance(data.get("action"), str):
            return self._json(dict(ok=False, reason="未知操作"), 404)
        if "direction" in data and not isinstance(data["direction"], str):
            return self._json(dict(ok=False, reason="方向无效"), 400)
        try:
            return self._json(CONTROL.act(data["action"], data))
        except CommandError as error:
            return self._json(dict(ok=False, reason=str(error)), 409)


if __name__ == "__main__":
    duckweb.Handler = ConsoleHandler
    duckweb.main()
