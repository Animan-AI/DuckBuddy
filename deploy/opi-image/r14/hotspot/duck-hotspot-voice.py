#!/usr/bin/python3
"""Keep r8's offline recognizer; submit QR credentials to the hotspot service."""

import importlib
import json
import socket
import threading
import time

voice = importlib.import_module("duck-wifi-setup")
CHANNEL = "/run/duck-hotspot/setup.sock"
HOTSPOT = {"热点", "热点联网", "热点连网", "热点配网", "手机配网", "用热点配网", "用热点连网"}
SPOKEN_SEPARATORS = str.maketrans("", "", "。！？，、,.!? \t\r\n")


def request(op, **args):
    try:
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(3)
            sock.connect(CHANNEL)
            sock.sendall(json.dumps({"op": op, **args}).encode() + b"\n")
            with sock.makefile("rb") as reply:
                return json.loads(reply.readline())
    except (OSError, ValueError):
        return {}


class Setup(voice.Setup):
    def __init__(self):
        super().__init__()
        self.last_web_result = ""

    def speak_ip(self, ip):
        self.say("addr_intro", *("dot" if char == "." else f"d{char}" for char in ip))

    def address(self):
        ip = voice.wifi_ip()
        if ip and ip != "10.42.0.1":
            self.speak_ip(ip)
        else:
            self.say("addr_none")

    def web_results(self):
        while True:
            state = request("status")
            result = state.get("web_result")
            if result and result != self.last_web_result and self.busy.acquire(blocking=False):
                try:
                    self.last_web_result = result
                    if state["phase"] == "connected":
                        self.say("joined")
                        self.speak_ip(state["ip"])
                    else:
                        self.say("failed")
                except (OSError, voice.subprocess.SubprocessError):
                    print("播放热点配网结果失败", flush=True)
                finally:
                    self.busy.release()
            time.sleep(1)

    def scan(self):
        self.say("scan_start")
        text = self.scan_with_camera()
        if self.cancel.is_set():
            self.say("cancelled")
            return
        if text is None:
            self.say("timeout")
            self.say("hotspot-ready")
            return
        network = voice.parse_wifi_qr(text)
        if network is None:
            self.say("not_wifi")
            return
        if not request("join", **network).get("ok"):
            self.say("failed")
            return
        self.say("joining")
        deadline = time.monotonic() + 65
        while time.monotonic() < deadline:
            state = request("status")
            if state.get("phase") == "connected":
                self.say("joined")
                self.speak_ip(state["ip"])
                return
            if state.get("phase") in ("ap", "failed", "error"):
                break
            time.sleep(1)
        self.say("failed")

    def scan_with_camera(self):
        # If the console has started visiond, use its camera lock for the first-boot scan too.
        try:
            with socket.socket(socket.AF_UNIX) as sock:
                sock.settimeout(38)
                sock.connect("/tmp/visiond.sock")
                sock.sendall(b'{"jsonrpc":"2.0","id":1,"method":"vision.read_setup_qr"}\n')
                with sock.makefile("rb") as reply:
                    response = json.loads(reply.readline(16384))
        except (FileNotFoundError, ConnectionRefusedError):
            response = {}
        result = response.get("result")
        if result is not None:
            if not result.get("ok"):
                raise RuntimeError("摄像头正在使用")
            return result.get("text")
        return voice.scan_qr(str(voice.VISION / "duckcam"), 30, self.cancel)

    def event(self, event):
        if event.get("type") == "capture_ready":
            first = not self.announced
            super().event(event)
            if first:
                threading.Thread(target=self.web_results, daemon=True).start()
            return
        words = str(event.get("text", "")).lower().translate(SPOKEN_SEPARATORS)
        if event.get("type") == "say" and event.get("addressed") and words in HOTSPOT:
            if not self.speaking.is_set() and self.busy.acquire(blocking=False):
                try:
                    ip = voice.wifi_ip()
                    if ip and ip != "10.42.0.1":
                        self.say("already")
                        self.speak_ip(ip)
                    else:
                        self.say("hotspot-ready")
                finally:
                    self.busy.release()
            return
        super().event({**event, "text": words})


if __name__ == "__main__":
    voice.Setup = Setup
    voice.main()
