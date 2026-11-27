"""Read robotd's policy catalog and submit bounded operator inputs."""

import re
import threading
import time
import tomllib
from pathlib import Path

POLICIES = {
    "stand": "起身／站立",
    "walk": "行走",
    "sitstand": "坐站",
    "ground_pick": "地面叼取",
    "lie": "趴下",
    "roulade": "前滚翻",
    "kick_left": "左脚踢",
    "kick_right": "右脚踢",
}
REQUIRES = {
    "wake": ("stand", "sitstand", "lie"),
    "stand": ("stand", "sitstand"),
    "sleep": ("sitstand", "lie"),
    "sit": ("sitstand",),
    "pick": ("ground_pick",),
    "walk": ("walk",),
}
COMMANDS = {
    "stand": ("robot.init", {}),
    "sleep": ("robot.rest", {"on": True, "deep": True}),
    "sit": ("robot.do", {"skill": "sit_toggle"}),
    "pick": ("robot.do", {"skill": "ground_pick"}),
    "walk": ("robot.enable", {"on": True}),
}
# Keep the headset controls within the standard image's gamepad speed range.
DIRECTIONS = {
    "forward": (0.18, 0.0, 0.0),
    "back": (-0.15, 0.0, 0.0),
    "left": (0.0, 0.15, 0.0),
    "right": (0.0, -0.15, 0.0),
    "turn-left": (0.0, 0.0, 0.65),
    "turn-right": (0.0, 0.0, -0.65),
}


class CommandError(RuntimeError):
    pass


class RobotControl:
    def __init__(self, call, config):
        self.call = call
        self.config = Path(config)
        self.lock = threading.Lock()
        self.driver = None
        self.sequence = -1
        self.last_status = None
        self.status_at = 0

    def status(self):
        received = self.call("robot.subscribe", {"hz": 2}) or {}
        health = self.call("robot.health") or {}
        try:
            configured = tomllib.loads(self.config.read_text(encoding="utf-8")).get("policy", {})
        except (OSError, ValueError):
            configured = {}
        loaded = bool(received.get("accepted")) and not received.get("unavailable")
        models = []
        active = set()
        for key, name in POLICIES.items():
            file = received.get(key) or configured.get(key)
            if not isinstance(file, str) or file in ("", "none"):
                continue
            if loaded and received.get(key):
                state = "loaded"
                active.add(key)
            else:
                path = Path(file).expanduser()
                if not path.is_absolute():
                    path = self.config.parent / path
                state = "configured" if path.is_file() else "missing"
            models.append(dict(id=key, label=name, file=Path(file).name, state=state))
        battery = health.get("battery") or {}
        reason = battery.get("problem") or health.get("reason") or received.get("unavailable")
        ready = bool(active and health.get("healthy") and not battery.get("problem"))
        return dict(online=bool(received.get("accepted")), ready=ready, reason=reason, models=models,
                    actions={name: ready and bool(active.intersection(keys)) for name, keys in REQUIRES.items()},
                    battery=battery, temperature=(health.get("motors") or {}).get("max_c"),
                    controls=bool(self.driver and ready))

    def snapshot(self, fresh=False):
        with self.lock:
            return self._status_locked(fresh)

    def _status_locked(self, fresh=False):
        if fresh or self.last_status is None or time.monotonic() - self.status_at > 0.8:
            self.last_status = self.status()
            self.status_at = time.monotonic()
        return self.last_status

    def _send(self, method, params):
        reply = self.call(method, params)
        if reply is None:
            raise CommandError("控制服务未响应")
        if not reply.get("accepted"):
            raise CommandError(reply.get("reason") or "控制服务拒绝了这项操作")
        return dict(ok=True)

    def act(self, action, data):
        with self.lock:
            if action in ("move", "stop"):
                return self._motion(action, data)
            if action not in REQUIRES and action != "relax":
                raise CommandError("没有这个动作")
            if action == "relax":
                self.driver = None
                return self._send("robot.relax", {})
            if not self._status_locked(fresh=True)["actions"].get(action):
                raise CommandError("当前没有加载可用的动作模型，请检查动作列表和控制服务")
            client = data.get("client", "")
            if action == "walk" and (not isinstance(client, str) or not re.fullmatch("[a-f0-9]{32}", client)):
                raise CommandError("无效驾驶会话")
            self.driver = None
            self._send("robot.stop", {})
            if action == "wake":
                state = self.call("robot.state")
                if state is None:
                    raise CommandError("还没有收到机身状态，请稍后再唤醒")
                result = self._send("robot.rest", {"on": False}) if state.get("rest") else self._send("robot.init", {})
            else:
                result = self._send(*COMMANDS[action])
            if action == "walk":
                self.driver, self.sequence = client, -1
            self.last_status = None
            return result

    def _motion(self, action, data):
        client, sequence = data.get("client"), data.get("sequence")
        if client == self.driver and self.driver is not None:
            if type(sequence) is not int or sequence <= self.sequence:
                return dict(ok=True, stale=True)
            self.sequence = sequence
        if action == "stop":
            return self._send("robot.stop", {})
        direction = DIRECTIONS.get(data.get("direction"))
        if direction is None or not self.driver or client != self.driver:
            raise CommandError("请先在驾驶舱启用行走")
        if not self._status_locked()["actions"]["walk"]:
            self.driver = None
            self.call("robot.stop", {})
            raise CommandError("行走模型当前不可用")
        return self._send("robot.move", dict(zip(("vx", "vy", "vyaw"), direction)))
