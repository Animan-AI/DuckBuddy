# -*- coding: utf-8 -*-
"""lifed: the duck's life layer, running on the robot next to robotd (L1 of
docs/project/duck-life-design-20261013-CN.md).

    python3 lifed.py [--robotd /tmp/robotd.sock] [--tap /tmp/padd/tap.sock] [--sock /tmp/lifed.sock]

It only *asks*. robotd keeps every limit and reflex; this never powers the motors, starts or stops a
policy, relaxes, or walks. When no policy is driving it sits quiet and watches.

Inputs: robotd's state stream (10 Hz) and health (5 s), padd's raw input tap (is the gamepad there,
is it being used), and events on its own socket (`life.event`) — a stand-in for voiced / visiond /
an agent until they exist. Outputs: head animation (25 Hz, `robot.head`), sounds, rest / wake.
The decisions are core.py's; this file is plumbing.
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import json
import math
import os
import random
import socket
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from core import Action, Body, Config, Life  # noqa: E402
import judgments  # noqa: E402
import mission  # noqa: E402
import prompts  # noqa: E402
import smallbrain  # noqa: E402
# who the duck is (its name, its character) comes from its persona: tools/persona, ~/persona on the robot
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "persona"))
from persona import load as load_persona  # noqa: E402

HEAD_KEYS = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
LIFE_HZ = 10.0
HEAD_HZ = 25.0
PAD_ACTIVE_HOLD_S = 5.0     # the pad counts as in use this long after its last real input
# The online brain answers in 0.6-1.0 s through frp (2026-10-16); past this the small brain answers
# instead, and the network is not tried again for BRAIN_RETRY_S (offline, answers come at once).
BRAIN_TIMEOUT_S = 3.5
BRAIN_RETRY_S = 60.0
# Jev (duckbrain /judge, docs/project/duck-life-jev-framework-20261017-CN.md): asked alongside the
# brain, so it adds nothing to the brain's ~0.7 s; past this it is skipped and the rules decide.
TOF_CENTRE = [r * 8 + c for r in range(2, 6) for c in range(2, 6)]
LOOK_TIMEOUT_S = 8.0          # a photo up through frp and the 27B looking (0.6 s on the box)
JEV_TIMEOUT_S = 1.2           # Jev answers in 0.3-0.5 s from here, 1.0 s on a cold connection (10-17)
CN_DIGITS = "零一二三四五六七八九十"


class Clock:
    """The local time, when there is one. The OPi has no RTC and its system zone is UTC (2026-10-16:
    at 00:05 here lifed called it afternoon), so: the wall clock only once NTP has set it, shifted
    by a fixed zone offset (the system's zone is left alone)."""

    def __init__(self, tz_hours: float):
        self.tz = tz_hours * 3600.0
        self.synced = False
        self.checked = -1e9

    def now(self):
        """(hour, minute), or None while the clock is not set."""
        if not self.synced and time.monotonic() - self.checked > 60.0:
            self.checked = time.monotonic()
            try:
                out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                                     capture_output=True, text=True, timeout=3).stdout.strip()
                self.synced = out == "yes"
            except (OSError, subprocess.SubprocessError):
                self.synced = False
        if not self.synced:
            return None
        t = time.gmtime(time.time() + self.tz)
        return t.tm_hour, t.tm_min

    @staticmethod
    def words(hour: int, minute: int) -> str:
        """晚上十一点二十分 — the time as the brain should hear it."""
        part = smallbrain.part_of_day(hour)
        m = "" if minute < 5 else "半" if 25 <= minute < 35 else f"{Clock.num(minute)}分"
        return f"{part}{smallbrain.hour_words(hour)}点{m}"

    @staticmethod
    def num(n: int) -> str:
        if n < 10:
            return "零" + CN_DIGITS[n]          # minutes: 十二点零五分
        tens, ones = divmod(n, 10)
        return ("" if tens == 1 else CN_DIGITS[tens]) + "十" + (CN_DIGITS[ones] if ones else "")


class Volume:
    """The speaker's loudness: the codec's own digital volume (PCM, 0.5 dB a step), set as a mapped
    percentage (`amixer -M`), for duck-speak and the person's-voice clips alike. What the user set is
    kept in a state file and put back at start (the boot script sets PCM to the top). At night
    (22:00-07:30, only with a synced clock) it is capped, unless the user turned it up that night."""
    MIN, MAX, NIGHT_CAP = 20, 100, 40      # amixer -M: 20 % is -35 dB, 40 % -20.5 dB, 70 % -8.5 dB
    NIGHT = (22 * 60, 7 * 60 + 30)

    def __init__(self, path: str, clock, card: str | None = "aic3104"):
        self.path, self.clock, self.card = path, clock, card
        self.user, self.night_ok_until, self.applied = 100, 0.0, None
        try:
            with open(path, encoding="utf-8") as f:
                st = json.load(f)
            self.user = int(st.get("volume", 100))
            self.night_ok_until = float(st.get("night_ok_until", 0.0))
        except (OSError, ValueError):
            pass

    def minutes_to_morning(self) -> int | None:
        """Minutes until 07:30 if it is night now, else None (also with no clock)."""
        now = self.clock.now()
        if now is None:
            return None
        m = now[0] * 60 + now[1]
        if not (m >= self.NIGHT[0] or m < self.NIGHT[1]):
            return None
        return (self.NIGHT[1] - m) % (24 * 60)

    def effective(self) -> int:
        if self.minutes_to_morning() is not None and time.time() > self.night_ok_until:
            return min(self.user, self.NIGHT_CAP)
        return self.user

    def change(self, set_to=None, step=None) -> int:
        """The user's new loudness (clamped); returns what it plays at now."""
        base = self.effective()
        new = int(set_to) if set_to is not None else base + int(step or 0)
        self.user = max(self.MIN, min(self.MAX, new))
        left = self.minutes_to_morning()
        if left is not None and self.user > self.NIGHT_CAP:
            self.night_ok_until = time.time() + left * 60    # turned up at night: so be it, until morning
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump({"volume": self.user, "night_ok_until": self.night_ok_until}, f)
        except OSError as e:
            log(f"volume: cannot keep it: {e!r}")
        return self.apply()

    def apply(self) -> int:
        """Set the codec to what it should play at now, if that changed (night starting, ending)."""
        eff = self.effective()
        if eff != self.applied and self.card:
            try:
                subprocess.run(["amixer", "-c", self.card, "-M", "-q", "sset", "PCM", f"{eff}%"],
                               timeout=3, check=True)
                log(f"volume {eff}% (set {self.user}%)")
            except (OSError, subprocess.SubprocessError) as e:
                log(f"volume: {e!r}")
                return eff
        self.applied = eff
        return eff

    def state(self) -> dict:
        eff = self.effective()
        return {"user": self.user, "now": eff, "night_cap": eff < self.user}


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


# ── robotd ─────────────────────────────────────────────────────────────────────────────────────

class Robot:
    def __init__(self, path):
        self.path = path
        self.state: dict | None = None
        self.state_at = 0.0
        self.battery_pct: float | None = None
        self._w: asyncio.StreamWriter | None = None

    async def state_loop(self):
        while True:
            try:
                r, w = await asyncio.open_unix_connection(self.path, limit=1 << 22)
                w.write(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":10}}\n')
                await w.drain()
                while line := await r.readline():
                    m = json.loads(line)
                    if m.get("method") == "robot.state":
                        self.state, self.state_at = m["params"], time.monotonic()
            except (OSError, ValueError) as e:
                log(f"robotd state stream: {e}")
            self.state = None
            await asyncio.sleep(2.0)

    async def request(self, method, params=None, timeout=3.0):
        try:
            r, w = await asyncio.wait_for(asyncio.open_unix_connection(self.path), timeout)
            req = {"jsonrpc": "2.0", "id": 1, "method": method}
            if params is not None:
                req["params"] = params
            w.write((json.dumps(req) + "\n").encode())
            await w.drain()
            line = await asyncio.wait_for(r.readline(), timeout)
            w.close()
            return json.loads(line)
        except (OSError, ValueError, asyncio.TimeoutError) as e:
            return {"error": {"message": str(e)}}

    async def notify(self, method, params):
        for _ in range(2):
            try:
                if self._w is None or self._w.is_closing():
                    _, self._w = await asyncio.open_unix_connection(self.path)
                self._w.write((json.dumps({"jsonrpc": "2.0", "method": method, "params": params}) + "\n").encode())
                await self._w.drain()
                return
            except OSError:
                self._w = None

    async def health_loop(self):
        while True:
            h = (await self.request("robot.health")).get("result") or {}
            pct = (h.get("battery") or {}).get("percent")
            if pct is not None:
                self.battery_pct = float(pct)
            await asyncio.sleep(5.0)


# ── the gamepad (padd's raw tap) ─────────────────────────────────────────────────────────────────

class Pad:
    """Whether a pad is connected, and when it last did something deliberate: a button, or an axis
    pushed past a quarter of its range from where it rested when the pad attached. A pad at rest
    is silent on the tap, so silence reads as idle."""

    def __init__(self, path):
        self.path = path
        self.present = False
        self.last_active = -1e9
        self.axes: dict[int, tuple[float, float]] = {}   # code -> (rest, quarter range)

    @property
    def active(self) -> bool:
        return self.present and time.monotonic() - self.last_active < PAD_ACTIVE_HOLD_S

    async def loop(self):
        while True:
            try:
                r, w = await asyncio.open_unix_connection(self.path, limit=1 << 22)
                w.write(b'{"jsonrpc":"2.0","id":1,"method":"pad.input"}\n')
                await w.drain()
                while line := await r.readline():
                    m = json.loads(line)
                    if m.get("method") == "pad.report":
                        self._report(m["params"])
            except (OSError, ValueError) as e:
                if self.present:
                    log(f"pad tap: {e}")
            self.present = False
            await asyncio.sleep(3.0)

    def _report(self, p):
        kind = p.get("report")
        if kind == "attached":
            self.present = True
            self.axes = {a["code"]: (a["value"], 0.25 * (a["max"] - a["min"])) for a in p["device"].get("axes", [])}
            log("pad attached")
        elif kind == "detached":
            if self.present:
                log(f"pad detached: {p.get('why')}")
            self.present = False
        elif kind == "frame":
            self.present = True
            for e in p.get("events", []):
                if e["kind"] == 1 and e["value"]:
                    self.last_active = time.monotonic()
                elif e["kind"] == 3:
                    rest, quarter = self.axes.get(e["code"], (0, 1))
                    hat = e["name"].startswith("ABS_HAT")
                    if (hat and e["value"]) or (not hat and abs(e["value"] - rest) > quarter):
                        self.last_active = time.monotonic()


# ── the head ───────────────────────────────────────────────────────────────────────────────────

def _ease(x):
    x = max(0.0, min(1.0, x))
    return 0.5 - 0.5 * math.cos(math.pi * x)


# The brain's gestures (tools/duckbrain), and the body moves it asks for said with the neck — the
# walking policy was never trained to track a body pose. (t s, neck, head pitch, yaw, roll) rad.
KEYFRAMES = {
    # surprise: the head snaps up, holds, settles
    "look_up": [(0, 0, 0, 0, 0), (0.15, 0.05, -0.38, 0, 0), (0.7, 0.05, -0.38, 0, 0), (1.3, 0, 0, 0, 0)],
    # sad or sleepy: the head sinks and stays down a while
    "droop": [(0, 0, 0, 0, 0), (0.9, 0, 0.38, 0, 0), (2.4, 0, 0.38, 0, 0), (3.4, 0, 0, 0, 0)],
    # coy: a slow tilt one way, the other, back
    "nuzzle": [(0, 0, 0, 0, 0), (0.6, 0, 0.05, 0.12, 0.22), (1.3, 0, 0.05, -0.08, -0.18),
               (2.0, 0, 0.05, 0.08, 0.15), (2.7, 0, 0, 0, 0)],
    # happy: three quick bobs
    "bob": [(0, 0, 0, 0, 0), (0.12, 0, 0.16, 0, 0), (0.24, 0, 0, 0, 0), (0.36, 0, 0.16, 0, 0),
            (0.48, 0, 0, 0, 0), (0.6, 0, 0.16, 0, 0), (0.75, 0, 0, 0, 0)],
    # curious: the neck reaches forward, the head lifts to keep looking
    "lean_in": [(0, 0, 0, 0, 0), (0.5, -0.25, -0.18, 0, 0), (2.0, -0.25, -0.18, 0, 0), (2.6, 0, 0, 0, 0)],
    # shy or startled: the neck pulls back, the head tucks
    "shrink": [(0, 0, 0, 0, 0), (0.3, 0.15, 0.12, 0, 0), (1.5, 0.15, 0.12, 0, 0), (2.1, 0, 0, 0, 0)],
}


class Head:
    """One owner of the head command, mixing a gaze (eased toward a target, held, eased home), one
    gesture at a time, a slow breath and a tired droop. Conventions (kinematics/src/head.rs):
    +head_yaw looks left, +head_pitch looks down; zero is straight ahead."""

    def __init__(self):
        self.yaw = self.pitch_up = 0.0          # current gaze
        self.tyaw = self.tpitch = 0.0           # target gaze
        self.hold_until = 0.0
        self.gaze_tau = 0.35
        self.gesture = None                     # (fn(t) -> 4 offsets, t0, duration)
        self.t = 0.0

    def gaze(self, yaw, pitch, hold_s, speed=1.0):
        self.tyaw, self.tpitch = max(-1.0, min(1.0, yaw)), max(-0.4, min(0.5, pitch))
        self.hold_until = self.t + hold_s
        self.gaze_tau = 0.35 / max(0.3, speed)

    def stop(self):
        self.tyaw = self.tpitch = 0.0
        self.gesture = None

    def play(self, name, amp=1.0, speed=1.0, **kw):
        if name == "nod":
            n, T, A = int(kw.get("times", 1)), 0.8 / speed, 0.35 * amp
            fn = lambda t: (0.0, A * 0.5 * (1 - math.cos(2 * math.pi * t / T)), 0.0, 0.0)
            dur = n * T
        elif name == "shake":
            n, T, A = int(kw.get("times", 2)), 0.9 / speed, 0.5 * amp
            fn = lambda t: (0.0, 0.0, A * math.sin(2 * math.pi * t / T) * min(1, t / (0.5 * T), (n * T - t) / (0.5 * T)), 0.0)
            dur = n * T
        elif name == "tilt":
            side, R, ramp, hold = kw.get("side", 1), 0.28 * amp, 0.45 / speed, 1.2
            dur = 2 * ramp + hold
            fn = lambda t: (0.0, -0.05 * amp * _ease(min(t, dur - t) / ramp), 0.0,
                            side * R * _ease(min(t, dur - t) / ramp))
        elif name in KEYFRAMES:
            # (t, neck, head pitch, yaw, roll), eased between; neck < 0 reaches forward (the LIE
            # pose's neck is -0.94 with the jaw on the floor), head pitch > 0 looks down
            keys = [(t / speed, *(v * amp for v in rest)) for t, *rest in KEYFRAMES[name]]

            def fn(t, keys=keys):
                for k0, k1 in zip(keys, keys[1:]):
                    if t <= k1[0]:
                        k = _ease((t - k0[0]) / (k1[0] - k0[0]))
                        return tuple(a + k * (b - a) for a, b in zip(k0[1:], k1[1:]))
                return (0.0, 0.0, 0.0, 0.0)
            dur = keys[-1][0]
        elif name == "wake_up":
            # look up, then left, right, back to the middle
            keys = [(0.0, 0, 0), (1.0, 0, -0.35), (1.8, 0.7, -0.15), (2.8, -0.7, -0.15), (3.6, 0, 0)]

            def fn(t):
                for (t0, y0, p0), (t1, y1, p1) in zip(keys, keys[1:]):
                    if t <= t1:
                        k = _ease((t - t0) / (t1 - t0))
                        return (0.0, p0 + k * (p1 - p0), y0 + k * (y1 - y0), 0.0)
                return (0.0, 0.0, 0.0, 0.0)
            dur = keys[-1][0]
        else:
            return
        self.gesture = (fn, self.t, dur)

    def tick(self, dt, energy, arousal):
        self.t += dt
        if self.t > self.hold_until:
            self.tyaw, self.tpitch = 0.0, 0.0
        k = 1.0 - math.exp(-dt / self.gaze_tau)
        self.yaw += k * (self.tyaw - self.yaw)
        self.pitch_up += k * (self.tpitch - self.pitch_up)
        e = 1.0 if energy is None else energy
        droop = 0.25 * (1.0 - e)                                   # tired: the head hangs a little
        # No breath: a head that never stops moving is a head whose servos never stop working
        # (2026-10-14: head_yaw 66 C in ten minutes of idle life). Still, between looks.
        neck, hp, yaw, roll = 0.0, -self.pitch_up + droop, self.yaw, 0.0
        if self.gesture:
            fn, t0, dur = self.gesture
            if self.t - t0 >= dur:
                self.gesture = None
            else:
                o = fn(self.t - t0)
                neck, hp, yaw, roll = neck + o[0], hp + o[1], yaw + o[2], roll + o[3]
        return (neck, hp, yaw, roll)


# ── the daemon ─────────────────────────────────────────────────────────────────────────────────

class Lifed:
    def __init__(self, a):
        self.robot = Robot(a.robotd)
        self.pad = Pad(a.tap)
        self.missions = getattr(a, "missions", "dry")
        self.persona = load_persona()
        log(f"persona: {self.persona.name} ({self.persona.source})")
        self.life = Life(Config(name=self.persona.name, lively=self.persona.lively, talkative=self.persona.talkative,
                                quiet=a.quiet, voice_s=clip_lengths(os.path.expanduser(a.voice_dir)),
                                missions=self.missions), random.Random())
        self.mission: mission.Mission | None = None
        self.mission_sim: mission.SimRobot | None = None
        self.mission_moving = False    # the last twist it sent was not zero
        self.tof_sock = getattr(a, "tof", "/tmp/tofd.sock")
        self.tof_m, self.tof_at = None, -1e9   # the head ToF's middle distance (None: nothing in range)
        self.head = Head()
        self.events: list[dict] = []
        self.sock = a.sock
        self.trace = a.trace
        self.body = Body()
        self.sent_head = None          # last head command this sent
        # what it sent over the last second: robot.state reports the head command 100-200 ms late,
        # so a head on the move never matches the latest send
        self.sent_recent = collections.deque()
        self.head_on = False           # streaming the head now
        self.yield_until = 0.0         # someone else drives the head: stand back until then
        self.mismatch_since = None
        self.brain = a.brain.rstrip("/")
        self.clock = Clock(a.tz_hours)
        self.vision_sock = a.vision
        self.watch_warned = -1e9
        self.voice_dir = os.path.expanduser(a.voice_dir)
        self.audio_device = a.audio_device
        self.voice_proc, self.voice_seq = None, 0     # the person's-voice clip playing
        self.volume = Volume(os.path.expanduser(getattr(a, "state", "~/lifed/state.json")), self.clock,
                             getattr(a, "card", "aic3104") or None)
        self.volume_checked = 0.0
        self.brain_down_until = 0.0
        self.last_say = ""
        self.last_meaning = ""
        self.jev_mode = a.jev
        self.jev_stats = {"ok": 0, "fallback": 0, "last_error": None, "recent": []}
        self.rng = random.Random()

    def build_body(self) -> Body:
        st = self.robot.state
        fresh = st is not None and time.monotonic() - self.robot.state_at < 2.0
        if not fresh:
            return Body(battery_pct=self.robot.battery_pct, pad_present=self.pad.present, pad_active=self.pad.active)
        mv = st.get("move") or st.get("movement") or {}   # robot.state calls it "move" (lifed read "movement" until 10-17)
        twist = mv.get("applied") or mv.get("requested") or [0.0, 0.0, 0.0]
        safety = st.get("safety") or {}
        od = st.get("odom") or {}
        pos = od.get("position")
        odom = (pos[0], pos[1], od.get("yaw") or 0.0) if pos else None
        return Body(t=st.get("t", 0.0), policy=st.get("policy") or "", fallen=bool(safety.get("fallen")), odom=odom,
                    gravity=tuple(safety.get("gravity") or (0, 0, -1)), temps_c=st.get("temps_c") or [],
                    twist=tuple(twist), head_cmd=tuple(st.get("head") or (0, 0, 0, 0)), rest=st.get("rest"),
                    battery_pct=self.robot.battery_pct, pad_present=self.pad.present, pad_active=self.pad.active)

    async def life_loop(self):
        period = 1.0 / LIFE_HZ
        last_trace = 0.0
        while True:
            t0 = time.monotonic()
            self.body = self.build_body()
            events, self.events = self.events, []
            for act in self.life.step(period, self.body, events):
                await self.carry_out(act)
            if self.mission is not None and self.mission.state == "running":
                await self.mission_tick(t0, period)
            elif self.mission_moving:
                await self.mission_move((0.0, 0.0, 0.0))     # paused, done, failed, cancelled: stop
            if t0 - self.volume_checked > 30.0:       # at start, and night beginning or ending
                self.volume_checked = t0
                await asyncio.to_thread(self.volume.apply)
            for line in self.life.drain_log():
                log(line)
            if self.trace and t0 - last_trace > 2.0:
                last_trace = t0
                with open(self.trace, "a", encoding="utf-8") as f:
                    f.write(json.dumps({"t": time.time(), **self.snapshot(log_lines=False)}, ensure_ascii=False) + "\n")
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    async def carry_out(self, act: Action):
        k, a = act.kind, act.args
        if k == "gaze":
            self.head.gaze(a["yaw"], a["pitch"], a.get("hold_s", 2.0), speed=self.life.inner.style(self.body.energy())[1])
        elif k == "gaze_stop":
            self.head.stop()
        elif k == "gesture":
            self.head.play(a["name"], a.get("amp", 1.0), a.get("speed", 1.0),
                           **{x: v for x, v in a.items() if x not in ("name", "amp", "speed")})
        elif k == "sound":
            await self.robot.notify("robot.sound", {"tag": a["tag"]})
        elif k == "rest":
            r = await self.robot.request("robot.rest", {"on": True, "deep": bool(a.get("deep"))} if a.get("deep")
                                         else {"on": True})
            log(f"robot.rest {a}: {r.get('result') or r.get('error')}")
        elif k == "wake":
            r = await self.robot.request("robot.rest", {"on": False})
            log(f"robot.rest off: {r.get('result') or r.get('error')}")
        elif k == "skill":
            r = await self.robot.request("robot.do", {"skill": a["skill"]})
            log(f"robot.do {a['skill']}: {r.get('result') or r.get('error')}")
        elif k == "ask":
            asyncio.create_task(self.ask_brain(a["text"], a.get("session", "xiaobai"), a.get("named", True)))
        elif k == "speak":
            asyncio.create_task(self.speak(a["pinyin"], a.get("mood", ""), a.get("delay_s", 0.0)))
        elif k == "voice":
            asyncio.create_task(self.voice(a))
        elif k == "volume":
            await asyncio.to_thread(self.volume.change, a.get("set"), a.get("step"))
        elif k == "mission":
            self.mission_cmd(a)
            if self.mission_moving and (self.mission is None or self.mission.state != "running"):
                await self.mission_move((0.0, 0.0, 0.0))
        elif k == "look":
            asyncio.create_task(self.look(a))
        elif k == "vision":
            asyncio.create_task(self.vision(a))

    def mission_cmd(self, a: dict):
        """The life layer's word on the mission: propose (a new one), start, pause, resume, cancel."""
        cmd, now = a["cmd"], time.monotonic()
        if cmd == "propose":
            real = self.missions == "real"
            self.mission = mission.Mission(a["text"], a["steps"], source="words",
                                           odom_scale=mission.ODOM_SCALE if real else 1.0,
                                           gait=mission.ROBOT_GAIT if real else mission.SIM_GAIT)
            self.mission_sim = None if real else mission.SimRobot()
            log(f"mission ({self.missions}): {self.mission.words()}")
            return
        m = self.mission
        if m is None:
            return
        if cmd == "start":
            m.start(now)
        elif cmd == "pause":
            m.pause(a.get("why", "told"))
        elif cmd == "resume":
            m.resume(now)
        elif cmd == "cancel":
            m.cancel()
        log(f"mission {cmd}: {m.state}")

    async def mission_move(self, twist):
        await self.robot.notify("robot.move", {"vx": twist[0], "vy": twist[1], "vyaw": twist[2]})
        self.mission_moving = any(abs(v) > 1e-6 for v in twist)

    def set_missions_mode(self, mode: str):
        """off / dry / real, from the phone page. A mission under way is called off first."""
        if mode not in ("off", "dry", "real") or mode == self.missions:
            return
        if self.mission is not None and self.mission.state in ("proposed", "running", "paused"):
            self.mission.cancel()
            self.events.append({"type": "mission", "what": "failed", "why": "mode changed", "words": ""})
        self.missions = self.life.cfg.missions = mode
        log(f"missions: {mode}")

    async def mission_tick(self, now: float, dt: float):
        """One step of the running mission: on the stand-in robot (dry), or the robot (real) —
        robotd's odometry scaled by the tape, a twist at 10 Hz (robotd's deadman stops it 0.5 s after
        the last), stopped by the pad, a fall, the ToF going quiet, or something within 0.30 m ahead."""
        m = self.mission
        if self.mission_sim is not None:
            sim = self.mission_sim
            tk = m.tick(now, sim.pose, sim.posture, None)
            sim.step(dt, tk.twist, tk.skill)
        else:
            b = self.body
            stop = None
            if b.posture == "fallen":
                m.state, m.why = "failed", "fell"
                tk = mission.Tick(events=[("failed", "fell")])
            else:
                if b.pad_active:
                    # someone has the sticks: the pad wins (padd sends a twist only while they are
                    # held, so a pad merely connected leaves the walking to the mission)
                    stop = "the pad"
                elif m.moving_step and now - self.tof_at > 1.0:
                    stop = "no ToF"
                elif m.moving_step and b.posture not in ("standing", "walking"):
                    stop = "not standing"
                if stop:
                    m.pause(stop)
                    tk = mission.Tick(events=[("paused", stop)])
                else:
                    pose = mission.Pose(*b.odom) if b.odom else None
                    tk = m.tick(now, pose, b.posture, self.tof_m)
            if m.moving_step or self.mission_moving:
                await self.mission_move(tk.twist if m.state == "running" else (0.0, 0.0, 0.0))
            if tk.skill:
                r = await self.robot.request("robot.do", {"skill": tk.skill})
                log(f"mission robot.do {tk.skill}: {r.get('result') or r.get('error')}")
        for e in tk.events:
            ev = {"type": "mission", "what": e[0]}
            if e[0] == "step_done":
                s = m.steps[e[1]]
                ev.update(i=e[1], words=s.words(), note=s.note)
            elif e[0] == "done":
                ev["words"] = m.words()
            elif e[0] in ("failed", "paused"):
                ev["why"] = e[1]
                if m.pc < len(m.steps):
                    ev["words"] = m.steps[m.pc].words()
            log(f"mission: {ev}")
            self.events.append(ev)

    async def tof_loop(self):
        """The head ToF's middle distance (tofd), for the mission's stop: the median of the valid
        middle 4x4 zones, None when nothing is in range."""
        while True:
            try:
                r, w = await asyncio.open_unix_connection(self.tof_sock, limit=1 << 20)
                w.write(b'{"jsonrpc":"2.0","id":1,"method":"tof.stream"}\n')
                await w.drain()
                while line := await r.readline():
                    m = json.loads(line)
                    if m.get("method") != "tof.frame":
                        continue
                    p = m["params"]
                    d, st = p.get("distance_mm") or [], p.get("status") or []
                    if len(d) >= 64:
                        vals = sorted(d[i] / 1000.0 for i in TOF_CENTRE if st[i] in (5, 9) and d[i] > 0)
                        self.tof_m = vals[len(vals) // 2] if len(vals) >= 4 else None
                        self.tof_at = time.monotonic()
            except (OSError, ValueError) as e:
                log(f"tof: {e!r}; retrying in 5 s")
            await asyncio.sleep(5.0)

    def _vision_call(self, method: str, params: dict, timeout: float):
        s = socket.socket(socket.AF_UNIX)
        s.settimeout(timeout)
        try:
            s.connect(self.vision_sock)
            s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}) + "\n").encode())
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
        finally:
            s.close()
        return json.loads(buf.split(b"\n")[0]).get("result") or {}

    async def look(self, a: dict):
        """"小白你看这是什么": a colour photo from the head camera to the brain (27B, multimodal, on the
        V100 only), which answers about it. Offline, or with no photo, it says so in a person's words."""
        def clip(key, meaning):
            self.events.append({"type": "reply", "emotion": "难过", "say": "", "pinyin": "", "pre": "coo",
                                "gesture": "shake", "body": "none", "action": "none", "voice": key,
                                "meaning": meaning, "via": "vision"})
        if time.monotonic() < self.brain_down_until:
            return clip("look_offline", prompts.PROMPTS["look_offline"])
        try:
            snap = await asyncio.to_thread(self._vision_call, "vision.snap", {"width": 1024}, 10.0)
        except (OSError, ValueError) as e:
            snap = {"ok": False, "reason": repr(e)}
        if not snap.get("ok"):
            log(f"look: no photo ({snap.get('reason')})")
            return clip("look_failed", prompts.PROMPTS["look_failed"])
        body = json.dumps({"text": a["text"], "context": self.context(), "session": a.get("session", "xiaobai"),
                           "image": snap["jpeg_b64"]}).encode()

        def call():
            req = urllib.request.Request(self.brain + "/express", data=body, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=LOOK_TIMEOUT_S) as r:
                return json.loads(r.read())
        t0 = time.monotonic()
        try:
            reply = await asyncio.to_thread(call)
        except (OSError, ValueError) as e:
            self.brain_down_until = time.monotonic() + BRAIN_RETRY_S
            log(f"look: brain unreachable after {time.monotonic() - t0:.2f} s ({e!r})")
            return clip("look_offline", prompts.PROMPTS["look_offline"])
        log(f"look {time.monotonic() - t0:.2f} s ({len(snap['jpeg_b64']) * 3 // 4 // 1024} KB): {a['text']!r} -> "
            f"{json.dumps(reply, ensure_ascii=False)}")
        self.last_say, self.last_meaning = reply.get("say", ""), reply.get("meaning", "")
        self.events.append({"type": "reply", **reply, "via": "eyes", "photo": snap["id"]})

    async def vision(self, a: dict):
        """Ask visiond (the eyes); if it is not there, say so to the life layer as a failed scan."""
        params = {k: v for k, v in a.items() if k != "cmd"}

        def call():
            s = socket.socket(socket.AF_UNIX)
            s.settimeout(3.0)
            s.connect(self.vision_sock)
            s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": f"vision.{a['cmd']}", "params": params})
                       + "\n").encode())
            data = s.recv(4096)
            s.close()
            return json.loads(data.split(b"\n")[0]).get("result") or {}
        try:
            res = await asyncio.to_thread(call)
        except (OSError, ValueError) as e:
            res = {"accepted": False, "reason": repr(e)}
        if a["cmd"] == "watch":
            # renewed every few seconds while awake: only a refusal is news, and not more than once a minute
            if not res.get("accepted") and time.monotonic() - self.watch_warned > 60.0:
                self.watch_warned = time.monotonic()
                log(f"vision.watch refused: {res.get('reason')}")
            return
        log(f"vision.{a['cmd']}: {res}")
        if a["cmd"] == "scan_qr" and not res.get("accepted", True):
            busy = res.get("reason") == "already scanning"
            self.events.append({"type": "qr", "result": "busy" if busy else "failed", "why": res.get("reason", "")})

    async def speak(self, pinyin, mood, delay_s):
        if delay_s > 0:
            await asyncio.sleep(delay_s)
        r = await self.robot.request("robot.speak", {"pinyin": pinyin, "mood": mood})
        res = r.get("result") or {}
        if not res.get("accepted"):
            log(f"robot.speak {pinyin!r}: {res.get('reason') or r.get('error')}")

    async def voice(self, a: dict):
        """A clip in a person's voice (prompts.py) through the shared speaker; duck-speak if it is missing.
        One at a time: a newer clip cuts the one playing (2026-10-17: the QR code read while "请把……二维码"
        was still playing, and "我已经连着……" talked over it)."""
        self.voice_seq += 1
        seq = self.voice_seq
        if a.get("delay_s", 0.0) > 0:
            await asyncio.sleep(a["delay_s"])
        if seq != self.voice_seq:
            return                                   # a newer clip came in while this one waited
        old = self.voice_proc
        if old is not None and old.returncode is None:
            old.terminate()
            log(f"voice: cut short for {a['clip']}")
        await self.robot.request("robot.speak", {"pinyin": "", "mood": ""})      # hush any duck-speak
        await self._play(a["clip"], a.get("pinyin", ""), a.get("mood", ""))
        if a.get("then") and seq == self.voice_seq:
            await self._play(a["then"], "", "")

    def address_wav(self) -> tuple[str | None, str]:
        """The page's address read out, put together from the clips: (wav, what it says)."""
        import wave
        ip = None
        try:
            out = subprocess.run(["ip", "-4", "-o", "addr", "show", "wlan0"], capture_output=True, text=True,
                                 timeout=3).stdout.split()
            if "inet" in out:
                ip = out[out.index("inet") + 1].split("/")[0]
        except (OSError, subprocess.SubprocessError):
            pass
        if ip is None:
            return os.path.join(self.voice_dir, "addr_none.wav"), "addr_none"
        keys = ["addr_intro"] + ["dot" if c == "." else f"d{c}" for c in ip] + ["addr_port"]
        frames, params = [], None
        try:
            for k in keys:
                with wave.open(os.path.join(self.voice_dir, k + ".wav")) as w:
                    params = params or w.getparams()
                    frames.append(w.readframes(w.getnframes()))
                gap = 0.18 if k in ("addr_intro", "dot") else 0.06
                frames.append(b"\0" * (int(gap * params.framerate) * params.sampwidth * params.nchannels))
        except (OSError, EOFError, wave.Error) as e:
            log(f"address: {e!r}")
            return None, ip
        path = "/tmp/lifed-address.wav"
        with wave.open(path, "wb") as w:
            w.setparams(params)
            w.writeframes(b"".join(frames))
        return path, ip

    async def _play(self, clip: str, pinyin: str, mood: str):
        if clip == "address":
            wav, said = self.address_wav()
            log(f"voice: the page's address ({said})")
        else:
            wav = os.path.join(self.voice_dir, clip + ".wav")
        if wav is None or not os.path.exists(wav):
            log(f"voice {clip}: no {wav}; duck-speak instead")
            if pinyin:
                await self.speak(pinyin, mood, 0.0)
            return
        a = {"clip": clip}
        try:
            p = await asyncio.create_subprocess_exec("aplay", "-q", "-D", self.audio_device, wav,
                                                     stdout=asyncio.subprocess.DEVNULL,
                                                     stderr=asyncio.subprocess.PIPE)
            self.voice_proc = p
            _, err = await p.communicate()
            if p.returncode and p.returncode > 0 and b"Interrupted system call" not in err:   # not cut short
                log(f"voice {a['clip']}: aplay exit {p.returncode}: {err.decode(errors='replace').strip()[:200]}")
        except OSError as e:
            log(f"voice {a['clip']}: {e!r}")

    def context(self) -> str:
        """What the duck is like right now, in a few words, for the brain."""
        b, inner = self.body, self.life.inner
        bits = []
        post = {"standing": "站着", "walking": "在走路", "sitting": "坐着", "resting": "坐着休息",
                "lying": "趴着", "rising": "正在起身", "fallen": "刚摔倒了"}.get(b.posture)
        if post:
            bits.append(post)
        if self.life.state == "SLEEPING":
            bits.append("在睡觉，迷迷糊糊的")
        e = b.energy(b.leg_max_c)
        if e is not None and e < 0.35:
            bits.append("腿有点累")
        if b.head_max_c is not None and b.head_max_c >= 62:
            bits.append("头有点热")
        if b.battery_pct is not None and b.battery_pct < 25:
            bits.append("快没电了")
        now = self.clock.now()
        if now is not None:
            bits.append("现在是" + Clock.words(*now))
        bits.append("心情很好" if inner.valence > 0.3 else "有点不开心" if inner.valence < -0.2
                    else "有点无聊" if inner.arousal < 0.2 else "心情平静")
        return "，".join(bits)

    def mind(self) -> smallbrain.Mind:
        """The duck as the small brain needs it."""
        b, inner = self.body, self.life.inner
        now = self.clock.now()
        return smallbrain.Mind(asleep=self.life.state == "SLEEPING", posture=b.posture,
                               energy=b.energy(b.leg_max_c), head_hot=b.head_max_c is not None and b.head_max_c >= 62,
                               battery_pct=b.battery_pct, valence=inner.valence, arousal=inner.arousal,
                               hour=None if now is None else now[0], minute=0 if now is None else now[1])

    async def judge(self, kind: str, state: dict, images: dict | None = None, timeout: float = JEV_TIMEOUT_S):
        """Jev's typed answers through duckbrain's /judge, or None: Jev off, the brain box out of
        reach, no key there, an error or too slow. Every None counts as a fallback to the rules."""
        if self.jev_mode == "off":
            return None
        if time.monotonic() < self.brain_down_until:
            self._jev_fallback("brain down")
            return None
        body = json.dumps({"kind": kind, "state": state, "images": images or {}}, ensure_ascii=False).encode()

        def call():
            req = urllib.request.Request(self.brain + "/judge", data=body, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=timeout + 0.5) as r:
                return json.loads(r.read())
        t0 = time.monotonic()
        try:
            r = await asyncio.wait_for(asyncio.to_thread(call), timeout)
        except (asyncio.TimeoutError, OSError, ValueError) as e:
            self._jev_fallback("timeout" if isinstance(e, asyncio.TimeoutError) else type(e).__name__)
            return None
        if not r.get("ok"):
            self._jev_fallback(r.get("error") or "error")
            return None
        took = time.monotonic() - t0
        self.jev_stats["ok"] += 1
        self.jev_stats["recent"] = (self.jev_stats["recent"] + [{
            "t": round(time.time(), 1), "kind": kind, "s": round(took, 2),
            "answers": {k: v.get("noul", v.get("choice")) for k, v in r["answers"].items()}}])[-5:]
        return r["answers"]

    def _jev_fallback(self, why: str):
        self.jev_stats["fallback"] += 1
        self.jev_stats["last_error"] = why

    async def _call_brain(self, text, session):
        """The brain's expression, or None when it cannot be reached (and then not for a minute)."""
        body = json.dumps({"text": text, "context": self.context(), "session": session}).encode()

        def call():
            req = urllib.request.Request(self.brain + "/express", data=body,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=BRAIN_TIMEOUT_S) as r:
                return json.loads(r.read())
        t0 = time.monotonic()
        try:
            reply = await asyncio.to_thread(call)
            log(f"brain {time.monotonic() - t0:.2f} s: {text!r} -> {json.dumps(reply, ensure_ascii=False)}")
            return reply
        except (OSError, ValueError) as e:
            self.brain_down_until = time.monotonic() + BRAIN_RETRY_S
            log(f"brain unreachable after {time.monotonic() - t0:.2f} s ({e!r}); the small brain answers"
                f" (and for the next {BRAIN_RETRY_S:.0f} s)")
            return None

    def _small(self, text):
        reply = {**smallbrain.respond(text, self.mind(), self.rng, self.last_say, name=self.persona.name,
                                      name_pinyin=self.persona.name_pinyin), "via": "small"}
        log(f"small brain: {text!r} -> {json.dumps(reply, ensure_ascii=False)}")
        return reply

    async def ask_brain(self, text, session="xiaobai", named=True):
        """Something said to the duck: Jev's view of it and the brain's answer, asked together.
        Jev may drop it (not for the duck), turn it into a command the keywords missed, or (mode on)
        let it be a nod or the small brain's quick answer; otherwise the brain answers — or, when it
        cannot be reached, the small brain. One brain session per conversation."""
        brain = None
        if time.monotonic() >= self.brain_down_until:
            brain = asyncio.create_task(self._call_brain(text, session))
        answers = await self.judge("utterance", {
            "text": text, "named": named, "in_conversation": self.life.in_convo(),
            "duck_last_said": self.last_meaning, "duck_is": self.context()})
        recent = self.life.t - self.life.last_spoke < judgments.RECENT_S
        d = judgments.decide_utterance(answers, named, self.jev_mode, recent)
        if d.summary:
            self.events.append({"type": "judged", "text": text, "summary": {**d.summary, "action": d.action}})
            log(f"jev: {text!r} -> {d.action} ({d.why})")
        if d.action in ("ignore", "nod", "command", "small") and brain is not None:
            brain.cancel()        # its answer is not wanted (the request itself runs out on its thread)
        if d.action == "command":
            self.events.append({"type": "command", "name": d.intent, "text": text, "via": "jev"})
        if d.action in ("ignore", "nod", "command"):
            self.events.append({"type": "ignored", "text": text, "why": d.why})
            return
        if d.action == "small":
            reply = self._small(text)
        else:
            reply = (await brain) if brain is not None else None
            if reply is None:
                reply = self._small(text)
        self.last_say = reply.get("say", "")
        self.last_meaning = reply.get("meaning", "")
        self.events.append({"type": "reply", **reply})

    async def head_loop(self):
        period = 1.0 / HEAD_HZ
        while True:
            t0 = time.monotonic()
            b = self.body
            cmd = self.head.tick(period, b.energy(), self.life.inner.arousal)
            allowed = (b.head_free and not b.pad_active and self.life.state not in ("OFF", "SLEEPING", "FALLEN")
                       and t0 >= self.yield_until
                       # walking a mission on the robot: the walk policy's head, not life's
                       and not (self.mission_sim is None and self.mission is not None and self.mission.state == "running"))
            # someone else is driving the head (the pad in head mode, a tool): stand back a while
            while self.sent_recent and t0 - self.sent_recent[0][0] > 1.0:
                self.sent_recent.popleft()
            if self.head_on and self.sent_recent and b.head_cmd:
                off = min(max(abs(x - y) for x, y in zip(b.head_cmd, c)) for _, c in self.sent_recent)
                if off > 0.08:
                    self.mismatch_since = self.mismatch_since or t0
                    if t0 - self.mismatch_since > 0.6:
                        log("head is being driven by someone else: standing back 10 s")
                        self.yield_until, allowed, self.mismatch_since = t0 + 10.0, False, None
                else:
                    self.mismatch_since = None
            if allowed:
                await self.robot.notify("robot.head", dict(zip(HEAD_KEYS, cmd)))
                self.sent_head, self.head_on = cmd, True
                self.sent_recent.append((t0, cmd))
            elif self.head_on:
                # hand back a centred head, unless the pad has it now
                if not b.pad_active and t0 >= self.yield_until:
                    await self.robot.notify("robot.head", dict.fromkeys(HEAD_KEYS, 0.0))
                self.head_on, self.sent_head = False, None
                self.sent_recent.clear()
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def snapshot(self, log_lines=True):
        s = self.life.snapshot(self.body)
        s["head_streaming"] = self.head_on
        # the dialog's times as wall-clock seconds, for the phone page
        offset = time.time() - self.life.t
        s["dialog"] = [{**d, "t": round(d["t"] + offset, 1)} for d in s.get("dialog", [])]
        s["brain_online"] = time.monotonic() >= self.brain_down_until
        s["jev"] = {"mode": self.jev_mode, **self.jev_stats}
        s["volume"] = self.volume.state()
        s["missions_mode"] = self.missions
        s["tof_m"] = None if time.monotonic() - self.tof_at > 1.0 else (
            round(self.tof_m, 2) if self.tof_m is not None else "far")
        if self.mission is not None:
            s["mission"] = {**self.mission.snapshot(), "mode": self.missions}
            if self.mission_sim is not None:
                p = self.mission_sim.pose
                s["mission"]["sim"] = {"x": round(p.x, 2), "y": round(p.y, 2), "yaw_deg": round(math.degrees(p.yaw)),
                                       "posture": self.mission_sim.posture}
        now = self.clock.now()
        s["clock"] = None if now is None else Clock.words(*now)
        if not log_lines:
            s.pop("log", None)
        return s

    async def serve(self):
        if os.path.exists(self.sock):
            os.unlink(self.sock)

        async def client(r, w):
            try:
                while line := await r.readline():
                    m = json.loads(line)
                    method, p, mid = m.get("method"), m.get("params") or {}, m.get("id")
                    if method == "life.state":
                        res = self.snapshot()
                    elif method == "life.event" and p.get("type") == "missions_mode":
                        self.set_missions_mode(str(p.get("mode")))
                        res = {"accepted": True, "mode": self.missions}
                    elif method == "life.event":
                        self.events.append(dict(p))
                        res = {"accepted": True}
                    else:
                        res = None
                    if mid is not None:
                        out = {"jsonrpc": "2.0", "id": mid}
                        out.update({"result": res} if res is not None else
                                   {"error": {"code": -32601, "message": f"unknown method {method}"}})
                        w.write((json.dumps(out, ensure_ascii=False) + "\n").encode())
                        await w.drain()
            except (OSError, ValueError):
                pass
            finally:
                w.close()

        server = await asyncio.start_unix_server(client, self.sock)
        os.chmod(self.sock, 0o660)
        async with server:
            await server.serve_forever()

    async def run(self):
        log(f"lifed up: robotd {self.robot.path}, pad tap {self.pad.path}, events on {self.sock}")
        await asyncio.gather(self.robot.state_loop(), self.robot.health_loop(), self.pad.loop(),
                             self.life_loop(), self.head_loop(), self.serve(), self.tof_loop())


def clip_lengths(voice_dir: str) -> dict:
    """Seconds per recorded person's-voice clip (prompts.py) in voice_dir."""
    import wave
    out = {}
    for key in prompts.PROMPTS:
        try:
            with wave.open(os.path.join(voice_dir, key + ".wav")) as w:
                out[key] = w.getnframes() / w.getframerate()
        except (OSError, EOFError, wave.Error):
            pass
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robotd", default="/tmp/robotd.sock")
    ap.add_argument("--tap", default="/tmp/padd/tap.sock")
    ap.add_argument("--sock", default="/tmp/lifed.sock")
    ap.add_argument("--trace", default=os.path.expanduser("~/robot/recs/lifed-trace.jsonl"))
    ap.add_argument("--quiet", action="store_true", help="no unprompted sounds")
    ap.add_argument("--brain", default="http://127.0.0.1:8210",
                    help="tools/duckbrain, through the frp tunnel (frpc-dev binds it here)")
    ap.add_argument("--vision", default="/tmp/visiond.sock", help="visiond (the eyes)")
    ap.add_argument("--voice-dir", default="~/lifed/voice", help="the person's-voice clips (render_prompts.py)")
    ap.add_argument("--audio-device", default="duck_out", help="ALSA device for them (dmix, shared with robotd)")
    ap.add_argument("--state", default="~/lifed/state.json", help="what it keeps across restarts (the volume)")
    ap.add_argument("--missions", choices=("off", "dry", "real"), default="dry",
                    help="spoken routes: dry runs them on a stand-in robot (nothing moves), real walks the robot "
                         "(the phone page switches it)")
    ap.add_argument("--tof", default="/tmp/tofd.sock", help="tofd's socket, for the mission's obstacle stop")
    ap.add_argument("--card", default="aic3104", help="the sound card whose PCM volume is the speaker's ('' = none)")
    ap.add_argument("--jev", choices=judgments.MODES, default="shadow",
                    help="Jev on what is said: off; shadow (drop garbage, catch missed commands); on (also no reply "
                         "for murmurs, the small brain for greetings)")
    ap.add_argument("--tz-hours", type=float, default=8.0,
                    help="local time zone, hours from UTC (the robot's system zone is UTC)")
    a = ap.parse_args()
    asyncio.run(Lifed(a).run())


if __name__ == "__main__":
    main()
