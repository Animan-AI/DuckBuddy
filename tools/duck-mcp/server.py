# -*- coding: utf-8 -*-
"""duck-mcp: the duck's body as an MCP server, for any mind — a voice agent, anima, Claude Code.

Runs on the robot next to robotd and talks to it over its socket (`/tmp/robotd.sock`, the same
JSON-RPC the gamepad and robotctl use). It only *asks*: robotd keeps every limit and safety rule
it already has, and this layer adds the ones a language model needs on top:

- nothing here turns the motors on or off, starts a policy, or walks (yet) — a person does that;
- head gestures only while the stand policy holds the robot upright (the held pose ignores the
  head, and a fallen or limp robot must not be animated);
- refusals say why, in words a model can repeat to a person: too hot, battery low, not standing.

stdio transport, so a client starts it over ssh:

    ssh orangepi@<robot-ip> ~/duck-mcp/.venv/bin/python ~/duck-mcp/server.py
"""
import json
import math
import socket
import time

from mcp.server.fastmcp import FastMCP

SOCK = "/tmp/robotd.sock"
SERVO_HOT_C = 67.0      # robotd stops walking at 70 (rev-a); leave room for a gesture to warm it
BATTERY_LOW_V = 10.8    # 3S: 3.6 V a cell under load
HEAD_ORIGIN = (0.009, 0.0, 0.098)  # head_yaw joint, trunk frame (rev-a model), metres
MAX_YAW_DEG, MAX_PITCH_DEG = 70.0, 35.0
# Heat budget (HL-2915, 2026-10-12): a servo that holds anything warms ~13 C a minute whatever
# the load; one that holds nothing cools ~4 C a minute. "Energy" is the time left before the
# walking guard (70 C) at that rate, over what a fully cool duck has (~2.5 min from 40 C).
WALK_MAX_C = 70.0
HEAT_C_PER_S = 13.0 / 60.0
FULL_ACTIVITY_S = 150.0

mcp = FastMCP("duck")


def _request(method, params=None, timeout=3.0):
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(timeout)
    s.connect(SOCK)
    req = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params is not None:
        req["params"] = params
    s.sendall((json.dumps(req) + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
    s.close()
    return json.loads(buf.split(b"\n")[0])


def _notify(sock, method, params):
    sock.sendall((json.dumps({"jsonrpc": "2.0", "method": method, "params": params}) + "\n").encode())


def _state(timeout=2.0):
    """One robot.state frame."""
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(timeout)
    s.connect(SOCK)
    s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.subscribe","params":{"hz":10}}\n')
    buf = b""
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            buf += s.recv(65536)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                m = json.loads(line)
                if m.get("method") == "robot.state":
                    return m["params"]
    finally:
        s.close()
    raise TimeoutError("robotd sent no state")


def _health():
    return _request("robot.health").get("result", {})


def _snapshot():
    """Everything a gate or a status line needs, in one place."""
    st, h = _state(), _health()
    g = st.get("safety", {}).get("gravity") or [0.0, 0.0, -1.0]
    bat, mot = h.get("battery") or {}, h.get("motors") or {}
    return {
        "policy": st.get("policy"),             # limp / held / stand / walk / sit ...
        "fallen": bool(st.get("safety", {}).get("fallen")),
        "roll_deg": math.degrees(math.atan2(g[1], -g[2])),
        "pitch_deg": math.degrees(math.atan2(-g[0], -g[2])),
        "battery_v": bat.get("volts"),
        "battery_pct": bat.get("percent"),
        "cpu_c": h.get("cpu_temp_c"),
        "servo_max_c": mot.get("max_c"),
        "servo_hottest": mot.get("hottest"),
        "servo_mean_c": mot.get("mean_c"),
        "rest": st.get("rest"),                 # {phase, reason, eta_s} while resting
        # neck, head pitch / yaw / roll (JOINT_NAMES 5..8)
        "head_max_c": max(st["temps_c"][5:9]) if len(st.get("temps_c") or []) >= 9 else None,
    }


def _activity_left_s(s):
    if s["servo_max_c"] is None:
        return None
    return max(0.0, (WALK_MAX_C - s["servo_max_c"]) / HEAT_C_PER_S)


def _gesture_refusal(s):
    """None if the head may move now, else the reason in plain words."""
    if s["fallen"]:
        return "I have fallen over; someone needs to pick me up first."
    rest = s.get("rest")
    if rest and rest.get("phase") != "resting":
        return f"I am {rest.get('phase')} for a rest; give me a moment."
    # Seated with the legs limp ("resting"), the body is on the floor and the head may still
    # move; otherwise only the standing policy holds a head that is safe to swing.
    if not rest and s["policy"] != "stand":
        return (f"I am not standing under my own balance (mode: {s['policy']}). "
                "A person has to stand me up before I can move my head.")
    # Resting, the legs are what is cooling: only the head's own servos matter for a gesture.
    hot_c, which = (s["head_max_c"], "neck") if rest else (s["servo_max_c"], s["servo_hottest"])
    if hot_c is not None and hot_c >= SERVO_HOT_C:
        return f"My {which} servo is at {hot_c:.0f} C; I need to cool down."
    if s["battery_v"] is not None and s["battery_v"] < BATTERY_LOW_V:
        return f"My battery is low ({s['battery_v']:.1f} V); please charge me."
    return None


def _gaze_point(yaw_deg, pitch_deg, dist=1.0):
    y, p = math.radians(yaw_deg), math.radians(pitch_deg)
    ox, oy, oz = HEAD_ORIGIN
    return {"x": ox + dist * math.cos(p) * math.cos(y), "y": oy + dist * math.cos(p) * math.sin(y),
            "z": oz + dist * math.sin(p)}


@mcp.tool()
def status() -> dict:
    """The duck's body right now: whether it is standing on its own, fallen, its tilt, battery,
    CPU and servo temperatures, whether it can move its head (`can_gesture`, with the reason if
    not), and how much energy it has: `energy` 0-1 and `activity_left_s`, the seconds of standing
    or walking left before it must rest to cool down. `rest` is set while it rests, with `eta_s`.
    Call this before promising a person any movement."""
    s = _snapshot()
    why = _gesture_refusal(s)
    left = _activity_left_s(s)
    s["activity_left_s"] = None if left is None else round(left)
    s["energy"] = None if left is None else round(min(1.0, left / FULL_ACTIVITY_S), 2)
    s.update({k: round(v, 1) for k, v in s.items() if isinstance(v, float)})
    s["can_gesture"] = why is None
    if why:
        s["why_not"] = why
    return s


@mcp.tool()
def look(yaw_deg: float = 0.0, pitch_deg: float = 0.0) -> str:
    """Turn the head to look in a direction. yaw_deg: + left, - right (limit +-70).
    pitch_deg: + up, - down (limit +-35). 0, 0 is straight ahead. The head stays there
    until told otherwise."""
    s = _snapshot()
    why = _gesture_refusal(s)
    if why:
        return f"refused: {why}"
    yaw = max(-MAX_YAW_DEG, min(MAX_YAW_DEG, yaw_deg))
    pitch = max(-MAX_PITCH_DEG, min(MAX_PITCH_DEG, pitch_deg))
    r = _request("robot.look", _gaze_point(yaw, pitch))
    if "error" in r:
        return f"robotd refused: {r['error'].get('message')}"
    res = r.get("result", {})
    note = " (as far as my neck reaches)" if res.get("clamped") else ""
    return f"looking yaw {yaw:+.0f} deg, pitch {pitch:+.0f} deg{note}"


HEAD_KEYS = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
HEAD_HZ = 25.0


def _head_motion(pose_at, duration_s):
    """Stream a smooth head trajectory: `pose_at(t)` -> (neck_pitch, head_pitch, head_yaw,
    head_roll) in rad, for t in [0, duration_s], then neutral. Steps jolt the standing policy
    (2026-10-12: a jumpy head on a robot that already shook a little); a sine does not."""
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(2.0)
    s.connect(SOCK)
    t0 = time.time()
    try:
        while (t := time.time() - t0) < duration_s:
            _notify(s, "robot.head", dict(zip(HEAD_KEYS, pose_at(t))))
            time.sleep(1.0 / HEAD_HZ)
    finally:
        _notify(s, "robot.head", dict.fromkeys(HEAD_KEYS, 0.0))
        s.close()


def _swing(n, period_s, amp, bias=0.0):
    """n full cycles of a sine of amplitude `amp`, eased in and out over the first and last
    half cycle so the head starts and ends at rest."""
    total = n * period_s

    def f(t):
        ease = min(1.0, t / (0.5 * period_s), (total - t) / (0.5 * period_s))
        return bias * ease + amp * ease * math.sin(2 * math.pi * t / period_s)
    return f, total


@mcp.tool()
def nod(times: int = 2) -> str:
    """Nod yes (1-4 times). Ends looking straight ahead."""
    s = _snapshot()
    why = _gesture_refusal(s)
    if why:
        return f"refused: {why}"
    n = max(1, min(4, times))
    f, total = _swing(n, 0.8, 0.55, bias=-0.15)
    _head_motion(lambda t: (0.0, f(t), 0.0, 0.0), total)
    return f"nodded {n} times"


@mcp.tool()
def shake_head(times: int = 2) -> str:
    """Shake the head no (1-4 times). Ends looking straight ahead."""
    s = _snapshot()
    why = _gesture_refusal(s)
    if why:
        return f"refused: {why}"
    n = max(1, min(4, times))
    f, total = _swing(n, 0.9, 0.95)
    _head_motion(lambda t: (0.0, 0.0, f(t), 0.0), total)
    return f"shook head {n} times"


@mcp.tool()
def tilt_head(direction: str = "left") -> str:
    """Tilt the head to one side for a moment, as if curious ("left" or "right")."""
    s = _snapshot()
    why = _gesture_refusal(s)
    if why:
        return f"refused: {why}"
    roll = 0.8 if direction == "left" else -0.8
    # ease over 0.4 s, hold, ease back: half a cosine each way
    ramp, hold = 0.4, 1.0

    def r(t):
        if t < ramp:
            return roll * 0.5 * (1 - math.cos(math.pi * t / ramp))
        if t < ramp + hold:
            return roll
        return roll * 0.5 * (1 + math.cos(math.pi * (t - ramp - hold) / ramp))
    _head_motion(lambda t: (0.0, 0.0, 0.0, r(t)), 2 * ramp + hold)
    return f"tilted head {direction}"


@mcp.tool()
def play_sound(tag: str = "chirp") -> str:
    """Make a duck sound: chirp, greet, inquire, coo, alarm, peck."""
    if tag not in ("chirp", "greet", "inquire", "coo", "alarm", "peck"):
        return "refused: unknown sound; use chirp, greet, inquire, coo, alarm or peck"
    r = _request("robot.sound", {"tag": tag})
    if "error" in r:
        return f"robotd refused: {r['error'].get('message')}"
    res = r.get("result", {})
    if isinstance(res, dict) and res.get("accepted") is False:
        return f"refused: {res.get('reason', 'no reason given')}"
    return f"played {tag}"


@mcp.tool()
def rest(deep: bool = False) -> str:
    """Sit down and let the legs go limp to cool off. The duck stands up again by itself once
    cool (about 30 s to a few minutes); the head can still move while it rests. Use it when
    energy is low, or before a long chat. `deep=True` lies down on the chin instead and lets
    the head go limp too (for a hot head, or sleep) — no looking or gestures until it wakes."""
    r = _request("robot.rest", {"on": True, "deep": True} if deep else {"on": True})
    if "error" in r:
        return f"robotd refused: {r['error'].get('message')}"
    res = r.get("result", {})
    if res.get("accepted") is False:
        return f"refused: {res.get('reason')}"
    s = _snapshot()
    if s["servo_max_c"] is not None:
        eta = max(30.0, (s["servo_max_c"] - 45.0) / (4.0 / 60.0))
        how = "lying down" if deep else "sitting down"
        return f"resting: {how} to cool off, about {eta / 60:.0f} min"
    return "resting: lying down to cool off" if deep else "resting: sitting down to cool off"


@mcp.tool()
def wake() -> str:
    """Stand up from a rest. A rest the duck took because it was too hot ends only once it has
    cooled; then this is a no-op and it gets up by itself."""
    s = _snapshot()
    rest_state = s.get("rest")
    if not rest_state:
        return "not resting"
    r = _request("robot.rest", {"on": False})
    if "error" in r:
        return f"robotd refused: {r['error'].get('message')}"
    if rest_state.get("reason") == "hot":
        eta = rest_state.get("eta_s")
        return "still cooling down" + (f", about {eta:.0f} s left" if eta else "") + "; I will get up by myself"
    return "getting up"


if __name__ == "__main__":
    mcp.run()
