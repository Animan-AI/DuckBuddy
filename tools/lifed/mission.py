# -*- coding: utf-8 -*-
"""Missions: a few steps in the real world, done one at a time and each checked before the next
(docs/project/duck-agent-tools-plan-20261017-CN.md §5.6).

- `parse_route` turns a spoken route ("往前走一米，左转，再往前走半米，最后坐下") into steps, with no
  network: the offline planner. Anything it cannot read for sure is a question back, not a guess
  ("往左走半米": turn and walk, or sidestep?).
- `Mission` runs them against the odometry: one step at a time, a step only counts once it is
  verified (settled, then checked against its target), a short step is tried once more, anything
  else stops the mission. The program counter and verify bit are AI4Robot's slot-belief-network
  lesson (V11: the verify bit took pick-and-place from 95.5 % to 99 %, and "done" never repeats).
- `SimRobot` stands in for the robot (a dry run, and the tests): the commanded twist integrated,
  with the walking policy's speed error, and a sit that takes a moment.

Pure: no IO, no clocks — tested in test_mission.py.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

@dataclass(frozen=True)
class Gait:
    """What to command, and how slow it may be before a step has taken too long."""
    walk_speed: float        # m/s, slowed near the end ...
    walk_min_speed: float    # ... but not below this
    turn_rate: float         # rad/s
    turn_min_rate: float
    expect_walk: float       # m/s at the slowest it should still get there (the step's time limit)
    expect_turn: float       # rad/s


SIM_GAIT = Gait(0.15, 0.06, 0.5, 0.2, 0.06, 0.2)
# The robot (v9 walk, 2026-10-17): it walks about half the commanded speed (0.27 -> 0.13-0.14 m/s)
# and stands still under ~0.1 m/s commanded (the policy reads that as standing): the pad's speed,
# not slowed much near the end.
ROBOT_GAIT = Gait(0.27, 0.20, 0.6, 0.35, 0.08, 0.15)
WALK_LEAD_M = 0.03       # stop this much short: it coasts
TURN_LEAD_DEG = 4.0
WALK_TOL_M = 0.15        # a walk is done within this of its target
TURN_TOL_DEG = 8.0
SETTLE_S = 0.6           # stopped this long before checking
POSTURE_TIMEOUT_S = 10.0
MAX_STEP_M = 1.5
MAX_TOTAL_M = 3.0
MAX_BACK_M = 0.3         # backwards blind (the ToF looks ahead)
# robotd's contact odometry against a tape, 2026-10-17 (v9 walk policy, 920 g, five ~1 m walks driven
# by the pad, docs/project/odom-cal-20261017-CN.md): forward it reads 0.917 of the true distance
# (sd 0.038), so it is scaled by 1.09, after which a metre is good to +-4 cm. Sideways it reads about
# twice the true drift (the robot does drift left, ~12 cm a metre), and its heading 9-29 deg.
ODOM_SCALE = 1.09
ODOM_SIGMA_PER_M = 0.05  # how unsure the position gets per metre walked
TURN_SIGMA_DEG = 3.0

# ── the offline planner: a spoken route ────────────────────────────────────────────────────────

DIG = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
NUM = r"(?:\d+(?:\.\d+)?|[零一二两三四五六七八九十百点]+|半)"
DIST_RE = re.compile(rf"({NUM})\s*(厘米|公分|cm|米|公尺|m)(半)?", re.I)
ANGLE_RE = re.compile(rf"({NUM})\s*度")
SPLIT_RE = re.compile(r"[，,。；;！!？?、]|然后|接着|之后|最后|再|先|(?<=转)后")
MOVE_WORDS = re.compile(r"走|进|退|转|移|挪|坐|站|起立")


def cn_int(s: str) -> int:
    total, num = 0, 0
    for c in s:
        if c in DIG:
            num = DIG[c]
        elif c == "十":
            total += (num or 1) * 10
            num = 0
        elif c == "百":
            total += (num or 1) * 100
            num = 0
    return total + num


def cn_number(s: str) -> float | None:
    s = s.strip()
    if re.fullmatch(r"\d+(\.\d+)?", s):
        return float(s)
    if s in ("半", "一半"):
        return 0.5
    if "点" in s:
        a, b = s.split("点", 1)
        if not all(c in DIG for c in b):
            return None
        return cn_int(a) + float("0." + "".join(str(DIG[c]) for c in b)) if b else float(cn_int(a))
    if s and all(c in DIG or c in "十百" for c in s):
        return float(cn_int(s))
    return None


def distance_m(c: str) -> float | None:
    m = DIST_RE.search(c)
    if not m:
        return None
    n = cn_number(m.group(1))
    if n is None:
        return None
    unit = m.group(2).lower()
    d = n / 100.0 if unit in ("厘米", "公分", "cm") else n
    return d + (0.5 if m.group(3) else 0.0)


@dataclass
class Step:
    skill: str                    # walk | turn | sit | stand
    args: dict = field(default_factory=dict)
    status: str = "pending"       # pending | running | done | failed
    note: str = ""

    def words(self) -> str:
        a = self.args
        if self.skill == "walk":
            d = a["forward_m"]
            return f"往{'前' if d > 0 else '后'}走 {abs(d):g} 米"
        if self.skill == "turn":
            d = a["deg"]
            return "转身" if abs(d) == 180 else f"{'左' if d > 0 else '右'}转 {abs(d):g}°"
        return {"sit": "坐下", "stand": "站起来"}.get(self.skill, self.skill)


@dataclass
class Route:
    steps: list
    problem: str | None = None    # turn_or_side | how_far | no_sidestep | too_far | too_far_back


def _clause(c: str) -> tuple[list[Step], str | None]:
    """One clause: its steps, or what is unclear about it."""
    d = distance_m(c)
    ang = ANGLE_RE.search(c)
    if re.search(r"侧移|平移|横移|横着|侧着", c):
        return [], "no_sidestep"
    if re.search(r"转身|向后转|掉头|调头", c):
        return [Step("turn", {"deg": 180.0})], None
    m = re.search(r"(左|右)转", c)
    if m:
        deg = cn_number(ang.group(1)) if ang else 90.0
        steps = [Step("turn", {"deg": deg if m.group(1) == "左" else -deg})]
        if d is not None and re.search(r"走|前进", c[m.end():]):
            steps.append(Step("walk", {"forward_m": d}))           # "左转走半米"
        return steps, None
    if re.search(r"后退|倒退|(往|向)后(走|退|挪)", c):
        if d is None:
            return [], "how_far"
        if d > MAX_BACK_M:
            return [], "too_far_back"
        return [Step("walk", {"forward_m": -d})], None
    if re.search(r"(往|向)(左|右)(边|面)?(走|移|挪)", c):
        return [], "turn_or_side"                               # turn and walk, or sidestep?
    if re.search(r"(往|向)?前(走|进|移|挪)|直走", c) or (d is not None and "走" in c):
        if d is None:
            return [], "how_far"
        return [Step("walk", {"forward_m": d})], None
    if re.search(r"坐下|坐好", c):
        return [Step("sit")], None
    if re.search(r"站起来|起立|站好", c):
        return [Step("stand")], None
    return [], None                                            # filler: "好吗", "小白"


def parse_route(text: str) -> Route | None:
    """A spoken route as steps; None if it is no route (no walking or turning in it)."""
    steps, problem = [], None
    for c in SPLIT_RE.split(text):
        if not c or not c.strip() or not MOVE_WORDS.search(c):
            continue
        s, p = _clause(c)
        steps += s
        problem = problem or p
    moving = any(s.skill in ("walk", "turn") for s in steps) or problem in (
        "turn_or_side", "how_far", "no_sidestep", "too_far_back")
    if not moving:
        return None
    if problem is None:
        walks = [abs(s.args["forward_m"]) for s in steps if s.skill == "walk"]
        if any(w > MAX_STEP_M for w in walks) or sum(walks) > MAX_TOTAL_M:
            problem = "too_far"
    return Route([] if problem else steps, problem)


# ── running it ──────────────────────────────────────────────────────────────────────────────────

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def ang_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Tick:
    twist: tuple = (0.0, 0.0, 0.0)     # what to command now (zero when not moving)
    skill: str | None = None           # a one-off (sit_toggle)
    events: list = field(default_factory=list)   # ("step_done", i) ("done",) ("failed", why) ("paused", why)


class Mission:
    """proposed -> running <-> paused -> done | failed | cancelled."""

    def __init__(self, text: str, steps: list[Step], source: str = "words", odom_scale: float = 1.0,
                 gait: Gait = SIM_GAIT):
        self.text, self.steps, self.source = text, steps, source
        self.gait = gait
        self.odom_scale = odom_scale    # true distance per odometry metre (ODOM_SCALE on the robot)
        self.state = "proposed"
        self.pc = 0
        self.why = ""
        self.phase = "go"              # go | settle  (within the running step)
        self.anchor: Pose | None = None
        self.turned = 0.0              # radians turned in this step so far
        self.last_yaw: float | None = None
        self.step_t0 = 0.0
        self.settle_t0 = 0.0
        self.retried = False
        self.skill_sent = False
        self.sigma_xy, self.sigma_yaw_deg = 0.0, 0.0   # how sure it is of where it is (belief)
        self.walked = 0.0

    def words(self) -> str:
        return " → ".join(s.words() for s in self.steps)

    # the person's commands
    def start(self, t: float):
        if self.state == "proposed":
            self.state = "running"
            self._begin_step(t)

    def pause(self, why: str):
        if self.state == "running":
            self.state, self.why, self.phase = "paused", why, "go"

    def resume(self, t: float):
        if self.state == "paused":
            self.state, self.why = "running", ""
            self.step_t0 = t                 # the step's clock restarts; its target stays
            self.skill_sent = False

    def cancel(self):
        if self.state in ("proposed", "running", "paused"):
            self.state = "cancelled"

    @property
    def active(self) -> bool:
        return self.state in ("running", "paused")

    def _begin_step(self, t: float):
        self.phase, self.anchor, self.turned, self.last_yaw = "go", None, 0.0, None
        self.step_t0, self.retried, self.skill_sent = t, False, False
        if self.pc < len(self.steps):
            self.steps[self.pc].status = "running"

    def _fail(self, why: str) -> Tick:
        self.state, self.why = "failed", why
        self.steps[self.pc].status, self.steps[self.pc].note = "failed", why
        return Tick(events=[("failed", why)])

    def _done_step(self, t: float, note: str = "") -> Tick:
        s = self.steps[self.pc]
        s.status, s.note = "done", note
        ev = [("step_done", self.pc)]
        self.pc += 1
        if self.pc >= len(self.steps):
            self.state = "done"
            ev.append(("done",))
        else:
            self._begin_step(t)
        return Tick(events=ev)

    def tick(self, t: float, pose: Pose | None, posture: str, obstacle_m: float | None = None) -> Tick:
        if self.state != "running":
            return Tick()
        s = self.steps[self.pc]
        if s.skill in ("walk", "turn") and pose is None:
            self.pause("no odometry")
            return Tick(events=[("paused", "no odometry")])
        if s.skill == "walk":
            return self._walk(t, s, pose, obstacle_m)
        if s.skill == "turn":
            return self._turn(t, s, pose)
        return self._posture(t, s, posture)

    def _walk(self, t, s, pose, obstacle_m) -> Tick:
        d = s.args["forward_m"]
        if self.anchor is None:
            self.anchor = pose
        a = self.anchor
        progress = self.odom_scale * ((pose.x - a.x) * math.cos(a.yaw) + (pose.y - a.y) * math.sin(a.yaw))
        left = abs(d) - math.copysign(1.0, d) * progress           # > 0: still short
        if self.phase == "go":
            if d > 0 and obstacle_m is not None and obstacle_m < 0.30:
                self.pause("obstacle")
                return Tick(events=[("paused", "obstacle")])
            g = self.gait
            if t - self.step_t0 > abs(d) / g.expect_walk + 5.0:
                return self._fail("timeout")
            if left <= WALK_LEAD_M:
                self.phase, self.settle_t0 = "settle", t
                return Tick()
            v = max(g.walk_min_speed, min(g.walk_speed, left))
            return Tick(twist=(math.copysign(v, d), 0.0, 0.0))
        if t - self.settle_t0 < SETTLE_S:
            return Tick()
        if abs(left) <= WALK_TOL_M:
            self.walked += abs(d)
            self.sigma_xy = ODOM_SIGMA_PER_M * self.walked
            return self._done_step(t, f"{abs(d) - left:.2f} m")
        if left > 0 and not self.retried:                          # short: once more
            self.retried, self.phase = True, "go"
            return Tick()
        return self._fail(f"{'short' if left > 0 else 'overshot'} by {abs(left):.2f} m")

    def _turn(self, t, s, pose) -> Tick:
        target = math.radians(s.args["deg"])
        if self.last_yaw is None:
            self.last_yaw = pose.yaw
        self.turned += ang_diff(pose.yaw, self.last_yaw)
        self.last_yaw = pose.yaw
        left = abs(target) - math.copysign(1.0, target) * self.turned
        if self.phase == "go":
            g = self.gait
            if t - self.step_t0 > abs(target) / g.expect_turn + 5.0:
                return self._fail("timeout")
            if left <= math.radians(TURN_LEAD_DEG):
                self.phase, self.settle_t0 = "settle", t
                return Tick()
            w = max(g.turn_min_rate, min(g.turn_rate, 2.0 * left))
            return Tick(twist=(0.0, 0.0, math.copysign(w, target)))
        if t - self.settle_t0 < SETTLE_S:
            return Tick()
        if abs(math.degrees(left)) <= TURN_TOL_DEG:
            self.sigma_yaw_deg += TURN_SIGMA_DEG
            return self._done_step(t, f"{math.degrees(self.turned):.0f}°")
        if left > 0 and not self.retried:
            self.retried, self.phase = True, "go"
            return Tick()
        return self._fail(f"{'short' if left > 0 else 'over'} by {abs(math.degrees(left)):.0f}°")

    def _posture(self, t, s, posture) -> Tick:
        want = "sitting" if s.skill == "sit" else "standing"
        if posture == want:
            return self._done_step(t)
        if t - self.step_t0 > POSTURE_TIMEOUT_S:
            return self._fail(f"not {want}")
        if not self.skill_sent and posture in ("standing", "sitting"):
            self.skill_sent = True
            return Tick(skill="sit_toggle")                          # sits a standing duck, stands a sitting one
        return Tick()

    @property
    def moving_step(self) -> bool:
        """The running step is one that moves the robot (walk or turn)."""
        return self.state == "running" and self.pc < len(self.steps) and self.steps[self.pc].skill in ("walk", "turn")

    def snapshot(self) -> dict:
        return {"text": self.text, "state": self.state, "why": self.why, "pc": self.pc, "source": self.source,
                "steps": [{"words": s.words(), "status": s.status, "note": s.note} for s in self.steps],
                "sigma_m": round(self.sigma_xy, 2), "sigma_deg": round(self.sigma_yaw_deg, 1)}


# ── a stand-in robot (dry runs, tests) ─────────────────────────────────────────────────────────

class SimRobot:
    """The commanded twist integrated, with the policy's speed error (`scale`, `turn_scale`), the
    twist taking effect `lag` ticks late; sit_toggle takes `sit_s`."""

    def __init__(self, scale: float = 0.9, turn_scale: float = 0.95, posture: str = "standing",
                 sit_s: float = 2.0):
        self.pose = Pose()
        self.scale, self.turn_scale = scale, turn_scale
        self.posture = posture
        self.sit_s = sit_s
        self.toggle_at: float | None = None
        self.t = 0.0

    def step(self, dt: float, twist: tuple, skill: str | None = None):
        self.t += dt
        vx, vy, wz = twist
        self.pose = Pose(self.pose.x + self.scale * (vx * math.cos(self.pose.yaw) - vy * math.sin(self.pose.yaw)) * dt,
                         self.pose.y + self.scale * (vx * math.sin(self.pose.yaw) + vy * math.cos(self.pose.yaw)) * dt,
                         ang_diff(self.pose.yaw + self.turn_scale * wz * dt, 0.0))
        if skill == "sit_toggle" and self.toggle_at is None:
            self.toggle_at = self.t + self.sit_s
        if self.toggle_at is not None and self.t >= self.toggle_at:
            self.posture = "sitting" if self.posture == "standing" else "standing"
            self.toggle_at = None
