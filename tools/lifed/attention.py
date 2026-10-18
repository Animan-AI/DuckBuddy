# -*- coding: utf-8 -*-
"""Where Xiaobai looks (docs/project/attention-gaze-design-20261018-CN.md §2-§5).

Everything it could look at is a Target: a slot with a direction, an uncertainty and a salience - the
same belief-slot idea as the mission layer (agent plan §5.6). Observations come from the ears (a
voice's direction, front-or-behind unsettled; a sound), the eyes (a face; something moving), the head
ToF (something near) and memory (where people were). Targets live in the world frame - the body's heading from the
odometry - so a turned head or body does not move them; the gaze it asks for is in the body frame.

Every tick `select` scores them - priority x salience x what the life state cares for, plus a look at
whatever matters and is unsure, minus the turn and a short "seen that" - and keeps the focus a moment
before moving on, unless something more urgent comes (its name always does). In a conversation the
eyes leave the face now and then and come back; with nothing to look at they wander where they have
not looked lately.

Pure: no IO, no clocks (t is passed in), an rng passed in - tested in test_attention.py.
Angles: radians, yaw positive to the left, elevation positive up; 0 straight ahead.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

HEAD_YAW_LIMIT = 1.0          # rad: the head turns no further (lifed's Head)
TURN_BEYOND = math.radians(50)  # a focus held this far round, for TURN_AFTER_S: ask to turn the body
TURN_AFTER_S = 2.0
DWELL_S = 1.0                 # a focus is kept at least this long ...
INTERRUPT_MARGIN = 0.2        # ... unless another's priority is this much higher
IOR_S = 6.0                   # inhibition of return: a target looked at with nothing new since
IOR_COST = 0.3
MAX_TARGETS = 8
MERGE_MIN = math.radians(10)

# kind: (priority, salience half-life s, sigma growth rad/s, max sigma, forget after s)
KINDS = {
    "call":   (1.0, 6.0, math.radians(1), math.radians(40), 20.0),   # its name was said, from there
    "person": (0.65, 20.0, math.radians(3), math.radians(30), 60.0),
    "near":   (0.8, 4.0, math.radians(2), math.radians(30), 10.0),   # the ToF: something close
    "sound":  (0.5, 3.0, math.radians(1), math.radians(40), 8.0),    # a clap, a bang
    "motion": (0.55, 2.0, math.radians(2), math.radians(25), 6.0),   # something moving in view
    "voice":  (0.55, 5.0, math.radians(1), math.radians(40), 15.0),  # speech not to it
    "object": (0.6, 10.0, math.radians(1), math.radians(20), 30.0),  # something shown to it
    "place":  (0.2, 300.0, 0.0, math.radians(30), 300.0),            # where a person was
}
# kinds that are one thing seen and heard: a face where a voice was is the person who spoke
MERGES = {("person", "voice"), ("person", "call"), ("person", "near"), ("voice", "call"), ("near", "call"),
          ("voice", "near"), ("person", "place"), ("person", "motion"), ("motion", "sound"),
          ("motion", "near"), ("object", "motion"), ("object", "near")}
# how long a focus with nothing new holds the eyes, by kind (a person talking or facing it: no limit)
BUDGET_S = {"sound": 1.5, "motion": 1.5, "voice": 2.5, "near": 2.5, "call": 3.0, "object": 4.0, "person": 5.0, "place": 3.0}

# how much each life state cares (lifed's states; walking: the walk policy has the head)
STATE_GAIN = {
    "OFF": 0.0, "SLEEPING": 0.0, "FALLEN": 0.0, "WAKING": 0.8, "IDLE": 1.0, "ATTENDING": 1.0,
    "PLAYING": 0.6, "RESTING": 0.6, "TIRED": 0.4,
}


@dataclass
class Target:
    tid: int
    kind: str
    az: float                       # world frame
    el: float = 0.0
    dist: float | None = None
    sigma: float = math.radians(10)
    seen: float = 0.0               # last observed
    salience: float = 1.0
    facing: bool = False
    talking_until: float = -1e9
    mirror: float | None = None     # the other answer of a two-mic pair (world frame), until settled
    looked: float = -1e9            # last held as the focus
    news: float = 0.0               # last time it changed in a way worth a new look


@dataclass
class Focus:
    yaw: float                      # body frame, clamped to the head's range
    el: float
    kind: str
    tid: int | None
    mode: str                       # saccade | pursue | look_away | wander | none
    want_turn: float | None = None  # body-frame yaw to turn the body to, when held far round


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Attention:
    targets: dict = field(default_factory=dict)
    next_id: int = 1
    focus: Focus | None = None
    focus_since: float = -1e9
    far_since: float | None = None
    away_until: float = -1e9        # looking away in a conversation until
    next_away: float | None = None
    away_side: int = 1
    sectors: list = field(default_factory=lambda: [-1e9] * 8)   # last looked, per 45 deg sector (world)
    wander: tuple | None = None     # (world az, el, until)

    # ── observations ─────────────────────────────────────────────────────────────────────────────

    def observe(self, t: float, kind: str, yaw_body: float, heading: float = 0.0, el: float = 0.0,
                dist: float | None = None, sigma: float | None = None, facing: bool | None = None,
                talking: bool = False, mirror_body: float | None = None, salience: float = 1.0) -> int:
        """Something seen or heard at `yaw_body` (body frame) while the body faced `heading` (world).
        Merges into the target it most likely is, or starts one. Returns the target's id."""
        az = wrap(yaw_body + heading)
        mirror = None if mirror_body is None else wrap(mirror_body + heading)
        sigma = sigma if sigma is not None else math.radians(10)
        best, best_d = None, None
        for tg in self.targets.values():
            if not (tg.kind == kind or (tg.kind, kind) in MERGES or (kind, tg.kind) in MERGES):
                continue
            d = min(abs(wrap(tg.az - az)), abs(wrap(tg.az - mirror)) if mirror is not None else math.inf,
                    abs(wrap(tg.mirror - az)) if tg.mirror is not None else math.inf)
            gate = max(2 * tg.sigma, 2 * sigma, MERGE_MIN)
            if d <= gate and (best_d is None or d < best_d):
                best, best_d = tg, d
        if best is None:
            if len(self.targets) >= MAX_TARGETS:
                weakest = min(self.targets.values(), key=lambda x: KINDS[x.kind][0] * x.salience)
                del self.targets[weakest.tid]
            tg = Target(self.next_id, kind, az, el, dist, sigma, t, salience, bool(facing), mirror=mirror, news=t)
            if talking:
                tg.talking_until = t + 3.0
            self.targets[tg.tid] = tg
            self.next_id += 1
            return tg.tid
        tg = best
        # which answer of an unsettled pair this observation agrees with
        if tg.mirror is not None and abs(wrap(tg.mirror - az)) < abs(wrap(tg.az - az)):
            tg.az, tg.mirror = tg.mirror, None
        elif mirror is None and tg.mirror is not None and kind in ("person", "near"):
            tg.mirror = None                                    # an eye or the ToF settles it
        # inverse-variance blend of the direction
        w_old, w_new = 1 / tg.sigma ** 2, 1 / sigma ** 2
        tg.az = wrap(tg.az + wrap(az - tg.az) * w_new / (w_old + w_new))
        tg.sigma = math.sqrt(1 / (w_old + w_new))
        tg.el = el if kind in ("person", "object") or kind == tg.kind == "motion" else tg.el
        tg.dist = dist if dist is not None else tg.dist
        tg.seen = t
        tg.salience = max(tg.salience, salience)
        if facing is not None:
            tg.facing = facing
        if talking:
            tg.talking_until = t + 3.0
        # what it is now: a face makes it a person (and a person stays one, talking or near); its name
        # makes the rest a call; otherwise the more urgent of the two
        if kind == "person":
            tg.kind = "person"
        elif tg.kind != "person":
            if kind == "call" or KINDS[kind][0] > KINDS[tg.kind][0]:
                tg.kind = kind
        if kind in ("call", "near", "sound") or talking:
            tg.news = t
        self._absorb(tg)
        return tg.tid

    def _absorb(self, tg: Target):
        """Other targets the same thing (compatible, within the gate) folded into this one."""
        for other in list(self.targets.values()):
            if other is tg or not ((tg.kind, other.kind) in MERGES or (other.kind, tg.kind) in MERGES
                                   or tg.kind == other.kind):
                continue
            if abs(wrap(other.az - tg.az)) <= max(2 * tg.sigma, 2 * other.sigma, MERGE_MIN):
                tg.salience = max(tg.salience, other.salience)
                tg.talking_until = max(tg.talking_until, other.talking_until)
                tg.dist = tg.dist if tg.dist is not None else other.dist
                tg.news = max(tg.news, other.news)
                tg.facing = tg.facing or other.facing
                del self.targets[other.tid]

    def not_found(self, t: float, tid: int):
        """The eyes looked at a target and saw no one: an unsettled pair turns to its other answer;
        otherwise the target is less likely, and less worth looking at."""
        tg = self.targets.get(tid)
        if tg is None:
            return
        if tg.mirror is not None:
            tg.az, tg.mirror = tg.mirror, None
            tg.news = t
        else:
            tg.salience *= 0.4
            tg.sigma = min(KINDS[tg.kind][3], tg.sigma * 1.5)

    # ── time ─────────────────────────────────────────────────────────────────────────────────────

    def decay(self, t: float, dt: float):
        for tid in list(self.targets):
            tg = self.targets[tid]
            _, half, grow, smax, forget = KINDS[tg.kind]
            tg.salience *= 0.5 ** (dt / half)
            tg.sigma = min(smax, tg.sigma + grow * dt)
            if t - tg.seen > forget:
                if tg.kind == "person":        # remembered as where someone was
                    tg.kind, tg.salience, tg.seen = "place", 1.0, t
                else:
                    del self.targets[tid]

    # ── choosing ─────────────────────────────────────────────────────────────────────────────────

    def score(self, tg: Target, t: float, state: str, gaze_world: float, heat_ok: bool,
              conversing: bool) -> float:
        prio = KINDS[tg.kind][0]
        if tg.kind == "person":
            if tg.talking_until > t:
                prio = 0.9 if conversing else 0.75
            elif tg.facing:
                prio = 0.7
        s = prio * tg.salience * STATE_GAIN.get(state, 1.0)
        if tg.sigma > math.radians(15) and prio >= 0.5 and tg.looked < tg.news:
            s += 0.1                                   # unsure, it matters, not looked at since: worth a look
        turn = abs(wrap(tg.az - gaze_world))
        s -= turn * (0.05 if heat_ok else 0.2)
        current = self.focus is not None and self.focus.tid == tg.tid
        if current:
            engaged = tg.kind == "person" and (tg.talking_until > t or tg.facing)
            if not engaged and tg.news < self.focus_since and t - self.focus_since > BUDGET_S.get(tg.kind, 3.0):
                s -= IOR_COST                          # looked long enough with nothing new: bored
        elif t - tg.looked < IOR_S and tg.news < tg.looked:
            s -= IOR_COST                              # just looked at, nothing new since
        return s

    def select(self, t: float, state: str, heading: float, gaze_body: float, rng,
               conversing: bool = False, thinking: bool = False, walking: bool = False,
               heat_ok: bool = True, can_turn: bool = False) -> Focus:
        """Where to look now. `gaze_body`: where the head points (body frame). `thinking`: waiting on
        the brain's answer (the eyes leave the face, up and aside). `walking`: the walk policy has the
        head - only the table is kept. `can_turn`: the body may turn (standing, no mission, no pad)."""
        if walking or STATE_GAIN.get(state, 1.0) == 0.0:
            self.focus, self.far_since = None, None
            return Focus(0.0, 0.0, "none", None, "none")
        gaze_world = wrap(gaze_body + heading)
        scored = sorted(((self.score(tg, t, state, gaze_world, heat_ok, conversing), tg)
                         for tg in self.targets.values()), key=lambda x: -x[0])
        cur = self.targets.get(self.focus.tid) if self.focus and self.focus.tid is not None else None
        best_s, best = scored[0] if scored else (None, None)
        chosen = None
        if best is not None and best_s > 0.15:
            chosen = best
            if cur is not None and cur is not best and t - self.focus_since < DWELL_S:
                cur_prio = KINDS[cur.kind][0]
                urgent = best.kind == "call" and best.news > self.focus_since
                if not urgent and KINDS[best.kind][0] < cur_prio + INTERRUPT_MARGIN:
                    chosen = cur
        if chosen is None:
            return self._wander(t, heading, rng)
        if self.focus is None or self.focus.tid != chosen.tid:
            self.focus_since = t
            self.far_since = None
        chosen.looked = t
        self.sectors[int(((chosen.az + math.pi) % (2 * math.pi)) / (math.pi / 4)) % 8] = t
        yaw = wrap(chosen.az - heading)
        mode = "pursue" if t - self.focus_since > 0.5 else "saccade"
        el = chosen.el
        # a conversation: the eyes leave the face now and then (and while it thinks), and come back
        if conversing and chosen.kind == "person":
            if self.next_away is None:
                self.next_away = t + rng.uniform(4.0, 8.0)
            if thinking or t >= self.next_away:
                if self.away_until < t:
                    self.away_until = t + rng.uniform(0.6, 1.0)
                    self.away_side = rng.choice((-1, 1))
                    self.next_away = t + rng.uniform(4.0, 8.0)
            if t < self.away_until:
                yaw += 0.25 * self.away_side
                el += 0.12
                mode = "look_away"
        want_turn = None
        if abs(yaw) > TURN_BEYOND:
            self.far_since = self.far_since if self.far_since is not None else t
            if can_turn and t - self.far_since >= TURN_AFTER_S:
                want_turn = yaw
        else:
            self.far_since = None
        f = Focus(max(-HEAD_YAW_LIMIT, min(HEAD_YAW_LIMIT, yaw)), el, chosen.kind, chosen.tid, mode, want_turn)
        self.focus = f
        return f

    def _wander(self, t: float, heading: float, rng) -> Focus:
        """Nothing worth a look: drift to where it has not looked for longest (within the head's reach),
        or where someone was, a few seconds each."""
        if self.wander is None or t >= self.wander[2]:
            places = [tg for tg in self.targets.values() if tg.kind == "place"]
            if places and rng.random() < 0.4:
                p = rng.choice(places)
                az = p.az
            else:
                reach = [i for i in range(8) if abs(wrap((i + 0.5) * math.pi / 4 - math.pi - heading)) <= HEAD_YAW_LIMIT]
                i = min(reach or range(8), key=lambda k: self.sectors[k])
                az = (i + 0.5) * math.pi / 4 - math.pi + rng.uniform(-0.2, 0.2)
                self.sectors[i] = t
            self.wander = (az, rng.uniform(-0.05, 0.1), t + rng.uniform(2.0, 5.0))
        yaw = max(-HEAD_YAW_LIMIT, min(HEAD_YAW_LIMIT, wrap(self.wander[0] - heading)))
        self.focus = Focus(yaw, self.wander[1], "wander", None, "wander")
        return self.focus

    def snapshot(self, t: float, heading: float = 0.0) -> dict:
        """For the phone page: the targets round the body, and the focus."""
        return {"targets": [{"id": tg.tid, "kind": tg.kind, "yaw": round(wrap(tg.az - heading), 3),
                             "sigma": round(tg.sigma, 3), "salience": round(tg.salience, 2),
                             "dist": tg.dist, "mirror": None if tg.mirror is None else round(wrap(tg.mirror - heading), 3)}
                            for tg in self.targets.values()],
                "focus": None if self.focus is None else {"yaw": round(self.focus.yaw, 3), "kind": self.focus.kind,
                                                          "mode": self.focus.mode, "turn": self.focus.want_turn}}
