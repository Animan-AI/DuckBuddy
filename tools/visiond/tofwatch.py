# -*- coding: utf-8 -*-
"""Someone coming close, from the head ToF (VL53L5CX 8x8, tofd's tof.stream on /tmp/tofd.sock).

The middle 4x4 zones are what the duck is facing. When their median distance drops below NEAR_M
from a background at least GAP_M farther (the last few seconds' level), for NEAR_FRAMES frames in a
row, that is someone (a person, a hand) arriving in front of it: a `person` event for lifed, which
turns to them and says hello if it is awake. Once per COOLDOWN_S.

Pure detector (`Approach.step`) plus a thread that reads tofd; visiond runs it.
"""
from __future__ import annotations

import json
import socket
import statistics
import threading
import time
from collections import deque

VALID = (5, 9)              # VL53L5CX target status: range valid (9: valid, large pulse)
NEAR_M = 0.7                # nearer than this...
GAP_M = 0.35                # ... and this much nearer than the background
NEAR_FRAMES = 3             # 0.2 s at 15 Hz
BACKGROUND_S = 3.0
COOLDOWN_S = 15.0
CENTRE = [r * 8 + c for r in range(2, 6) for c in range(2, 6)]


def centre_m(frame: dict) -> float | None:
    """Median distance (m) of the valid middle zones, None if fewer than 4 are valid."""
    d, st = frame.get("distance_mm") or [], frame.get("status") or []
    if len(d) < 64:
        return None
    vals = [d[i] / 1000.0 for i in CENTRE if st[i] in VALID and d[i] > 0]
    return statistics.median(vals) if len(vals) >= 4 else None


class Approach:
    """Feed it one depth reading per frame (None: nothing valid); it says when someone arrived."""

    def __init__(self, hz: float = 15.0):
        self.background = deque(maxlen=int(BACKGROUND_S * hz))
        self.near_run = 0
        self.last_event = -1e9

    def step(self, t: float, dist: float | None) -> float | None:
        """The distance of someone who just arrived, or None."""
        if dist is None:
            # nothing in range (open space, a far ceiling): background is "far"
            self.near_run = 0
            self.background.append(4.0)
            return None
        bg = statistics.median(self.background) if len(self.background) >= 5 else None
        if dist < NEAR_M and (bg is None or bg - dist >= GAP_M):
            self.near_run += 1
        else:
            self.near_run = 0
            self.background.append(dist)
        if self.near_run == NEAR_FRAMES and bg is not None and t - self.last_event >= COOLDOWN_S:
            self.last_event = t
            return dist
        return None


def watch(sock_path: str, on_person, stop: threading.Event, log=print):
    """Read tofd forever (reconnecting) and call on_person(dist_m) when someone arrives."""
    det = Approach()
    while not stop.is_set():
        try:
            s = socket.socket(socket.AF_UNIX)
            s.settimeout(5.0)
            s.connect(sock_path)
            s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"tof.stream"}\n')
            f = s.makefile("rb")
            log(f"tof: reading {sock_path}")
            for line in f:
                if stop.is_set():
                    break
                m = json.loads(line)
                if m.get("method") != "tof.frame":
                    continue
                p = m["params"]
                dist = det.step(p.get("at_us", 0) / 1e6, centre_m(p))
                if dist is not None:
                    on_person(dist)
        except (OSError, ValueError) as e:
            log(f"tof: {e!r}; retrying in 5 s")
            time.sleep(5.0)
