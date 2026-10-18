# -*- coding: utf-8 -*-
"""What the eyes tell lifed after Jev looked (docs/project/duck-life-jev-framework-20261017-CN.md §4.2).

The ToF says something came close; the camera takes one colour frame; Jev (duckbrain /judge
`person`) says whether a real person is there, what they are doing and where the face is. This turns
that into lifed's `person` event — or, with no answer, the bare ToF one lifed has always had.
Pure (the frame helpers need OpenCV only when called).
"""
from __future__ import annotations

import math

T_PERSON = 0.60        # a real person (not the wall the ToF saw)
T_ACTIVITY = 0.40      # below this the activity is "unclear"
HFOV_DEG = 62.0        # IMX219 at 1920x1080


def _choice(answers: dict, name: str) -> tuple[str | None, float]:
    a = answers.get(name) or {}
    c = a.get("choice")
    p = (a.get("probabilities") or {}).get(c, a.get("confidence", 0.0)) if c else 0.0
    return c, float(p or 0.0)


def person_event(answers: dict | None, dist: float | None) -> dict:
    """lifed's `person` event: present, activity, facing, yaw (radians, + left, from the face box,
    relative to where the head was looking) — the bare ToF event without an answer."""
    base = {"type": "person", "yaw": 0.0, "dist": dist, "via": "tof"}
    if not answers:
        return base
    present = float((answers.get("person") or {}).get("noul", 0.0)) >= T_PERSON
    activity, p_act = _choice(answers, "activity")
    ev = {**base, "via": "jev", "present": present,
          "activity": activity if (activity and p_act >= T_ACTIVITY) else "unclear",
          "facing": round(float((answers.get("facing") or {}).get("noul", 0.0)), 2)}
    box = answers.get("face") or {}
    if present and {"left", "right"} <= box.keys():
        cx = (float(box["left"]) + float(box["right"])) / 2.0
        ev["yaw"] = round(math.radians((0.5 - cx) * HFOV_DEG), 3)     # right of centre: turn right (−)
    return ev


def nv12_to_jpeg(nv12, width: int, height: int, out_w: int = 960, quality: int = 80) -> bytes:
    """A colour JPEG for Jev from duckcam's NV12 frame: the bottom 16 rows dropped (their chroma is
    never written), scaled to `out_w` wide — inside Jev's 280-1568 px per side."""
    import cv2
    bgr = cv2.cvtColor(nv12.reshape(height * 3 // 2, width), cv2.COLOR_YUV2BGR_NV12)[: height - 16]
    h = round(bgr.shape[0] * out_w / bgr.shape[1])
    small = cv2.resize(bgr, (out_w, h), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return buf.tobytes()
