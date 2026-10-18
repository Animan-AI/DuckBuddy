# -*- coding: utf-8 -*-
"""Faces for the eyes' watch mode (docs/project/attention-gaze-design-20261018-CN.md §1.3): YuNet (OpenCV
Zoo, models/) on a small colour frame, and where a face is as angles from the camera's axis - in the
robot's convention, yaw positive to the left, pitch positive up. The life layer adds the head's pose.
The angle maths is pure and tested in test_faces.py.
"""
from __future__ import annotations

import math
import os

HFOV_DEG = 62.0           # the IMX219 as mounted (jevlook.py uses the same)
# 320x180: a face is found down to ~10 px, ~3.5 m away - a room; 13 ms on one A733 core (640x360: 43 ms
# on eight, 170 ms of CPU - OpenCV's threads mostly spin)
DETECT_SIZE = (320, 180)
MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "face_detection_yunet_2023mar.onnx")


def face_angles(cx: float, cy: float, width: float, height: float, hfov_deg: float = HFOV_DEG) -> tuple[float, float]:
    """(yaw, pitch) in radians of the image point (cx, cy): the image's right is the robot's right
    (negative yaw), its top is up (positive pitch). Square pixels, the axis at the centre."""
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return -math.atan((cx - width / 2) / f), -math.atan((cy - height / 2) / f)


def small_bgr(nv12, width: int, height: int, size=DETECT_SIZE):
    """duckcam's NV12 frame (flat or (h*3/2, w)) as a small BGR image, cheaply: every 2nd pixel of luma
    and chroma, area-averaged down to `size`, converted at that size. The bottom 16 rows (chroma never
    written) are dropped."""
    import cv2
    import numpy as np
    f = nv12.reshape(height * 3 // 2, width)
    hh = height - 16
    y = np.ascontiguousarray(f[:hh:2, ::2])
    uv = np.ascontiguousarray(f[height:height + hh // 2].reshape(hh // 2, width // 2, 2)[::2, ::2])
    w, h = size
    y = cv2.resize(y, (w, h), interpolation=cv2.INTER_AREA)
    uv = cv2.resize(uv, (w // 2, h // 2), interpolation=cv2.INTER_AREA).reshape(h // 2, w)
    return cv2.cvtColor(np.vstack([y, uv]), cv2.COLOR_YUV2BGR_NV12)


class FaceFinder:
    """YuNet at DETECT_SIZE. `find(bgr)` -> [(x, y, w, h, score)] in that frame's pixels, best first
    (largest: the nearest)."""

    def __init__(self, model: str = MODEL, score: float = 0.7):
        import cv2
        self.det = cv2.FaceDetectorYN.create(model, "", DETECT_SIZE, score, 0.3, 20)

    def find(self, bgr) -> list[tuple[float, float, float, float, float]]:
        _, faces = self.det.detect(bgr)
        if faces is None:
            return []
        out = [(float(f[0]), float(f[1]), float(f[2]), float(f[3]), float(f[-1])) for f in faces]
        return sorted(out, key=lambda b: -b[2] * b[3])
