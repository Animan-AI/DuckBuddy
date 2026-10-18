# -*- coding: utf-8 -*-
"""What moves catches the eye (docs/project/attention-gaze-design-20261018-CN.md §1.3): the eyes' watch
mode compares each small grey frame with the one before and reports the largest moving patch - a hand,
a ball, a cat - as a point in the frame. A frame where much of the picture changed is the head (or the
light) moving, not something in the room: nothing is reported. lifed also drops what comes while its own
head was turning. Pure numpy/OpenCV, tested in test_motion.py.
"""
from __future__ import annotations

SMALL = (160, 90)          # the grey frame compared, w x h
DIFF_LEVEL = 25            # grey levels: less is sensor noise and the ISP's small exposure steps
MIN_FRACTION = 0.002       # of the frame: less is noise (~29 pixels at 160x90)
MAX_FRACTION = 0.15        # more: the camera moved, or the light changed
MIN_BLOB_PX = 12


def small_grey(y_plane):
    """The luma plane (h x w, uint8) as the blurred SMALL grey frame motion is looked for in."""
    import cv2
    import numpy as np
    step = max(1, y_plane.shape[1] // (SMALL[0] * 3))   # 1920 wide: every 4th pixel, then averaged
    g = cv2.resize(np.ascontiguousarray(y_plane[::step, ::step]), SMALL, interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(g, (5, 5), 0)


def moving_blob(prev, cur):
    """(cx, cy, fraction, blob_fraction) of the largest moving patch between two SMALL grey frames,
    in SMALL pixels - or None: nothing moved, or too much did."""
    import cv2
    import numpy as np
    if prev is None or prev.shape != cur.shape:
        return None
    mask = (cv2.absdiff(prev, cur) > DIFF_LEVEL).astype(np.uint8)
    frac = float(mask.mean())
    if not (MIN_FRACTION <= frac <= MAX_FRACTION):
        return None
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < MIN_BLOB_PX:
        return None
    return float(cents[i][0]), float(cents[i][1]), frac, area / mask.size
