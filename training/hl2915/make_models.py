# -*- coding: utf-8 -*-
"""Build the rev-a (HL-2915, measured masses) copy of microduck_rl's robot models.

microduck_rl's `src/mjlab_microduck/robot/microduck/` has drifted from
`hardware/original/mjcf` (STAND2 keyframe, backlash variants, collision edits), so the
mass change is applied to *its* XMLs rather than shipping ours: every XL330 becomes a
28 g HL-2915 and everything else is scaled so the robot weighs the measured total —
the same transform as `hardware/rev-a/mjcf/mjcf_mass.py`, which this imports.

    python make_models.py <microduck_rl>/src/mjlab_microduck/robot/microduck \
                          <microduck_rl>/src/mjlab_microduck/robot/microduck_hl2915 --total 0.850

XMLs without XL330 parts (ball, scenes, defaults) are copied unchanged; `assets/` is a
symlink, so the meshes are not duplicated.
"""
import argparse
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "hardware", "rev-a", "mjcf"))
import mjcf_mass  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("out")
ap.add_argument("--total", type=float, default=mjcf_mass.MEASURED_TOTAL, help="kg, with battery")
a = ap.parse_args()

os.makedirs(a.out, exist_ok=True)
for name in sorted(os.listdir(a.src)):
    src = os.path.join(a.src, name)
    dst = os.path.join(a.out, name)
    if name == "assets":
        if not os.path.lexists(dst):
            os.symlink(os.path.abspath(src), dst)
        continue
    if os.path.isdir(src) or name.endswith((".pyc", ".py")):
        continue
    text = open(src, encoding="utf-8", errors="replace").read() if name.endswith(".xml") else ""
    if "<!-- Part xl330" in text:
        report = mjcf_mass.process(src, dst, a.total)
        before = mjcf_mass.total_mass(text)
        after = mjcf_mass.total_mass(open(dst, encoding="utf-8").read())
        print(f"{name:<44} {len(report):2d} bodies  {before * 1000:6.1f} g -> {after * 1000:6.1f} g")
    else:
        shutil.copy2(src, dst)
