# -*- coding: utf-8 -*-
"""Score the HL-2915 stand-up, sit/stand and roulade policies in the BAM simulation.

Same simulator and stress conditions as `stress_hl2915.py` (leg servos at a fraction of the
fitted gain, a heavier head further forward, more delay), with each skill started the way
robotd starts it:

  standup   from each lying pose (and seated, and standing), joints first held at the
            standing pose as robotd's limp-fall sequence leaves them; zero command.
  sitstand  standing; sit flag 1 from 1 s to 7 s, then 0 (stand back up).
  roulade   standing; switched in at t = 0 with every command slot zero.

    python training/bam/eval_skills_hl2915.py --standup S.onnx --sitstand T.onnx --roulade R.onnx
"""
import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import sim_hl2915 as sh  # noqa: E402
from stress_hl2915 import CONDITIONS, legs  # noqa: E402

# Heads outside the training range (mass x0.95-1.3, CoM +5 mm, plus the stock ±10 mm CoM
# spread): how much margin is there before the weighed head says retrain?
HEAD_CONDITIONS = [
    ("head x1.5, +20 mm", ["--head-mass-scale", "1.5", "--head-com-forward-mm", "20"], 20),
    ("legs 0.5 + head x1.5, +20 mm", ["--kp-scale", legs(0.5), "--head-mass-scale", "1.5",
                                      "--head-com-forward-mm", "20"], 20),
    ("legs 0.5 + head x1.3, -10 mm", ["--kp-scale", legs(0.5), "--head-mass-scale", "1.3",
                                      "--head-com-forward-mm", "-10"], 20),
]

SCENE = os.path.join(ROOT, "hardware/rev-a/mjcf/scene.xml")  # all collisions, as these tasks train
PARAMS = os.path.join(ROOT, "training/bam/params/hl2915/m4.json")
ROBOTD_LOWPASS = (0.5, 0.7)  # robotd's default head / legs target low-pass (--lowpass)
UPRIGHT_GZ = -0.9   # gravity z in the trunk frame: -1 upright, 0 on the side
STAND_Z = 0.10      # trunk height that counts as standing (stands at ~0.115, sits ~0.06)
SIT_Z = 0.085

ZERO = {"twist": [0.0, 0.0, 0.0], "head": [0.0] * 4, "body": [0.0] * 3}


def sit_schedule(t):
    return dict(ZERO, twist=[1.0 if 1.0 <= t < 7.0 else 0.0, 0.0, 0.0])


def args_for(policy, extra, delay, init, duration, cmd_fn=None, lowpass=(1.0, 1.0), backlash=""):
    ap = argparse.Namespace(
        scene=SCENE, policy=policy, params=PARAMS, position=False, kp=16, kp_head=16, vin=10.7,
        current_ma=1300.0, delay_ms=delay, timestep=0.001, settle=1.0, settle_kp=16,
        action_scale=1.0, vx=0.0, duration=duration, video=None, kp_scale="",
        head_mass_scale=1.0, head_com_forward_mm=0.0, ramp_s=0.0, init=init, cmd_fn=cmd_fn,
        head_lowpass=lowpass[0], legs_lowpass=lowpass[1], backlash_deg=backlash, backlash_seed=0, friction=None)
    it = iter(extra)
    for flag in it:
        setattr(ap, flag.lstrip("-").replace("-", "_"), type(getattr(ap, flag.lstrip("-").replace("-", "_")))(next(it)))
    return ap


def standing(log):
    return (np.asarray(log["z"]) > STAND_Z) & (np.asarray(log["gz"]) < UPRIGHT_GZ)


def first_held(mask, hold=25):
    """First tick from which `mask` stays true for `hold` ticks (0.5 s), in seconds."""
    run = 0
    for i, m in enumerate(mask):
        run = run + 1 if m else 0
        if run >= hold:
            return (i - hold + 1) / 50.0
    return None


def standing_jitter(log, seconds=2.0):
    """Mean |target step| per tick over the last `seconds`, degrees: all joints, and the worst
    joint. On the robot (2026-10-08) standup_v3bl stood at 0.77° (ankle 2.2°) and shook at
    ~5 Hz; standup_v1 at 0.22°."""
    tg = np.degrees(np.asarray(log["target"])[-int(seconds * 50):])
    step = np.abs(np.diff(tg, axis=0)).mean(0)
    return step.mean(), step.max()


def score_standup(log):
    up = standing(log)
    t = first_held(up)
    ok = bool(up[-25:].all())
    jit, worst = standing_jitter(log)
    return ok, ("up %.1fs" % t if ok and t is not None else "FAIL") + " j%.2f/%.1f" % (jit, worst)


def score_sitstand(log):
    z, gz, tt = np.asarray(log["z"]), np.asarray(log["gz"]), np.asarray(log["t"])
    sat = (z < SIT_Z) & (gz < -0.7)
    sit_ok = bool(sat[(tt > 6.0) & (tt < 7.0)].all())
    sit_t = first_held(sat)
    up = standing(log)
    rise_ok = bool(up[-25:].all())
    rise_t = first_held(up[tt >= 7.0]) if rise_ok else None
    tilt = np.degrees(np.arccos(np.clip(-gz, -1, 1))).max()
    s = ("sit %.1fs" % (sit_t - 1.0) if sit_ok and sit_t else "no-sit") + " / " + \
        ("rise %.1fs" % rise_t if rise_ok and rise_t is not None else "no-rise")
    return sit_ok and rise_ok, s + " tilt%3.0f" % tilt


def score_roulade(log):
    gyro = np.asarray(log["gyro"])
    rot = np.degrees(np.cumsum(gyro[:, 1]) / 50.0)
    total = rot[np.argmax(np.abs(rot))]
    up = standing(log)
    ok = abs(total) > 300 and bool(up[-25:].all())
    done = np.argmax(np.abs(rot) > 330) / 50.0 if (np.abs(rot) > 330).any() else None
    back = first_held(up[int(done * 50):]) if done is not None else None
    s = "rot%+4.0f" % total
    if done is not None:
        s += " over %.1fs" % done
        if back is not None:
            s += " up %.1fs" % (done + back)
    return ok, s if ok else s + " FAIL"


SKILLS = {
    "standup": (["face_down", "face_up", "face_up_rolled", "side_left", "side_right", "sit", "stand"],
                8.0, None, score_standup),
    "sitstand": (["stand"], 12.0, sit_schedule, score_sitstand),
    "roulade": (["stand"], 6.0, None, score_roulade),
}


def job(skill, policy, cond, init, lowpass=(1.0, 1.0), backlash=""):
    label, extra, delay = cond
    inits, duration, cmd_fn, score = SKILLS[skill]
    log = sh.run(args_for(policy, extra, delay, init, duration, cmd_fn, lowpass, backlash), SCENE, policy, ZERO)
    return skill, label, init, score(log)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for s in SKILLS:
        ap.add_argument("--" + s)
    ap.add_argument("--head", action="store_true", help="the out-of-range head conditions instead")
    ap.add_argument("--lowpass", action="store_true", help="robotd's default target low-pass (v3 policies train with it)")
    ap.add_argument("--backlash-deg", default="", help='gear play in the sim, e.g. "0.35" (measured); the backlash '
                    "policies were trained with it and misread a play-free sim")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    a = ap.parse_args()
    conditions = HEAD_CONDITIONS if a.head else CONDITIONS
    lowpass = ROBOTD_LOWPASS if a.lowpass else (1.0, 1.0)
    jobs = [(s, getattr(a, s), c, i, lowpass, a.backlash_deg) for s in SKILLS if getattr(a, s)
            for c in conditions for i in SKILLS[s][0]]
    with ProcessPoolExecutor(a.workers) as ex:
        results = list(ex.map(job, *zip(*jobs)))
    for s in SKILLS:
        rows = [r for r in results if r[0] == s]
        if not rows:
            continue
        print(f"\n== {s}: {os.path.basename(getattr(a, s))}")
        for label, *_ in conditions:
            cells = [(i, r) for (_, l, i, r) in rows if l == label]
            n_ok = sum(ok for _, (ok, _) in cells)
            print(f"  {label:<30} {n_ok}/{len(cells)}  " +
                  "  ".join(f"{i}:{txt}" for i, (_, txt) in cells))


if __name__ == "__main__":
    main()
