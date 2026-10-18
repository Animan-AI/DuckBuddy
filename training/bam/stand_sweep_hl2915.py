"""Stand-up policy holding the robot (as robotd's stand slot, legs x0.8 gain) under robot-matched
latencies: pitch-rate RMS, dominant frequency, ankle target std, for standing action scales."""
import os, sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE)); sys.path.insert(0, HERE)
STAND = os.path.join(ROOT, "training/policies/hl2915_standup_v3bl_5999.onnx")
WALK = os.path.join(ROOT, "training/policies/hl2915_walk_v6lp_3250.onnx")
def one(job):
    import bam.actuator, bam.model, bam.mujoco  # noqa: F401  the installed BAM, before sim_hl2915 puts training/ (and its bam/) first
    import sim_hl2915 as sh, videos_hl2915 as v
    scale, lag, ratio, bl, seed = job
    a = v.sh_args(WALK, 0.0, 6.2, "640x480", None, None, 1.0, v.SIDE); a.video = None
    a.pre_policy, a.pre_s, a.pre_kp_ratio = STAND, 6.0, ratio
    a.delay_ms, a.imu_lag_ms, a.vin = 32.0, lag, 11.6
    a.backlash_deg, a.backlash_seed = bl, seed
    a.action_scale = scale  # the sim scales the stand-up policy's action too
    log = sh.run(a, a.scene, WALK, {"twist": [0, 0, 0], "head": [0] * 4, "body": [0] * 3})
    i0, i1 = 100, 300
    gy = np.degrees(log["gyro"][i0:i1, 1]); g = gy - gy.mean()
    F = np.abs(np.fft.rfft(g * np.hanning(len(g)))); fr = np.fft.rfftfreq(len(g), 0.02)
    ank = np.degrees(log["target"][i0:i1, 4]).std()
    return job, float(np.sqrt((gy ** 2).mean())), float(fr[1 + np.argmax(F[1:])]), float(ank)
if __name__ == "__main__":
    jobs = [(s, lag, r, bl, seed) for s in (1.0, 0.8, 0.6) for lag in (0.0, 20.0, 30.0) for r in (0.8,) for bl, seed in (("0.35", 0), ("0.3:1.0", 1), ("0.3:1.0", 2))]
    with ProcessPoolExecutor(10) as ex: res = list(ex.map(one, jobs))
    import collections; agg = collections.defaultdict(list)
    for (s, lag, r, bl, seed), rms, f, ank in res: agg[(s, lag)].append((rms, f, ank))
    for k, v in agg.items():
        v = np.array(v); print(f"scale {k[0]} imu lag {k[1]:>4.0f} ms: pitch-rate RMS {v[:,0].mean():5.1f} deg/s  peak {np.median(v[:,1]):.1f} Hz  ankle target std {v[:,2].mean():.2f} deg")
