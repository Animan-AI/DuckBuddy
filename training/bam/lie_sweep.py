"""Where does a limp rev-a duck come to rest from a candidate lying pose? Actuators off, joint
friction only; drop from just above the floor, settle 4 s, report trunk pitch/roll/z, which bodies
touch the floor, and how far the joints drifted from the start pose."""
import sys, numpy as np, mujoco
sys.path.insert(0, "training"); import sim_walk as sw
m = mujoco.MjModel.from_xml_path("hardware/rev-a/mjcf/scene.xml")
m.actuator_gainprm[:] = 0; m.actuator_biasprm[:] = 0           # unpowered
FRIC = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
for j in range(m.njnt):
    if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
        m.dof_frictionloss[m.jnt_dofadr[j]] = FRIC
d = mujoco.MjData(m)
J = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in sw.POLICY_JOINTS}
floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
def run(q14, pitch_deg):
    mujoco.mj_resetData(m, d)
    for n, v in zip(sw.POLICY_JOINTS, q14): d.qpos[J[n]] = v
    quat = np.zeros(4); mujoco.mju_euler2Quat(quat, [0, np.radians(pitch_deg), 0], "xyz"); d.qpos[3:7] = quat
    d.qpos[2] = 0.3; mujoco.mj_forward(m, d)
    low = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom) if g != floor and m.geom_contype[g] | m.geom_conaffinity[g])
    d.qpos[2] -= low - 0.005; mujoco.mj_forward(m, d)
    for _ in range(4000): mujoco.mj_step(m, d)
    R = d.xmat[1].reshape(3, 3); g = R.T @ np.array([0, 0, -1.0])
    pitch = np.degrees(np.arctan2(-g[0], -g[2])); roll = np.degrees(np.arctan2(g[1], -g[2]))
    drift = np.degrees(np.abs(np.array([d.qpos[J[n]] for n in sw.POLICY_JOINTS]) - q14)).max()
    touch = sorted({m.body(m.geom_bodyid[c.geom1 if c.geom2 == floor else c.geom2]).name for c in d.contact[:d.ncon] if floor in (c.geom1, c.geom2)})
    speed = np.abs(d.qvel).max()
    return pitch, roll, d.qpos[2], drift, touch, speed
cands = {
  # name: legs (hip_pitch, knee, ankle) mirrored, neck, head_pitch, trunk pitch at drop (deg, + = nose down)
  "sit-fold, head fwd":      (-0.41, 1.35, 0.0, -0.3, 0.8, 30),
  "sit-fold, head tucked":   (-0.41, 1.35, 0.0,  0.4, 1.5, 30),
  "prone, legs back":        ( 0.6,  1.4, -0.6, -0.3, 0.6, 70),
  "prone, legs tucked":      (-0.9,  1.6,  0.3, -0.3, 0.6, 60),
  "prone, head on floor":    (-0.9,  1.6,  0.3, -0.9, 0.3, 60),
  "kneel, chest down":       (-1.2,  1.5,  0.5, -0.5, 0.5, 45),
}
print(f"joint friction {FRIC} N·m")
for name, (hp, kn, an, nk, hd, pt) in cands.items():
    q = np.array(sw.HOME_14, dtype=float)
    q[2], q[3], q[4] = hp, kn, an; q[11], q[12], q[13] = -hp, -kn, -an
    q[1] = q[10] = 0.0; q[5], q[6] = nk, hd
    pitch, roll, z, drift, touch, speed = run(q, pt)
    print(f"{name:<24} -> trunk pitch {pitch:+6.1f} roll {roll:+5.1f} z {z*1000:5.1f} mm | joint drift max {drift:5.1f} deg | still {speed:.3f} | floor: {', '.join(touch)}")
