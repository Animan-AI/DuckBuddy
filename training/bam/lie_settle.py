import sys, numpy as np, mujoco
sys.path.insert(0, "training"); import sim_walk as sw
m = mujoco.MjModel.from_xml_path("hardware/rev-a/mjcf/scene.xml")
m.actuator_gainprm[:] = 0; m.actuator_biasprm[:] = 0
for j in range(m.njnt):
    if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE: m.dof_frictionloss[m.jnt_dofadr[j]] = 0.1
d = mujoco.MjData(m)
J = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in sw.POLICY_JOINTS}
floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
q = np.array(sw.HOME_14, dtype=float); q[2], q[3], q[4] = -0.9, 1.6, 0.3; q[11], q[12], q[13] = 0.9, -1.6, -0.3
q[1] = q[10] = 0.0; q[5], q[6] = -0.9, 0.3
quat = np.zeros(4); mujoco.mju_euler2Quat(quat, [0, np.radians(60), 0], "xyz")
mujoco.mj_resetData(m, d)
for n, v in zip(sw.POLICY_JOINTS, q): d.qpos[J[n]] = v
d.qpos[3:7] = quat; d.qpos[2] = 0.3; mujoco.mj_forward(m, d)
low = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom) if g != floor and m.geom_contype[g] | m.geom_conaffinity[g])
d.qpos[2] -= low - 0.005
for _ in range(5000): mujoco.mj_step(m, d)
final = np.array([d.qpos[J[n]] for n in sw.POLICY_JOINTS])
R = d.xmat[1].reshape(3, 3); g = R.T @ np.array([0, 0, -1.0])
print("trunk z %.4f  pitch %.1f deg  (gravity in body %s)" % (d.qpos[2], np.degrees(np.arctan2(-g[0], -g[2])), np.round(g, 3)))
for n, v in zip(sw.POLICY_JOINTS, final): print(f"  {n:<16} {v:+.4f} rad  {np.degrees(v):+6.1f} deg")
# robustness: perturb the settled pose and settle again
rng = np.random.default_rng(0); worst = 0
for k in range(10):
    mujoco.mj_resetData(m, d)
    for n, v in zip(sw.POLICY_JOINTS, final + rng.normal(0, 0.1, 14)): d.qpos[J[n]] = v
    p = np.radians(np.degrees(np.arctan2(-g[0], -g[2])) + rng.normal(0, 5))
    mujoco.mju_euler2Quat(quat, [0, -p, 0], "xyz"); d.qpos[3:7] = quat; d.qpos[2] = 0.3; mujoco.mj_forward(m, d)
    low = min(d.geom_xpos[gg][2] - m.geom_rbound[gg] for gg in range(m.ngeom) if gg != floor and m.geom_contype[gg] | m.geom_conaffinity[gg])
    d.qpos[2] -= low - 0.005
    for _ in range(4000): mujoco.mj_step(m, d)
    R2 = d.xmat[1].reshape(3, 3); g2 = R2.T @ np.array([0, 0, -1.0])
    worst = max(worst, abs(np.degrees(np.arctan2(-g2[0], -g2[2])) - np.degrees(np.arctan2(-g[0], -g[2]))))
print("perturbed x10 (joints ±0.1 rad, trunk ±5°): worst final trunk pitch difference %.1f deg" % worst)
