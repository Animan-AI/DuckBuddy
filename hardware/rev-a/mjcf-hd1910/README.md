# 小黑 HD-1910（2S）MuJoCo 模型

模型总质量为 806 g，按小黑装上 2S 电池后的实测重量设定。每只 HD-1910（含舵盘）按 22.8 g 建模。模型复用相邻目录 `../mjcf/assets/` 的 STL 网格，下载时请保留仓库目录结构。

- [scene_walk.xml](scene_walk.xml) · 行走场景。
- [scene.xml](scene.xml) · 完整碰撞场景，可用于起立仿真。
- [robot_allcollisions_rollers.xml](robot_allcollisions_rollers.xml) · 带滚轮关节的机器人模型。

安装 MuJoCo 后，在仓库根目录运行：

```bash
python -m mujoco.viewer --mjcf=hardware/rev-a/mjcf-hd1910/scene_walk.xml
```

网格与原始模型基于 Pollen Robotics 的 Microduck。

## 行走策略

[HD-1910 行走策略（Kp 16，checkpoint 6500）](../../../training/policies/hd1910_walk_kp16_6500.onnx) 是 `-r3` 镜像为 HD-1910（2S）配置的基线策略。加载 ONNX 时需沿用 DuckBuddy 控制器的观测与动作接口。
