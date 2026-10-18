# 训练产出的策略（HL-2915 rev-a）

全部是 `obs[1,61] → actions[1,14]`，robotd 直接加载。部署时 `[policy] action_scale` 和
`standing_action_scale` 都设 1.0（训练时的动作缩放），`[bus] position_kp` / `position_kp_head` 都设 16。

| 文件 | 来源 | 状态 |
|---|---|---|
| `walk-baseline.onnx` | 原版机器人（XL330）上训的行走基线，2026-08-30 | `training/sim_walk.py` 的默认策略 |
| `hl2915_walk_v1_3999.onnx` | `Mjlab-Velocity-Flat-HL2915`，rev-a 850 g + BAM m4，每环境统一的随机化 | 2026-10-05 实机：站立很稳，0.2 m/s 走 2.9 s 后前扑（见 `training/data/2026-10-05-hl2915-realtests/`） |
| `hl2915_walk_v2_3999.onnx` | 同上 + 每关节刚度 ×0.4–1.1、摩擦 ×0.7–2.0，头部质量 ×0.95–1.3、质心前移 5 mm | 仿真压力测试（`training/bam/stress_hl2915.py`）全部通过，颈部不再前扎；待实机 |
| `hl2915_walk_v2bl_3999.onnx` | v2 的齿轮间隙版（`Mjlab-Velocity-Flat-Backlash-HL2915`） | 同上，高延迟下走得更远；待实机 |

| `hl2915_standup_v1_5999.onnx` | `Mjlab-StandUp-Flat-HL2915`（随机化同 v2）：从趴 / 仰 / 侧躺 / 坐姿起身，站稳后跟随身体姿态指令 | 仿真 5 种压力条件 × 7 种起始姿态全部起身（0.3–1.0 s）；待实机。robotd `stand` 槽 |
| `hl2915_sitstand_v1_3999.onnx` | `Mjlab-SitStand-Flat-HL2915`：坐下标志在 twist vx 槽（1 坐 / 0 站） | 仿真 5 种条件全部坐下并站回；待实机。robotd `sitstand` 槽 |
| `hl2915_roulade_v1_6999.onnx` | `Mjlab-Roulade-Flat-HL2915`：站姿起手，头顶着地翻过去，背着地后坐起、站起 | 仿真 5 种条件全部完成（翻转 360° 约 1.0 s，1.1 s 站稳）；头部 ×1.5、质心前移 20 mm 时翻不过去。robotd `roulade` 槽 |

| `hl2915_walk_v3bl_3999.onnx` | v3：齿轮间隙 + 训练内含 robotd 低通（头 0.5 / 腿 0.7）+ 头部 DR 居中（称重后） | BAM 仿真（带低通）5 种条件全走；待实机 |
| `hl2915_standup_v3bl_5999.onnx` | v3 起身：同上 + 3000 迭代后加头部关节速度惩罚 | BAM 仿真 35/35 起身；颈不再撞 −90° 限位（最低 −65°），甩头峰速 540 → 360–490°/s；待实机 |
| `hl2915_sitstand_v3bl_3999.onnx` | v3 坐/站 | 训练指标与 v1 相同，但在**无齿轮间隙**的 BAM 仿真里正常条件下不坐 —— v3bl 策略依赖训练中的齿轮间隙回读，BAM 仿真评不准；待实机 |

v3 部署：robotd 低通保持默认（0.5 / 0.7，训练已含），`standing_gain_ratio = 1.0`。

v4 起不再取最后一个 checkpoint，按 `training/bam/checkpoint_sweep_hl2915.py` 的扫描选（同一次训练里各 checkpoint
直走偏航从 −30 到 +42 °/s 不等，训练指标看不出；结果在 `training/data/sim-hl2915/sweep-*/`）。以下由
`training/hl2915/ckpt_onnx.py` 在 CPU 上导出（与 export.py 的输出差 <1e-5）。

| 文件 | 来源 | 状态 |
|---|---|---|
| `hl2915_walk_v2bl_3250.onnx` | v2bl 第 3250 轮 | 仿真 0.2 m/s 偏航 +5~+14°/8 s（3999 为 −88~−133°），不摔；实机用来校准仿真偏航排名；待实机 |
| `hl2915_walk_v4bl_3500.onnx` | v4：v2bl 配方 + 每关节每回合随机间隙 ±0.3–1.0°（`Velocity-Flat-Backlash`，服务器 215ea1a） | 训练指标与 v2bl 相同（回合 959 / 摔 8.5%）；0.2 m/s 偏航 0~+11°/8 s，压力 0/20 摔（3999 偏 +27~+120°）；待实机 |
| `hl2915_walk_v4bl_lowfric_3999.onnx` | v4 + 脚底摩擦 0.4–1.3（`Velocity-Flat-Backlash-LowFriction`） | **扫描最佳**：偏航 −1~−11°/8 s、走到指令距离 71% 以上、压力 0/20 摔；待实机 |

| `hl2915_standup_v4bl_5999.onnx` | v4 起身：v2bl 配方（训练不含低通）+ 每回合随机间隙 ±0.3–1.0° + 头部速度惩罚 | BAM 仿真（间隙 ±0.35°、低通）35/35 起身 0.3–1.0 s；腿 ×0.4 时不自激（v3bl 同条件 2/7 起始姿势每拍跳 ~3°）；待实机 |

v4 部署同 v2bl：低通 0.5 / 0.7（训练不含），`standing_gain_ratio = 0.8`（起身策略仍是 v3bl 时）。

仿真里复测：

```bash
python training/bam/stress_hl2915.py training/policies/hl2915_walk_v2_3999.onnx training/policies/hl2915_walk_v2bl_3999.onnx
python training/bam/eval_skills_hl2915.py --standup training/policies/hl2915_standup_v1_5999.onnx \
    --sitstand training/policies/hl2915_sitstand_v1_3999.onnx --roulade training/policies/hl2915_roulade_v1_6999.onnx
python training/bam/eval_skills_hl2915.py --head ...   # 超出训练范围的头部
```

起身 / 坐站 / 翻滚策略部署时：`standing_gain_ratio = 1.0`（训练全程 Kp 16，默认 0.8 会把站立时降到 13）；
`stand` 槽换成起身策略之后 `[safety] limp_fall_hand_back = true`（摔倒后交给它起身）；`roulade_duration = 1.0` 与仿真吻合。
录像与分帧图：`training/data/sim-hl2915/skills/`。

### v5 与 v4 技能（2026-10-10）

| 文件 | 来源 | 状态 |
|---|---|---|
| `hl2915_walk_v5bl_delay50_2000.onnx` | v5：v4 配方 + 指令延迟 10–50 ms（`Velocity-Flat-Backlash-Delay50`，实测实机等效延迟 ≈40 ms），第 2000 轮 | 扫描：11 个 checkpoint 压力条件全部 0/20 摔；2000 轮 0.2 m/s 走 0.97 m、偏航 +4~+8°（后期 checkpoint 0.2 m/s 原地不动）。延迟对比（`training/data/sim-hl2915/delay-sweep-20261010/`）：50 / 60 ms 只摔 1/8、2/8（v4bl_lowfric 4/8、8/8，v2bl 3/8、7/8）；待实机 |
| `hl2915_sitstand_v4bl_3999.onnx` | v4 坐站（v2bl 配方 + 随机间隙） | BAM 仿真（间隙 0.35°、低通）5/5 坐下并站起，倾斜 ≤17°（v3bl 2/5）；待实机 |
| `hl2915_roulade_v4bl_6999.onnx` | v4 翻滚 | 5/5（与 v3bl 相同）；待实机 |

### v6（2026-10-11）

| 文件 | 来源 | 状态 |
|---|---|---|
| `hl2915_walk_v6lp_3250.onnx` | v6：v4lf 配方加四项改动：IMU 观测延迟 0–40 ms、腿刚度随机化 0.6–1.2、每回合动作增益 0.75–1.05、训练带 robotd 低通（`Velocity-Flat-Backlash-LowFriction-V6LP`）；第 3250 轮 | 对齐实机的仿真（`training/data/sim-hl2915/v6-sweep-20261011/`）里综合最好：幅度 1.0 时 0 摔，俯仰角速度 RMS 21 °/s，颈部摆动 2.7°；IMU 延迟 30 ms 也不摔（v4lf 幅度 1.0 摔 5/6）；待实机 |
| `hl2915_walk_v6lp_3999.onnx` | 同上，第 3999 轮 | 30 ms 下幅度 0.8 时最好（RMS 21，8 s 偏航 42°，前进 0.81 m）；待实机 |
| `hl2915_walk_v6lp_slip_3999.onnx` | v6lp 配方，地面摩擦 0.15–1.3（`Velocity-Flat-Backlash-Slippery-V6LP`）；第 3999 轮 | **10-11 晚实机硬地 + 硅胶脚垫最好**：幅度 0.9，偏航角速度 p95 93°/s（v6lp 125–133），晃动 RMS 32，单向慢漂 6 s −32°；加方向保持 v2 后结束偏航 +8°。配置 `training/data/2026-10-12-feet/today-slip3999-09-st08-hold.toml` |

v6 部署：低通 0.5 / 0.7（训练已含），`action_scale = 1.0`（先试），再试 0.9；其余同 G 组配置（中值关、校正 3.3°、`walk_settle_s = 2.0`）。

