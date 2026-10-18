# 实机测试脚本（OPi 上用）

部署：`scp start.sh trial.sh thermal_run.sh settle_hold.py rpc.py orangepi@<ip>:robot/`，每次开机后
`cp ~/robot/rpc.py /tmp/rpc.py`（`/tmp` 断电清空）。录像存 `~/robot/recs`。

| 脚本 | 作用 |
|---|---|
| `start.sh <cfg.toml> <tag>` | 后台启动 robotd（`/tmp/robotd.sock`），日志 `recs/robotd-<tag>.log`；舵机不上电 |
| `rpc.py init / enable / disable / relax / health / watch S / record S PATH / move VX S` | robotd IPC |
| `trial.sh <tag> [vx] [秒]` | 一次行走试次：温度门限（CPU 80 °C / 舵机 65 °C）→ 等站立策略接管（最多 4 s）再站 1 s → 站稳检查（必须是站立策略在控制，`QUIET_MAX`，默认 1.5°/拍）→ 默认 0.2 m/s 走 4 s。手柄服务 padd-dev 要先停，它会持续发零速度 |
| `ping_all.py [N]` | 逐个 ping 15 个舵机和 IMU 板（ID 200），统计丢包，并读每个舵机的电压、温度和状态。**robotd 运行时不要跑**，两者会抢总线。IMU 板对单独读取应答慢，它的等待时间是 30 ms |
| `settle_hold.py` | 等站稳 1 s 再停策略（不在起身中途停） |
| `thermal_run.sh <tag> walk\|stand [秒] [vx] [vyaw]` | 升温测试：绕约 0.75 m 的圈走（或只站），最多走到给定时长（默认 180 s），录 `recs/thermal_<tag>.jsonl`；robotd 的 `[thermal]` 到 65 °C 会自己停走 |
| `thermal_stats.py`（在电脑上跑） | 读录像里的每只舵机温度 `temps_c`：起止温度、升温速率（°C/分钟）、最热的舵机几秒到 55 / 60 / 65 °C |

换策略要重启 robotd：先托住机器人（退出时卸力），`kill <精确 PID>`，再 `start.sh`；`init` 前把头扶起来
（颈离站姿 >100° 会被拒绝）。分析试次：`training/data/2026-10-09-hl2915-v4test/trial_stats.py`
（摔倒按机身倾角 >30° 判，不要只看 robotd 的 limp-fall 标签 —— 它会把行走指令清零）。

没装头部风扇时 OPi 会过热（2026-10-09：烫手，同时总线大量读写失败）；风扇到之前用台式风扇对着头吹。
