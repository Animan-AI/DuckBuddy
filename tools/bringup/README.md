# bringup —— 装机 / 维修工具（网页）

装好或修好一台 DuckBuddy（MicroDuck）后用的工具，网页里有四个页签：

1. **舵机**：
   - **新舵机批量配置向导**（接飞特 USB 调试板）：一只一只接上去，自动完成改 ID、限位 0/4095、模式 0、卸载条件（HD-1910 写 5，HL-2915 写 13）、改成 500k 并上锁、读回核对。第一只会让你断电重上电，验证 EEPROM 真的存住了；每只最后回中到 2048 并保持扭矩，装好舵盘再松。支持串成链（ID 倒着分，做法同 `tools/servo/setup_ids.py`）和每只单独接两种接法，中断后重新开始不会撞号。
   - **关节映射**（装好以后，机器人总线或 USB 调试板）：逐只摆动 ±8°，看哪个关节动了就点哪个关节；点完再“左右对照”摆一遍核对。然后二选一：按选定顺序（关节顺序 1–15、同小白、配置里的、自定义）批量改 ID，先全部挪到临时号 160 + 新 ID 再落位，互换也行；或者不改 ID，把 `joint_ids` 写进机器人配置（先备份）。
   - 单只操作：扫描总线，改 ID / 改波特率，回中，读写关键寄存器，摆动测试。
   - **当前位置设为中位**：写 EEPROM 31（位置校正，BIT15 符号位），把舵机现在的位置记成 2048。0/4095 的跳变点和目标位置会一起平移，所以舵盘装偏了、或关节零位的读数靠近跳变点时，不用拆舵盘。10-31 在 HL-2915（固件 3.45）上台架实测通过：断电保存、跨原跳变点读数连续、回中和摆动都按新坐标到位；STS 的“写 40=128”在它上面没反应。HD-1910 还没测过。
2. **零位标定**：装配后标出 `[bus] joint_signs` / `joint_offsets`。直接嵌入 `tools/joint_align`（同一份代码、同一个页面）。
3. **IMU 固件**：在电脑上构建 `firmware/imu-feetech`，用 ST-Link 经 PDB J2 烧录，再经机器人总线读 ID 200 核对。
4. **整机测试**：舵机总线（每个 ID 都应答、SYNC_READ 错误率）、IMU、麦克风、喇叭、ToF、摄像头。每项给出通过 / 不通过和测量值，可以导出 JSON 报告。

## 启动

```bash
python tools/bringup/app.py --host orangepi@duck-1a2b.local              # 系统镜像起的主机名 duck-XXXX
python tools/bringup/app.py --host orangepi@192.168.1.50 --com COM11      # 用 IP，同时用飞特 USB 调试板
```

浏览器打开 <http://localhost:8780>，零位标定用 8781。每个页签顶部有一段可折叠的图文教程，写了操作步骤、HD-1910 / HL-2915 各自的寄存器取值，以及不通过时该查哪里。加 `--listen 0.0.0.0` 后，局域网里的手机也能打开。

前提：

- 机器人要能免密 ssh；机器人上有 `python3` 和 `python3-serial`，不需要另装东西。
- 电脑上有 `pyserial`。零位页还需要 `mujoco`、`numpy`、`Pillow`（和 joint_align 一样）。IMU 页还需要 `cargo` 和 `probe-rs`（`cargo install probe-rs-tools --locked`）。

其他参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--config` | `~/robot/active.toml` | robotd 的配置文件，零位标定写回这个文件 |
| `--unit` | `robotd-dev` | 停止 / 启动 robotd 用的 `systemctl --user` 单元 |
| `--restart-cmd` | `systemctl --user restart robotd-dev` | 写入零位后执行，让 robotd 重读配置 |
| `--init-cmd` | `robotctl robot init` | 零位页的“写入并回 home”在重启后执行 |

## 结构

```
浏览器 ⇄ app.py（电脑） ─ ssh ─ agent.py（机器人：/dev/ttyS7 500k，以及音频 / ToF / 摄像头）
                       ├─ agent.py --usb（电脑：飞特 USB 调试板，COMx）
                       ├─ tools/joint_align（零位标定，端口 +1）
                       └─ cargo / probe-rs（IMU 固件）
```

- `agent.py` 每次连接时经 ssh 的 stdin 送到机器人上运行，所以机器人上没有要部署的文件。它一行收一个 JSON 请求、回一行 JSON 应答，也可以在机器人上手动运行：`python3 agent.py`，然后在 stdin 里敲请求。
- 出厂舵机是 1M。OPi 的 UART7 设成 1M 时实际只有 750k，所以新舵机改 ID、改波特率要接飞特 USB 调试板（CH343，总览页里选串口）。FD 上位机开着时会独占这个串口。

## 安全规则（工具里已经做了的）

- **robotd 占着总线。** 它在跑时，舵机页、总线测试和 IMU 测试一律拒绝执行。总览页里可以停掉它：如果它是 `robotd-dev` 单元起的，就用 systemctl 停；否则只按页面上显示的那个 PID 停。**停 robotd 不会卸力。**
- 每个会让舵机动或上扭矩的操作（摆动、回中、上扭矩、写 40/41/42/44/46）都要先在网页上确认“托好了”。
- 写目标时 42..47 一起写（目标位置 + 44 + 46）。原因是 HD-1910 不重写 46 会出现零电流卡住。
- 舵机读数超出 0..4095（多圈计数）时拒绝上扭矩。多圈计数只有断舵机电才能清掉。
- 摆动测试：以当前位置为中心，摆 ±摆幅后回到原位，然后松扭矩。会跨过 0/4095 的那一侧跳过。限流默认 400 mA；电流持续超过 1.5 A，或位置偏离 15°，就立即切扭矩。
- 回中：转到同一圈里最近的 2048 并保持扭矩，用来装舵盘，只适用于还没装进关节的舵机。如果越走越远或过流，就切扭矩。
- 页头的“全部卸力”：robotd 在跑时发 `robot.relax`，否则在总线上广播“扭矩开关=0”。
- 写 EEPROM 时先解锁（55=0），写完再上锁。改 ID 前会先确认新 ID 没被占用；IMU 的 200 不能用。

## 整机测试的判据

| 项目 | 做法 | 通过条件 |
|---|---|---|
| 舵机总线 | 配置里的 `joint_ids` 逐个 PING，再按 robotd 的方式做 N 轮 SYNC_READ（地址 56 起 15 字节，IMU 排在最后） | 全部应答；错误率 ≤ 0.5%；状态位全 0；没有多圈读数 |
| IMU | 读 ID 200 的版本、型号、56..70 块和 90 起的诊断计数，持续 2 秒 | 型号 0x16D6；标志 = 3；融合输出 90–150 Hz；\|q\| ≈ 1；静止时陀螺噪声 < 1 dps、零偏 < 5 dps；板上没有接收错误。俯仰 / 横滚按 `[bus] imu_axes` 换算到躯干系 |
| 麦克风 | 一边放静音一边录 4 秒（先放后录，否则录音会整段顶满），录音期间拍手或说话 | 两路都不是全 0；直流偏移 < 3000；削顶 < 2%；峰值 ≥ 4 倍底噪 |
| 喇叭 | 放 1 kHz（−12 dBFS）并同时录音，比较有声窗口和安静窗口里 1 kHz 的能量；再问人耳听没听到 | 高出 ≥ 15 dB，并且人耳听到 |
| ToF | 读 tofd 的 `tof.stream`；tofd 没在跑时，临时起一个（`--bus` 取自 tofd-dev.service，默认 /dev/i2c-2），测完只停这一个 | ≥ 10 Hz；有效区 ≥ 32/64 |
| 摄像头 | 找名字是 `vin_video*` 的节点，用 `duckcam --half` 抓 45 帧（ISP 自动曝光） | ≥ 10 fps；亮度不是全黑或全白；画面有纹理。附缩略图 |

- 摄像头测试从不调用 `v4l2-ctl --list-ctrls`，那条命令会把板子卡死。vin 驱动没加载时，要先运行 `sudo modprobe -a vin_io imx219 vin_v4l2`。
- visiond 在跑时摄像头被它占着，摄像头测试会报占用。

## 蓝牙手柄配对

`-r3` 镜像已安装配对服务。尚未配对时，开机后让 Xbox 模式的 BR/EDR 手柄进入快闪状态，鸭子会在约 90 秒的扫描窗口内尝试配对。配对后，开机按手柄 Home 键回连。换手柄时，在工作台「总览」点「配对新手柄」。工作台通过 SSH 发送请求，首次开机自动配对由板端服务独立完成。

已有的 Orange Pi Debian 系统可以保留。在主控上以 `orangepi` 登录，进入 DuckBuddy 仓库目录，安装板端服务：

```bash
sudo apt install bluez python3-dbus python3-gi
sudo install -m 755 deploy/opi-image/duck-gamepad-pair.py /usr/local/sbin/duck-gamepad-pair
sudo install -m 644 deploy/opi-image/duck-gamepad-pair.service /etc/systemd/system/
sudo systemctl enable --now duck-gamepad-pair.service
```

服务启动时会立即检查已有配对记录。旧系统仍需具备可用的蓝牙适配器及手柄所需的内核驱动（如 `uhid`、`hidp`）；服务只负责配对。工作台的舵机、标定和体检功能仍需要主控具备各自的驱动、配置或服务。
