# Orange Pi Zero 3W A733 音频离线部署

本文用于 MicroDuck Orange Pi Zero 3W A733 HAT 上的
`TLV320AIC3104`。目标系统当前固定为：

- 板型：`orangepizero3w` / `sun60iw2`
- 系统：Debian Trixie server 模式
- 内核：`6.6.98-sun60iw2`
- HAT 总线：I2C2、I2S0、UART7
- codec 地址：I2C2 `0x18`
- codec 复位：PB0，低有效
- 功放使能：PB3，高有效

`scripts/setup-board.sh` 是 Radxa Zero 3W/RK3566 专用脚本，不得在
A733 上运行。它使用 `/boot/armbianEnv.txt`、`ttyS5` 和 `rk35xx`
内核，而 A733 使用 `/boot/orangepiEnv.txt`、`ttyS7` 和 `sun60iw2`
内核。脚本现在会识别 A733 并停止，防止误改。

## 1. 本机准备源码包

开发板不需要、也不应直接访问 GitHub。以下命令在 Windows 本机的
PowerShell 中运行：

```powershell
Set-Location <repo>

git clone --bare --branch kernel-sun60iw2-6.6 --single-branch `
  https://github.com/scallopmuscle/linux-kernel.git `
  tools/reference/linux-kernel-sun60iw2-6.6.git

git --git-dir=tools/reference/linux-kernel-sun60iw2-6.6.git `
  rev-parse e3ee05f508bd6db1026cf1e7012c2eeb128da410

git -c core.protectNTFS=false `
  --git-dir=tools/reference/linux-kernel-sun60iw2-6.6.git `
  archive --format=tar.gz `
  --prefix=linux-sun60iw2-6.6.98/ `
  --output=tools/cache/a733/linux-sun60iw2-6.6.98-e3ee05f.tar.gz `
  e3ee05f508bd6db1026cf1e7012c2eeb128da410

Get-FileHash -Algorithm SHA256 `
  tools/cache/a733/linux-sun60iw2-6.6.98-e3ee05f.tar.gz
```

本次从上述提交生成的归档为 `342785029` bytes，SHA256 为
`AB72B32F6B9A91FC3210EB2B6A53923235C549CE8E6915ACEA01CF6BB7B21D13`。
传输完成后应在板端用 `sha256sum` 再核对一次。

必须保留 `-c core.protectNTFS=false`。内核树包含名为 `aux.c` 的文件，
Git for Windows 默认会把它当作 Windows 保留设备名而拒绝归档。

源码来源为第三方 A733 内核快照，而不是 Orange Pi 官方仓库：

- 仓库：`scallopmuscle/linux-kernel`
- 分支：`kernel-sun60iw2-6.6`
- 提交：`e3ee05f508bd6db1026cf1e7012c2eeb128da410`
- 提交说明：`fix: restore missing arch/arm64/kernel/vmlinux.lds.S`

量产镜像若更换内核，必须重新取得完全匹配的源码并重编模块；不能把这里的
`.ko` 直接复制到不同 `uname -r` 的系统。

## 2. 从本机传到开发板

先在板端建立目录：

```powershell
ssh orangepi@<robot-ip> \
  "mkdir -p /home/orangepi/microduck-offline/kernel"
```

再从本机传输完整源码包和本仓库的音频目录。命令会交互询问板端密码：

```powershell
& 'C:\Program Files\PuTTY\pscp.exe' `
  tools/cache/a733/linux-sun60iw2-6.6.98-e3ee05f.tar.gz `
  orangepi@<robot-ip>:/home/orangepi/microduck-offline/kernel/

& 'C:\Program Files\PuTTY\pscp.exe' -r `
  deploy/audio `
  orangepi@<robot-ip>:/home/orangepi/microduck-offline/
```

首次连接显示的 SSH 主机指纹必须与本机已确认的开发板指纹核对，不要盲目接受。

## 3. 板端安装

Debian 软件源可用时先安装构建依赖；此步骤不访问 GitHub：

```bash
sudo apt update
sudo apt install -y \
  build-essential bc flex bison libssl-dev libelf-dev dwarves \
  device-tree-compiler i2c-tools alsa-utils
```

然后执行离线安装器：

```bash
cd /home/orangepi/microduck-offline
sha256sum kernel/linux-sun60iw2-6.6.98-e3ee05f.tar.gz
sudo sh audio/install-aic3104-a733.sh \
  kernel/linux-sun60iw2-6.6.98-e3ee05f.tar.gz
sudo reboot
```

安装器会完成以下操作：

1. 核对板型和当前内核版本；
2. 从本地归档准备内核树；
3. 编译并安装 AIC3X core 与 I2C 两个模块；
4. 编译并安装 `microduck-aic3104` DT overlay；
5. 在 `/boot/orangepiEnv.txt` 中追加 overlay；
6. 安装并启用 `aic3104-a733.service`。

## 4. 无 HAT 时验收

未接实体 HAT 时，以下状态才是正确的：

```bash
uname -r
modinfo snd-soc-tlv320aic3x-i2c | grep -E 'filename|vermagic'
lsmod | grep snd_soc_tlv320aic3x
systemctl status aic3104-a733.service --no-pager
cat /sys/class/leds/microduck-amp-enable/brightness
```

预期：

- 内核和 `vermagic` 都是 `6.6.98-sun60iw2`；
- 两个 `snd_soc_tlv320aic3x` 模块已加载；
- 服务为 `active (exited)`；
- 日志提示 `TLV320AIC3104 not detected on I2C2`；
- PB3 对应的功放使能值为 `0`。

即使没有物理 codec，ASoC 也可能注册名为 `aic3104` 的逻辑声卡。因此
`aplay -l` 单独出现该声卡不能证明 HAT 已连接。

## 5. 接入 HAT 后验收

断电安装 HAT 后重新上电，依次检查：

```bash
i2cdetect -y 2
journalctl -u aic3104-a733.service -b --no-pager
cat /sys/class/leds/microduck-amp-enable/brightness
aplay -l
arecord -l
```

预期 `0x18` 由内核驱动占用时显示为 `UU`。初始化日志不再出现
`not detected`，功放使能变为 `1`。最后必须使用实物完成：

- 左右声道播放与静音测试；
- 两个麦克风接口分别录音，确认 `MIC2`/`LINE1` 路由；
- 示波器检查 MCLK、BCLK、LRCK；
- 复位与冷启动测试；
- 喇叭工作时的 5 V、3.3 V、1.8 V 电源稳定性测试。

## 6. 已验证的非音频接口

当前样机已经验证以下系统节点：

- UART7：`/dev/ttyS7`
- I2C2：`/dev/i2c-2`
- USB 摄像头：`/dev/video0`，驱动 `uvcvideo`
- GPU render node：`/dev/dri/renderD128`
- EGL/OpenGL ES：PowerVR B-Series BXM-4-64，OpenGL ES 3.2
- Vulkan：PowerVR proprietary driver，Vulkan 1.3

这些结果证明驱动和 pinmux 已启用，但不代替 HAT 首件的电气测试。当前唯一已知的
失败 systemd 单元是 `smartmontools.service`，与 HAT、GPU、摄像头和声卡无关。

## 7. 尚未包含的整机软件迁移

本文完成的是 A733 的板级接口和 AIC3104 bring-up，不表示当前 Radxa 软件镜像可原样
部署到 Orange Pi。整机软件至少还有三项平台差异：

- `robotd` 的仓库默认值仍是 `/dev/ttyS5`，A733 样机必须改为 `/dev/ttyS7`；
- `mediad` 当前使用 Rockchip `mpph264enc`，A733 已有 `/dev/cedar_dev*` 内核节点，
  但 Allwinner 用户态硬件编码链路尚未验收；
- `.rknn` 检测模型属于 RK3566/RKNN，A733 应先使用 CPU ONNX，或另做 A733 NPU 适配。

因此，声卡实物验收后还需单独完成 A733 版应用配置和媒体性能验收。不要在 A733 上
运行 `scripts/setup-gstreamer.sh` 或 `scripts/setup-npu.sh`，两者目前都是 Rockchip 专用。
