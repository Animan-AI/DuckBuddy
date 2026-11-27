# DuckBuddy r14 镜像与 HD-1910 策略

r14 适用于 Orange Pi Zero 3W（Debian Trixie），默认 HD-1910（普通 2S LiPo，满充 8.4 V）。HL-2915（3S）在工作台「总览 → 整机机型」切换后再标定。完成关节映射、零位标定和 IMU 核对后，在工作台启用整机服务。

刷机可直接从 [r14 发布页](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r14) 下载 `.7z` 和 `.img.sha256`。解压后校验 `.img` 再烧录。首次联网和标定见[新手教程](https://animan-ai.github.io/DuckBuddy/tutorial.html#image-r14)。

## HD-1910 运动配置

| 动作 | DuckBuddy 自训权重 |
|---|---|
| 行走 | [8498](../../../training/policies/hd1910_walk_kissqy_m6_p5_8498.onnx) |
| 起身／站立 | [9997](../../../training/policies/hd1910_stand_noise10_9997.onnx) |
| 坐站 | [6998](../../../training/policies/hd1910_sitstand_kp16_6998.onnx) |

[实测 PD 配置](hd1910-pd.toml)记录了小黑使用的寄存器值。双腿多数关节 P8，双膝和颈俯仰 P10，头俯仰 P8，头偏航、横滚和嘴部 P5，各轴 D0。`robotd` 启动时写入这些 RAM 增益。电流上限 1.3 A，电流寄存器比例 0.65 mA/格，UART7 使用 500 kbit/s，控制频率 50 Hz。

行走 8498 从 DuckBuddy 的 LinMid 7499 在[陆吾发布的 BAM M6 参数](../../../training/bam/params/hd1910/README.md)上续训。r14 上机使用自己的三份权重。[MuJoCo 机身模型](../../../hardware/rev-a/mjcf-hd1910/README.md)用于电脑上的仿真。

## 网页后台

联网后访问 `http://鸭子的IP:8088/`。驾驶舱可查看和操作已加载的动作模型。摄像头页可预览、拍照并下载 JPEG。「对话与设置」包含语音与模型供应商设置。驾驶舱与相机后台已有用户实机测试通过。r14 整卡首启和行走仍按预览版记录。

对应源文件在 [console/](console/)，热点配网程序和语音提示在 [hotspot/](hotspot/)。

## 从公开材料重建 r14

在 Linux 或 WSL 2 里执行，需要 root 挂载权限、Python 3.11+、`losetup`、`sfdisk`、`e2fsck`、`systemctl` 和约两份 IMG 的可用空间。源码与下载文件可以放在不同目录。

1. 下载 [r8 发布镜像](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r8)并解压。
2. 从 [r14 发布页](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r14)下载 `robotd-r14-aarch64`。这是 r14 里实测的 HD-1910 控制程序。
3. 在仓库根目录运行，将示例替换成实际路径。

    ```bash
    sudo bash deploy/opi-image/r14/build-r14.sh \
      /path/to/DuckBuddy-opizero3w-trixie-20261120-r8.img \
      /path/to/robotd-r14-aarch64 \
      /path/to/DuckBuddy-r14-rebuilt.img
    ```

脚本校验 r8 镜像、控制程序和三份 ONNX 的 SHA-256。输出新的 `.img` 和 `.img.sha256`，重建文件的校验值随文件系统时间戳变化。默认机型、配套模型、运行程序和网页功能与 r14 一致。

HD-1910 控制源码在 [Feetech 驱动](../../../duck-control/src/feetech.rs)、[参数](../../../robotd-params/src/lib.rs)和 [robotd](../../../robotd/src/main.rs)。有 Rust、Zig 和 `cargo-zigbuild` 时，可以用下面的命令自行编译控制程序。复现脚本使用上面固定 SHA-256 的发行二进制。

```bash
rustup target add aarch64-unknown-linux-gnu
cargo board --locked -p robotd --bin robotd
```
