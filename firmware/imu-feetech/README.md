# imu-feetech — PDB IMU 板固件

PDB R2f 上的 STM32G031F8P6 + LSM6DSV16X。把芯片自带的姿态融合（SFLP 游戏旋转向量）
伪装成舵机总线上的一个飞特设备（ID 200，500 kbit/s），robotd 每个周期在同一次
`SYNC_READ` 里连同 15 个舵机一起读走。

- 协议与寄存器布局：`hardware/rev-a/electronics/imu_to_dxl-spec-feetech.md`
- 主机侧：`duck-control/src/feetech.rs`（每拍读 56 起 15 字节）

## 地址 56 起的 15 字节

| 偏移 | 内容 |
|---|---|
| 0..6 | 陀螺仪 x/y/z，i16 LE 原始计数，±500 dps |
| 6..12 | SFLP 四元数 x/y/z，binary16 LE，FIFO 原样；首个融合输出前全 0（主机"未就绪"约定） |
| 12 | 采样计数，每个融合输出 +1 |
| 13 | 标志：bit0 传感器已配置，bit1 姿态有效 |
| 14 | 保留 0 |

另可 `READ`：地址 5 = 200（ID），6 = 1（500k），3..4 = 0x16D6（型号），0..1 = 固件版本。

## 构建与烧录

```bash
rustup target add thumbv6m-none-eabi
cargo build --release
cargo install probe-rs-tools --locked     # 一次
probe-rs run --chip STM32G031F8Px target/thumbv6m-none-eabi/release/imu-feetech
```

ST-Link 接 PDB 的 J2：1 SWDIO、2 SWCLK、3 GND、4 3V3（板子另有电源时 ST-Link 的 3V3 可以不接）。

协议逻辑可在电脑上测试，不需要硬件：

```bash
rustc --edition 2021 --test src/proto.rs -o target/proto-test && target/proto-test
```

## 上机后要核对的

- 安装方向：主机默认 `DEFAULT_MOUNT` 假设芯片绕 Y 轴 +90° 安装（规格书 §4）。PDB 上 U2 在背面，
  实际方向要在机器人上摆姿态核对，不对就改主机侧安装四元数。
- robotd 配置 `[bus] imu = true` 才会把 ID 200 放进同步读取。
