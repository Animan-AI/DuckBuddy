# 打印件 / Printed parts

## 按舵机型号下载 / Download by servo model

| 舵机 / Servo | 打印工程 / Bambu Studio project |
| --- | --- |
| HL-2915（3S） | [DuckBuddy_2915.3mf](DuckBuddy_2915.3mf) |
| HD-1910（2S） | [DuckBuddy_1910.3mf](DuckBuddy_1910.3mf) |

以下为旧版 `DuckBuddy.3mf` 的说明和历史记录。  
The notes below describe the earlier `DuckBuddy.3mf` project.

`DuckBuddy.3mf` 是整机打印件的拓竹 Bambu Studio 工程，共 6 个打印盘：外观件、头部、轮子、TPU 软件、结构件（含加了散热孔和麦克风孔的头部上壳），以及一根 BAM 摆臂。零件来自 Microduck 原版的打印件，有以下修改：

- **头部前脸（face_part）**：改了安装位，用来装 **8×8 ToF 模块**（VL53L5CX 类）和 **IMX219 摄像头模组**；
- **头壳（top_head_shell）**：加了**散热孔**和**两个麦克风孔**（左右各一，对应 HAT 的 J7/J8，用来听声辨向）；
- **BAM 摆臂**（第 6 盘，新增）：120×20×3 mm 的平板，一端两个孔装在舵机输出盘上，做舵机参数辨识（[BAM](https://github.com/Rhoban/bam)）时用来摆动；只装机器人不用打；
- 其余零件（腿、髋、脚、躯干壳、电池托盘等）沿用原版，HL-2915 舵机直接装（外形、孔位、输出轴和 XL330 一样，M2×6 自攻螺丝）。

**打印参数**（工程里已经设好）：

| 项 | 值 |
|---|---|
| 打印机 | Bambu Lab P1S，0.4 mm 喷嘴（其它机型自己导入零件即可） |
| 材料 | PETG；软的零件（脚底、软嘴等）用 TPU |
| 层高 | 0.2 mm |
| 墙 / 填充 | 2 圈墙，30% 填充 |
| 支撑 | 开 |

用其它切片软件的话，3MF 也能直接打开（OrcaSlicer、PrusaSlicer 等）。

许可证：零件源自 Microduck（Apache-2.0），修改部分同样按 Apache-2.0 发布。

更新记录：
- 2026-10-25 工程改名为 `DuckBuddy.3mf`，打印盘整理成 6 个（原来是 `microduck2.3mf`，9 个盘）。
- 2026-10-30 去掉了没改过的旧头部上壳，改过的头部上壳挪到第 5 盘（结构件）；第 6 盘换成 BAM 摆臂。

---

`DuckBuddy.3mf` is a Bambu Studio project of all the printed parts, in 6 plates (shell parts, head, wheels,
TPU soft parts, structure with the head's top shell, and a BAM arm): Microduck's parts with the
head's face plate reworked to mount an 8×8 ToF module and an IMX219 camera, and the head shell given
cooling vents and two microphone holes (left and right, for the sound direction). PETG, TPU for the soft
parts; 0.2 mm layers, 2 walls, 30% infill, supports on; printed on a Bambu Lab P1S. Apache-2.0, as Microduck.
Plate 6 is a 120×20×3 mm arm for identifying the servos with BAM (not needed for the robot itself).
2026-10-25: renamed from `microduck2.3mf` (9 plates) and tidied into 6 plates.
2026-10-30: the unmodified old head top shell removed, the reworked one moved to plate 5; plate 6 is now the BAM arm.
