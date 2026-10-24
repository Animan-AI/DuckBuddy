# 打印件 / Printed parts

`DuckBuddy.3mf` 是整机打印件的拓竹 Bambu Studio 工程，共 6 个打印盘：外观件、头部、轮子、TPU 软件、结构件，以及加了散热孔和麦克风孔的头部上壳。零件来自 Microduck 原版的打印件，有以下修改：

- **头部前脸（face_part）**：改了安装位，用来装 **8×8 ToF 模块**（VL53L5CX 类）和 **IMX219 摄像头模组**；
- **头壳（top_head_shell）**：加了**散热孔**和**两个麦克风孔**（左右各一，对应 HAT 的 J7/J8，用来听声辨向）；
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

更新记录：2026-10-25 工程改名为 `DuckBuddy.3mf`，打印盘整理成 6 个（原来是 `microduck2.3mf`，9 个盘）。

---

`DuckBuddy.3mf` is a Bambu Studio project of all the printed parts, in 6 plates (shell parts, head, wheels,
TPU soft parts, structure, and the head's top shell with vents and microphone holes): Microduck's parts with the
head's face plate reworked to mount an 8×8 ToF module and an IMX219 camera, and the head shell given
cooling vents and two microphone holes (left and right, for the sound direction). PETG, TPU for the soft
parts; 0.2 mm layers, 2 walls, 30% infill, supports on; printed on a Bambu Lab P1S. Apache-2.0, as Microduck.
2026-10-25: renamed from `microduck2.3mf` (9 plates) and tidied into 6 plates.
