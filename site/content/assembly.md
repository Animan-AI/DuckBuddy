---
title: 组装说明
lead: 两块扩展板怎么装：躯干里的 PDB/IMU 板，头里的 HAT + Orange Pi。
---

# 组装说明：电路板部分

这一页讲两块电路板的安装：躯干里的 **PDB/IMU 板**，和头里叠在一起的 **HAT + Orange Pi Zero 3W**。舵机、轴承和外壳的装法和 Microduck 原版一样，按打印件装就行。要用的零件见[物料清单](bom.html)。

照片里是一只 HD-1910（2S）版的鸭子；HL-2915（3S）版的板子装法相同，只是舵机线多了 3 根转接线（板子附带）。

<figure class="photo"><img src="img/assembly/parts.jpg" alt="拆开的机器鸭：躯干和腿、头里的电路板、头壳、电池" loading="lazy"><figcaption>全部零件摊开：左上是 2S 电池，中间是装好 HAT 和 Orange Pi 的脸部，下面是装好 PDB 的躯干和两条腿，右边是头壳。</figcaption></figure>

## 要准备的零件 {#parts}

| 零件 | 数量 | 用在哪 |
|---|---|---|
| 2.6 × 8 × 6 自攻螺丝（镀镍十字圆头带垫，螺纹 2.6 mm、长 8 mm、头 6 mm） | 2 | PDB 的两个大孔，配硅胶减震圈 |
| M1.8 × 8 自攻螺丝（十字圆头） | 2 | PDB 的两个小孔，背面垫白色 3D 打印垫圈 |
| 硅胶减震圈，M3 规格：内径 3 mm、外径 7 mm、厚 2 mm | 2 | 套在 PDB 的两个大孔里，减震 |
| 白色 3D 打印垫圈 / 垫柱 | 若干 | PDB 两个小孔背面的垫圈；头里撑住 Orange Pi 和 HAT 的垫柱 |
| HAT ↔ PDB 的 PH2.0 3P 同向线 | 1 | **要自己买**：两头都是 PH2.0 3P，同向（1 对 1） |
| PH1.25 ↔ AMP2.0 3P 同向转接线（HL-2915 版） | 3 | 板子附带 |

硅胶减震圈在淘宝搜“穿越机飞控减震圈 硅胶隔离垫片”，选 M3 的就是。

## PDB/IMU 板：装在躯干里 {#pdb}

PDB 板装在躯干顶上，压在左右两只髋部舵机的上面。电池从板子下面的电池托盘引出，通过 XT30 插头接到板子上。

1. **先把舵机线接好再装板子。** 板子装上后，下面的插座就不好够到了。左腿、右腿各一路舵机线接 PDB 两侧的插座，头部那一路接 HAT 的线也从这块板子出去。
2. **4 个安装孔对准下面的孔位。** 板子上有两个大孔、两个小孔：
    - **两个大孔**：先套上硅胶减震圈，再拧 2.6 × 8 × 6 的自攻螺丝。
    - **两个小孔**：板子背面垫一个白色的 3D 打印垫圈，再拧 M1.8 × 8 的自攻螺丝。
3. **螺丝拧到贴紧就停。** 硅胶圈要留一点弹性才能减震，拧太紧会压死它，还可能把打印件的孔拧滑。

<figure class="photo"><img src="img/assembly/pdb-top.jpg" alt="PDB 板装在两只舵机上方，四角用螺丝和硅胶减震圈固定" loading="lazy"><figcaption>装好的 PDB：左右两边能看到螺丝和下面的硅胶减震圈，中间白色的是电源和舵机总线插座。</figcaption></figure>

<figure class="photo"><img src="img/assembly/pdb-side.jpg" alt="从侧面看 PDB 板：板子下面用垫圈垫高，旁边是电池" loading="lazy"><figcaption>侧面：板子和下面的舵机之间用垫圈隔开，右边是电池和它的 XT30 插头。</figcaption></figure>

<div class="photos">
<figure class="photo"><img src="img/assembly/pdb-battery.jpg" alt="躯干背面：电池托盘用两颗螺丝固定，下面是四只舵机的插座" loading="lazy"><figcaption>躯干背面：电池托盘用两颗螺丝固定，下面四只舵机的线从这里走。</figcaption></figure>
</div>

PDB 上还有 IMU，它的朝向决定姿态解算对不对：板子按照片的方向装，**别装反、别转 90°**。IMU 固件可以用工作台的「IMU 固件」页烧（见[新手教程](tutorial.html#step4)）。

## HAT + Orange Pi：装在头里 {#hat}

HAT 叠在 Orange Pi Zero 3W 上，两块板一起装在脸部（face_part）里面，摄像头和 ToF 也装在脸上。

**先认 HAT 上的插座**（编号对应下面照片里的标记，丝印都印在板子上）：

<figure class="photo annot"><div class="annot-img"><img src="img/assembly/hat-top.jpg" alt="HAT 正面，插座上标了编号：1 风扇、2 ToF、3 左麦克风、4 右麦克风、5 喇叭、6 接 PDB" loading="lazy"><span class="mark" style="left:35.4%;top:22.2%">1</span><span class="mark" style="left:30.4%;top:40.4%">2</span><span class="mark" style="left:37.7%;top:79.7%">3</span><span class="mark" style="left:50%;top:79.7%">4</span><span class="mark" style="left:59.9%;top:59.4%">5</span><span class="mark" style="left:92.5%;top:45.7%">6</span></div><figcaption>HAT 正面：左上角是风扇和 ToF，下面一排是左、右麦克风和喇叭，右边白色 3P 是接 PDB 的 J2。</figcaption></figure>

| 编号 | 位置 / 丝印 | 接什么 |
|---|---|---|
| <span class="num">1</span> | 左上角的 2P 小插座 | 散热小风扇 |
| <span class="num">2</span> | 左上，丝印 TOF（4P） | 8 × 8 ToF 模块（3V3 / GND / SCL / SDA） |
| <span class="num">3</span> | 下排左边，J7 MIC | 左麦克风 |
| <span class="num">4</span> | 下排中间，J8 MIC P/N | 右麦克风 |
| <span class="num">5</span> | 下排右边，J6 SPK P/N | 喇叭 |
| <span class="num">6</span> | 右边白色 3P，J2 HEAD/PDB（1 G · 2 V · 3 D） | 接 PDB 的那根 PH2.0 3P 同向线：GND、电池、总线数据。**GND 和 DATA 别接反**，接反了每只舵机都会回 `ff ff 00` |
| — | 5V PWR（J6 右边） | 指示灯：HAT 的 5 V 有电时亮 |

<figure class="photo"><img src="img/assembly/hat-stack.jpg" alt="HAT 叠在 Orange Pi Zero 3W 上，麦克风、喇叭和 ToF 线都已插好" loading="lazy"><figcaption>HAT 叠在 Orange Pi 上，麦克风、喇叭和 ToF 的线都插好了。麦克风线用双绞线，能少一点干扰。</figcaption></figure>

**散热小风扇。** Orange Pi 旁边贴一个小风扇帮它散热，用热熔胶固定在板子侧面就行，线插 HAT 左上角的风扇座（上面照片的 1 号）。

<div class="photos">
<figure class="photo"><img src="img/assembly/hat-fan.jpg" alt="小风扇贴在 Orange Pi 侧面" loading="lazy"><figcaption>风扇贴在 Orange Pi 侧面。</figcaption></figure>
<figure class="photo"><img src="img/assembly/fan.jpg" alt="小风扇背面，四角有热熔胶" loading="lazy"><figcaption>风扇四角点热熔胶。</figcaption></figure>
</div>

**装进头里。** 两块板用白色的 3D 打印垫柱撑在脸部里面，排线朝下走：摄像头的 FPC 排线、麦克风和喇叭线、去 PDB 的 3P 线，都从板子下面绕出去。整理线的时候留一点余量，头要能左右转、上下点，线不能被扯紧。

<div class="photos">
<figure class="photo"><img src="img/assembly/head-inside.jpg" alt="头里：Orange Pi 和 HAT 用白色垫柱固定在脸部，下面是喇叭和摄像头排线" loading="lazy"><figcaption>装进头里：上面是两块板，中间是喇叭，橙色的是摄像头排线，左边红黄黑三根是去 PDB 的线。</figcaption></figure>
<figure class="photo"><img src="img/assembly/head-standoffs.jpg" alt="白色 3D 打印垫柱撑住 Orange Pi 和 HAT 的特写" loading="lazy"><figcaption>白色的 3D 打印垫柱特写。mini HDMI 口留在侧面，调试时还能插线。</figcaption></figure>
</div>

## 装完之后

1. 通电前再看一遍：J2 那根线的 GND / DATA 没接反，电池节数和舵机型号对得上（HL-2915 用 3S，HD-1910 用 2S）。
2. 接上电池，HAT 上的 **5V PWR** 灯应该亮，Orange Pi 开始启动。
3. 接着照[新手教程](tutorial.html)烧卡、开机，用工作台的「整机测试」把舵机总线、IMU、麦克风、喇叭、ToF、摄像头都测一遍。
