---
title: 组装说明
lead: 两块扩展板怎么装：躯干里的 PDB/IMU 板，头里的 HAT + Orange Pi。
---

# 组装说明：电路板部分

这一页讲怎么装躯干里的 **PDB/IMU 板**和头里的 **HAT + Orange Pi Zero 3W**。如果两块板已经装好，可以直接看[两块板怎么连线](#pdb-hat-wiring)。要用的零件见[物料清单](bom.html)。

照片里是一只 HD-1910（2S）版的鸭子；HL-2915（3S）版的板子装法相同，只是舵机线多了 3 根转接线（板子附带）。

<figure class="photo annot">
  <div class="annot-img">
    <img src="img/assembly/parts.jpg" alt="拆开的机器鸭。上方脸部是 HAT，左下躯干是 PDB，两块板由一根三芯线连接。" loading="lazy">
    <span class="mark port-label" style="left:36%;top:12%">头部 HAT</span>
    <span class="mark port-label" style="left:34%;top:61%">躯干 PDB</span>
  </div>
  <figcaption>装机照片用于找板子。躯干的 PDB 用 <b>J5（HAT）</b> 接头部的 HAT <b>J2（HEAD/PDB）</b>。选线看下方的接线图。</figcaption>
</figure>

## 要准备的零件 {#parts}

| 零件 | 数量 | 用在哪 |
|---|---|---|
| 2.6 × 8 × 6 自攻螺丝（镀镍十字圆头带垫，螺纹 2.6 mm、长 8 mm、头 6 mm） | 2 | PDB 的两个大孔，配硅胶减震圈 |
| M1.8 × 8 自攻螺丝（十字圆头） | 2 | PDB 的两个小孔，背面垫白色 3D 打印垫圈 |
| 硅胶减震圈，M3 规格：内径 3 mm、外径 7 mm、厚 2 mm | 2 | 套在 PDB 的两个大孔里，减震 |
| 白色 3D 打印垫圈 / 垫柱 | 若干 | PDB 两个小孔背面的垫圈；头里撑住 Orange Pi 和 HAT 的垫柱 |
| HAT ↔ PDB 三芯连接线 | 1 | **要自己买**。选 **PH2.0 3P 双头反向线，长 300 mm**。看[接线图](#pdb-hat-wiring)核对线序。 |
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

HAT 叠在 Orange Pi Zero 3W 上，两块板一起装在脸部（face_part）里面。合上头壳之前，按[头部传感器接线](#camera-tof)接好摄像头和 ToF。

**先认 HAT 上的插座**（编号对应下面照片里的标记，丝印都印在板子上）：

<figure class="photo annot"><div class="annot-img"><img src="img/assembly/hat-top.jpg" alt="HAT 正面，标出了风扇、ToF、两个麦克风、喇叭，以及右侧接 PDB 的 J2。" loading="lazy"><span class="mark" style="left:35.4%;top:22.2%">1</span><span class="mark" style="left:30.4%;top:40.4%">2</span><span class="mark" style="left:37.7%;top:79.7%">3</span><span class="mark" style="left:50%;top:79.7%">4</span><span class="mark" style="left:59.9%;top:59.4%">5</span><span class="mark port-label" style="left:90.7%;top:45.7%">6 · J2</span></div><figcaption>HAT 正面。右侧白色三针座是 <b>J2 HEAD/PDB</b>，线的另一端插躯干里的 PDB J5。照片用于找接口，插头规格和线序按配套版本核对。</figcaption></figure>

| 编号 | 位置 / 丝印 | 接什么 |
|---|---|---|
| <span class="num">1</span> | 左上角的 2P 小插座 | 散热小风扇 |
| <span class="num">2</span> | 左上，丝印 TOF（4P） | 8 × 8 ToF 模块（3V3 / GND / SCL / SDA） |
| <span class="num">3</span> | 下排左边，J7 MIC | 左麦克风 |
| <span class="num">4</span> | 下排中间，J8 MIC P/N | 右麦克风 |
| <span class="num">5</span> | 下排右边，J6 SPK P/N | 喇叭 |
| <span class="num">6</span> | 右边白色 3P，J2 HEAD/PDB | 接 PDB 的 J5，传电池电压和舵机总线数据。按[接线图](#pdb-hat-wiring)核对 PH2.0 反向线。 |
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

## 摄像头排线与 ToF 接线 {#camera-tof}

<figure class="photo annot">
  <div class="annot-img">
    <a href="img/assembly/camera-tof-wiring.jpg"><img src="img/assembly/camera-tof-wiring.jpg" alt="头部传感器接线实拍。左侧香橙派 CAM2 用橙色排线连中央的 IMX219；摄像头下方的蓝色 ToF 模块焊接四芯线，另一端插右侧 HAT 的 J5 TOF。" width="1200" height="674" loading="lazy"></a>
    <span class="mark" style="left:33%;top:34%" aria-hidden="true">1</span>
    <span class="mark" style="left:64%;top:38%" aria-hidden="true">2</span>
    <span class="mark" style="left:54%;top:72%" aria-hidden="true">3</span>
    <span class="mark" style="left:81.5%;top:55%" aria-hidden="true">4</span>
  </div>
  <figcaption>① 香橙派 CAM2 排线座 · ② IMX219 摄像头 · ③ ToF 模块焊点 · ④ HAT J5 TOF 插座。点击照片可放大。</figcaption>
</figure>

### 接摄像头排线

1. 拔掉电池和其他供电。用配套的 **22 针转 15 针 MIPI 摄像头排线**，22 针端插香橙派的 **CAM2**（①），15 针端插 **IMX219** 摄像头（②）。
2. 两端分别打开卡扣，让排线的金属触点朝向座内触点。把排线插平、插到底，再扣回卡扣。两侧插入深度要一致，走线时留弯，避免锐折或夹在头壳边缘。

**照片中的短排线是单独购买的。** 摄像头随附的长排线也可以直接使用，接法和功能相同，把多出的长度在头壳内留成宽弯即可。

### 焊接 ToF 的四根线

1. 按图准备一端有 **SH1.0 4P** 插头、另一端是散线的线束。插头端接 **HAT J5 TOF**（④），散线端逐根焊到 ToF 模块背面的对应焊盘（③）。
2. 按下表把同名信号接在一起。剥线只留够焊接的裸芯，给线头和焊盘预上锡，再逐根焊接。线色可辅助辨认线束，接线以丝印和万用表实测为准。

| HAT J5 TOF | ToF 模块焊盘 | 用途 |
|---|---|---|
| 1 · 3V3 | 3.3 V 供电脚 | 模块供电 |
| 2 · G / GND | GND | 地 |
| 3 · SCL | SCL | I²C 时钟 |
| 4 · SDA | SDA | I²C 数据 |

3. 通电前检查四根线逐根导通、相邻焊点没有连锡，并确认 3V3 和 GND 之间没有短路。焊点确认后固定线束，让拉力落在固定处，避开 ToF 传感器窗口。

**合壳前测试。** 接上电源后在工作台「整机测试」分别运行「摄像头」和「ToF」。摄像头应能看到正常缩略图，白纸在 ToF 前来回移动时，距离读数应随之变化。

## PDB 和 HAT 怎么连 {#pdb-hat-wiring}

**躯干 PDB 的 J5（HAT）接头部 HAT 的 J2（HEAD/PDB）。** PDB 上另一个叫 J2 的接口用于给 IMU 烧录。在有 J7 HEAD 的 PDB 上，J7 接头部舵机。

使用 **PH2.0 3P 双头反向线，长 300 mm**。购买时可搜「PH2.0 3P 双头反向线 300mm」。两个插头的卡扣朝同一方向观察时，线位应是 **① ↔ ③、② ↔ ②、③ ↔ ①**。 [JST PH 规格](https://www.jst.com/products/crimp-style-connectors-wire-to-board-type/ph-connector/)

<figure class="photo wiring-figure">
  <div class="wiring-map" role="img" aria-label="PH2.0 3P 反向线示意。PDB J5 的线位一 GND 连到 HAT J2 的线位三 GND。线位二电池正极连到线位二。线位三 DATA 连到线位一 DATA。">
    <strong class="wiring-title">PDB J5</strong>
    <strong class="wiring-title">HAT J2</strong>
    <div class="wiring-pins" aria-hidden="true">
      <span class="wire-ground">① GND</span>
      <span class="wire-power">② VBAT</span>
      <span class="wire-data">③ DATA</span>
    </div>
    <div class="wiring-cross" aria-hidden="true"></div>
    <div class="wiring-pins" aria-hidden="true">
      <span class="wire-data">① DATA</span>
      <span class="wire-power">② VBAT</span>
      <span class="wire-ground">③ GND</span>
    </div>
  </div>
  <figcaption><b>PH2.0 3P 双头反向线（300 mm）。</b> 左右两列是同一朝向看两个插头时的线位。圈号用来核对线束，板上的接点按 G/V/D 丝印和通断测量确认。VBAT 是电池正极，DATA 是舵机信号。</figcaption>
</figure>

**插电前，用万用表核对三根线。** 拔掉电池和线束两端，按图用蜂鸣档逐根验证。再按板上丝印确认插好后 **GND 到 GND、VBAT 到 VBAT、DATA 到 DATA**。

## 装完之后

1. 通电前再看一遍：[PDB J5 ↔ HAT J2 的三根线](#pdb-hat-wiring)已逐根核对，电池节数和舵机型号对得上（HL-2915 用 3S，HD-1910 用 2S）。
2. 接上电池，HAT 上的 **5V PWR** 灯应该亮，Orange Pi 开始启动。
3. 接着照[新手教程](tutorial.html)烧卡、开机，用工作台的「整机测试」把舵机总线、IMU、麦克风、喇叭、ToF、摄像头都测一遍。
