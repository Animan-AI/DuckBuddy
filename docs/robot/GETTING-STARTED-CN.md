# DuckBuddy 新手教程

**已经装好并标定好一只鸭子？** 从下面的开机开始。日常用手机就能看状态，想带它走路再用手柄。

**刚拿到零件或一张空卡？** 从 [组装说明](https://animan-ai.github.io/DuckBuddy/assembly.html) 和 [烧录、接线与标定](https://animan-ai.github.io/DuckBuddy/tutorial.html#prepare) 开始。已经刷好了卡，但关节还没标定的，先完成标定再试走路。

日常使用选 [r8 系统镜像](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r8)。已经能正常使用的鸭子，可以按现有系统直接练习。

[开机](#power-on) · [充电](#charge) · [日常使用](#daily-use) · [让 AI Agent 做第一个小功能](#ai-development)

<a id="power-on"></a>

## 开机，打开豆豆的网页

1. **先看机型和电池标签。** HL-2915 用 **3S LiPo 或 3S LiHV**，HD-1910 用普通 **2S LiPo**。`2S`、`3S` 是串联的电芯节数，代表不同电压。请同时确认舵机、扩展板和电池是配套的。拿不准时，找卖家确认机型再接电池。
2. **在平地上接电。** 刚装好或换过板子，先按接线图核对 PDB 和 HAT 之间的三根线。再扶住躯干或把鸭子放稳，把电池插入机身的电池接口。插电后主控会开机。新刷的卡第一次启动要等几分钟。听到提示或看到主控启动后再继续。 [PDB 与 HAT 接线图](https://animan-ai.github.io/DuckBuddy/assembly.html#pdb-hat-wiring)
3. **让手机和鸭子连同一个 Wi‑Fi。** 如果鸭子已经连过这个网络，可以问「豆豆，你的 IP 是多少」。第一次连网或换了路由器，在手机的 Wi‑Fi 设置中点当前的 **2.4 GHz Wi‑Fi**，找「分享」或「二维码」。说「豆豆，扫码」，听到提示后把二维码举到摄像头前约 20–30 厘米。连上后记下它播报的 IP。配网失败时可用显示器和键盘，按[首次连网步骤](https://animan-ai.github.io/DuckBuddy/tutorial.html#step2)处理。
4. **在手机浏览器打开页面。** 地址栏输入 `http://鸭子的IP:8088/`，例如鸭子报 `192.168.1.50` 时，输入 `http://192.168.1.50:8088/`。IP 就是这只鸭子在当前网络里的地址，换网络后可能变。有时浏览器会自动改成 HTTPS，把开头改回 `http://` 再试。

看到网页上的鸭子名字和状态，就可以试着打个招呼。已经改过名字的，把下面的「豆豆」换成你起的名字。

<a id="charge"></a>

## 充电和关机

**网页顶部可以看电量。** HD-1910 的 2S 版低于约 30% 时先充电再试起身。连续走路时如果出现电量提示或供电重启，也先结束这轮测试。

### 先关机，再取电池

1. 松开摇杆。把鸭子扶稳，或让躯干稳稳支在软垫上，能承受关机过程中的卸力。
2. 已连接的手柄可以长按 **Select** 约 **2 秒**（有些手柄标着 **Back**）。会用电脑登录鸭子的，也可以在鸭子终端输入 `sudo poweroff`。登录方法见 [首次登录](https://animan-ai.github.io/DuckBuddy/tutorial.html#step2)。
3. 等关机动作结束、系统停止响应后再多等约 10 秒，再拔掉电池。HAT 的电源灯由电池供电，灯还亮时也可能已经关机。

### 用锂电平衡充电器充电

| 鸭子版本 | 电池标签 | 充电器模式 | 单节充满 | 整包充满 |
|---|---|---|---|---|
| HD-1910 | 普通 2S LiPo | LiPo Balance / 2S | 4.20 V | 8.40 V |
| HL-2915 | 普通 3S LiPo | LiPo Balance / 3S | 4.20 V | 12.60 V |
| HL-2915 | **3S LiHV，标有 4.35 V/节** | **LiHV Balance / 3S** | **4.35 V** | **13.05 V** |

充电前先看电池标签上的类型和节数，再按表选模式。普通 3S LiPo 的满充电压是 12.60 V，**13.05 V 只适用于明确额定 4.35 V/节的 3S LiHV**。HD-1910 按 2S LiPo 的 8.40 V 上限使用。 [充电器手册及电池电压表](https://www.skyrc.com/download/100189_T400Q_Manual.pdf)

1. **取下电池，等它冷却。** 鼓包、破损或异常发热的电池先停用。充电用与电池规格匹配的平衡充电器。香橙派上的 USB 接口不负责给电池充电。
2. **按充电器说明接线。** 粗线插头是主电源，细线的小插头是平衡头，用来逐节测量和均衡。根据上表核对充电器显示的模式和节数。充电电流按电池厂家的说明设置。
3. **放在耐热、不燃的平面上充电，人留在旁边。** 充电器提示完成后，按说明拆下电池。充电器报节数不符或电池异常时先停止，核对标签和接线。 [平衡充电器说明](https://www.skyrc.com/download/100189_T400Q_Manual.pdf)

<a id="daily-use"></a>

## 日常怎么用

### 聊天和看状态

- **叫名字。** 先说「豆豆，你好」或「豆豆，现在几点了」。网页会显示它听到的话、回应和电量。语音识别和部分本地回应可离线使用。
- **想开在线聊天。** 在鸭子的网页展开「大模型与声音」，选供应商，填自己从该平台取得的 **API Key**。它相当于访问模型服务的密码，直接填到鸭子的网页里。连着 Wi‑Fi 但显示「待填 API Key」时，先保存对应供应商的 Key，再问一句确认是否用了大模型。 [模型和声音设置](https://animan-ai.github.io/DuckBuddy/tutorial.html#online-model)
- **想切换声音。** 网页选「人声」或「鸭语」，也可以说「豆豆，切换到人声」或「豆豆，切换到鸭语」。r8 的默认在线人声是 Edge 晓晓。

### 用手柄走一小段

1. **先确认已经标定，并启用了整机服务。** 标定是让鸭子知道每个关节的正确零位，新的鸭子首次走路前需要做一次。 [启用整机服务](https://animan-ai.github.io/DuckBuddy/tutorial.html#step8)
2. **连接手柄。** 已配对时短按 **Home** 回连。用 BSP-G6 首次配对时，把模式拨到 **X**，长按 **Home** 约 3 秒，等灯快闪后在工作台点「配对新手柄」。 [手柄按键图和配对步骤](https://animan-ai.github.io/DuckBuddy/tutorial.html#gamepad-bsp-g6)
3. **托稳后按 Start，站稳后放在平整、不滑的地面。** 轻推**左摇杆**前进，松开摇杆停住。**右摇杆**用来转向。第一次用时在旁边扶护。
4. **需要卸力时，先稳住身体。** 舵机卸力后关节会变软，所以扶稳或垫稳后，再用工作台的「全部卸力」。HL-2915 版可先说「豆豆，坐下」。HD-1910 版停下后仍可能站着，扶稳后再卸力。

**想试嘴巴夹取？** 这是 [r9 夹取预览版](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r9-clothpick-preview) 的内测功能，只支持 **HL-2915（3S）**。完成标定、启用整机服务并站稳后，在平地扶护着按 **A** 键。手柄普通行走用 **Start** 开始。

<a id="ai-development"></a>

## 让 AI Agent 做第一个小功能

**AI Agent 是能在你的电脑上读代码、改文件、运行命令的助手。** 任选一个能打开本地项目的工具，比如 Codex、Claude Code 或腾讯 WorkBuddy。这次让它给 DuckBuddy 的电量读数加一条进度条，先在电脑上用演示数据验证。 [Claude Code 入门](https://code.claude.com/docs/en/quickstart) · [WorkBuddy](https://cloud.tencent.com/product/workbuddy)

### 第一步：让 Agent 打开 DuckBuddy 项目

1. 打开 DuckBuddy 仓库，点绿色 **Code → Download ZIP**，下载后解压。打开里面能看到 `README.md` 和 `tools` 的那一层文件夹。 [DuckBuddy 仓库](https://github.com/Animan-AI/DuckBuddy)
2. 在 Agent 里新建一个本地任务，把刚解压的项目文件夹选为**工作目录**。工作目录就是这次交给它处理的文件夹。让它先找到下面这个样例文件，确认打开的是 DuckBuddy 项目。 [电量样例源码](https://github.com/Animan-AI/DuckBuddy/blob/main/tools/duckweb/examples/battery_check.py)
3. 在电脑上装好 Python 3。在这个任务里请 Agent 运行一次 `python --version`。看到 Python 3 的版本号，就能跑下面的样例。 [Python 安装说明](https://docs.python.org/3/using/windows.html)

### 第二步：交给 Agent 修改并运行

把下面这段话发给 Agent。它会在项目文件夹里完成任务，你可以等它跑完再看结果。

> 我第一次改 DuckBuddy。请先读 `tools/duckweb/examples/battery_check.py`，运行 `python tools/duckweb/examples/battery_check.py --demo`，记下原来的输出。然后直接修改这个文件，在电量百分比旁边加一条 10 格的文字进度条。保留 `--demo`、`--url` 和收不到电量时的提示。改完再运行一次 `--demo`，给我看修改前后的实际输出和修改了哪个文件。这次只在我的电脑上改这个样例。

### 第三步：看结果，再读实物电量

1. **看 Agent 的运行结果。** 改之前会显示「电量 78%（演示数据）」，改之后应保留 **78%**，旁边多一条进度条。Agent 应只列出这一个文件的改动。鸭子关着也能做完这一步。
2. **有实物再读一次。** 电脑能打开鸭子的网页后，把鸭子的 IP 告诉 Agent，请它运行下面的命令。让它把示例里的 `192.168.1.50` 换成你的地址，再把结果和网页上的电量对一下。

    ```powershell
    python tools/duckweb/examples/battery_check.py --url http://192.168.1.50:8088
    ```

3. **有问题就把现象告诉 Agent。** 比如「演示有进度条，网页明明有电量，脚本却提示没收到」。请它读实际报错，复现后再改。想重来，可以请它把这一个文件恢复成仓库里的原版。分享日志前先遮掉 Wi‑Fi 密码和 API Key。

下一个练习可以让 Agent 加上低电量提示，或者按[名字与性格设置](https://animan-ai.github.io/DuckBuddy/tutorial.html#step9)换一个名字。涉及舵机动作时，先在仿真中验证，再在有人扶护的实机上试，保留原机配置方便恢复。
