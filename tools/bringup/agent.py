# -*- coding: utf-8 -*-
"""bringup 的机器人端代理：一行一个 JSON 请求进、一行一个 JSON 应答出（stdin/stdout）。

app.py 每次连接时经 ssh 把本文件从 stdin 送过去执行（机器人上不用装任何东西），
或者在电脑上直接跑它、接飞特 USB 调试板（出厂 1M 的新舵机只能这样改 ID/波特率：
OPi 的 UART7 设 1M 实际是 750k）。

    {"id": 1, "op": "scan", "args": {"lo": 0, "hi": 30}}
    -> {"id": 1, "ok": true, "result": {...}}

只依赖标准库 + pyserial。总线操作前查 robotd：它在跑就拒绝（它占着 /dev/ttyS7）。
每个总线操作自己开关串口，不常驻占用。会动舵机的操作（center / wiggle / torque）由网页
先确认"托好了"再发；这里只做硬性兜底（限流、偏离/过流切扭矩、多圈读数拒绝）。

手动调试：  python3 agent.py --config ~/robot/active.toml   然后在 stdin 敲请求。
"""
import argparse
import array
import base64
import io
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import time
import wave

# ---- 飞特 HLS 内存表（HD-1910 相同），见 tools/servo/fts_bus.py / probe_regs.py ---------------
LOCK, ID_ADDR, BAUD_ADDR, MODE, TORQUE, GOAL_POS, PRES_POS = 55, 5, 6, 33, 40, 42, 56
BAUDS = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]  # 寄存器 6 的编号 0..7
BROADCAST = 0xFE
IMU_ID = 200
IMU_MODEL = 0x16D6
DEG_PER_COUNT = 360.0 / 4096

# (地址, 字节数, 名称, 类别, 有符号) 类别：ro 只读 / ee EEPROM（写时自动解锁再上锁）/ ram
REGS = [
    (0, 1, "固件主版本", "ro", False), (1, 1, "固件次版本", "ro", False), (3, 2, "型号", "ro", False),
    (5, 1, "ID", "ee", False), (6, 1, "波特率编号 0=1M 1=500k 2=250k", "ee", False),
    (7, 1, "返回延时", "ee", False), (8, 1, "应答级别", "ee", False),
    (9, 2, "最小角度限制", "ee", False), (11, 2, "最大角度限制", "ee", False),
    (13, 1, "最高温度 °C", "ee", False), (14, 1, "最高电压 ×0.1V", "ee", False),
    (15, 1, "最低电压 ×0.1V", "ee", False), (16, 2, "最大扭矩 ×0.1%", "ee", False),
    (18, 1, "相位", "ee", False), (19, 1, "卸载条件 b0电压 b2温度 b3电流", "ee", False),
    (20, 1, "LED 报警条件", "ee", False),
    (21, 1, "P 系数", "ee", False), (22, 1, "D 系数", "ee", False), (23, 1, "I 系数", "ee", False),
    (28, 2, "保护电流（9 位，单位同电流）", "ee", False),
    (31, 2, "位置校正（读数 = 原始 − 它，BIT15 符号位）", "ee", True),
    (33, 1, "运行模式 0位置 1恒速 2恒流 3PWM 4PD", "ee", False),
    (38, 1, "过流保护时间 ×10ms", "ee", False),
    (40, 1, "扭矩开关 0关 1开 2阻尼", "ram", False), (41, 1, "加速度", "ram", False),
    (42, 2, "目标位置", "ram", True), (44, 2, "目标电流（=0 无力）", "ram", True),
    (46, 2, "运行速度 ×0.732RPM（=0 停）", "ram", True), (48, 2, "转矩限制", "ram", False),
    (55, 1, "锁标志 0解锁 1上锁", "ram", False),
    (56, 2, "当前位置", "ro", True), (58, 2, "当前速度", "ro", True), (60, 2, "当前负载", "ro", False),
    (62, 1, "当前电压 ×0.1V", "ro", False), (63, 1, "当前温度 °C", "ro", False),
    (65, 1, "状态位", "ro", False), (66, 1, "移动标志", "ro", False), (69, 2, "当前电流", "ro", True),
]
REG_BY_ADDR = {r[0]: r for r in REGS}


def smag(v):
    """符号位+数值 -> 有符号。飞特用 BIT15 做方向位，不是补码。"""
    return -(v & 0x7FFF) if v & 0x8000 else v


def enc(v):
    v = int(v)
    return (0x8000 | -v) if v < 0 else v


class BusError(Exception):
    pass


class Bus:
    """飞特协议 1.0 的最小主机端。不处理回波（OPi 的 HAT 和 CH343 调试板都没有回波）。"""

    def __init__(self, port, baud, timeout=0.012):
        import serial  # 只在真要开总线时才需要
        self.port, self.baud = port, baud
        self.s = serial.Serial(port, baud, timeout=timeout, write_timeout=0.5)

    def close(self):
        self.s.close()

    def _send(self, sid, inst, params=()):
        body = [sid, len(params) + 2, inst] + list(params)
        self.s.reset_input_buffer()
        self.s.write(bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))
        self.s.flush()

    def _status(self, sid):
        hdr = self.s.read(4)
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF or hdr[2] != sid or hdr[3] < 2:
            return None
        n = hdr[3]
        rest = self.s.read(n)
        if len(rest) < n or (~(sid + n + sum(rest[:-1]))) & 0xFF != rest[-1]:
            return None
        return rest[0], bytes(rest[1:-1])

    def ping(self, sid):
        self._send(sid, 0x01)
        r = self._status(sid)
        return None if r is None else r[0]

    def read(self, sid, addr, n, tries=2):
        for _ in range(tries):
            self._send(sid, 0x02, (addr, n))
            r = self._status(sid)
            if r is not None and len(r[1]) == n:
                return r[1]
        return None

    def write(self, sid, addr, data, tries=2):
        """写入并等应答；广播不等。返回错误字节（0 = 正常）或 None（没应答）。"""
        for _ in range(tries):
            self._send(sid, 0x03, [addr] + list(data))
            if sid == BROADCAST:
                time.sleep(0.005)
                return 0
            r = self._status(sid)
            if r is not None:
                return r[0]
        return None

    def r8(self, sid, addr):
        d = self.read(sid, addr, 1)
        return None if d is None else d[0]

    def r16(self, sid, addr):
        d = self.read(sid, addr, 2)
        return None if d is None else d[0] | (d[1] << 8)

    def w8(self, sid, addr, v):
        return self.write(sid, addr, [int(v) & 0xFF])

    def w16(self, sid, addr, v):
        v = int(v) & 0xFFFF
        return self.write(sid, addr, [v & 0xFF, v >> 8])

    def must(self, err, what):
        if err is None:
            raise BusError(f"{what}：没应答")

    def pos(self, sid):
        v = self.r16(sid, PRES_POS)
        return None if v is None else smag(v)

    def sync_read(self, ids, addr, n):
        """SYNC_READ：按 ids 顺序各回一个状态包。返回 {id: bytes}，缺的、坏的不在里面。"""
        self._send(BROADCAST, 0x82, [addr, n] + list(ids))
        want = len(ids) * (6 + n)
        buf = self.s.read(want)
        out, i = {}, 0
        while i + 6 <= len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            sid, ln = buf[i + 2], buf[i + 3]
            end = i + 4 + ln
            if ln < 2 or end > len(buf):
                i += 1
                continue
            body = buf[i + 2:end - 1]
            if (~sum(body)) & 0xFF == buf[end - 1] and ln == n + 2 and sid in ids:
                out[sid] = bytes(buf[i + 5:end - 1])
                i = end
            else:
                i += 1
        return out


# ---- 运行环境 ----------------------------------------------------------------------------------

class Ctx:
    port = "/dev/ttyS7"
    baud = 500000
    usb = False
    config = None
    cfg_path = None


def procs():
    """[(pid, comm, cmdline)]。只读 /proc，不 fork，不会匹配到自己的命令行。"""
    out = []
    if not os.path.isdir("/proc"):
        return out
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            comm = open(f"/proc/{d}/comm").read().strip()
            cmd = open(f"/proc/{d}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        out.append((int(d), comm, cmd))
    return out


def robotd_pids():
    return [p for p, c, _ in procs() if c == "robotd"]


def user_unit(unit, action):
    return subprocess.run(["systemctl", "--user", action, unit], capture_output=True, text=True, timeout=20)


def load_config(path):
    path = os.path.realpath(os.path.expanduser(path))
    if not os.path.exists(path):
        return None, path
    try:
        import tomllib
    except ImportError:  # Python < 3.11
        return None, path
    with open(path, "rb") as f:
        return tomllib.load(f), path


def bus_cfg():
    return (Ctx.config or {}).get("bus", {})


def ma_per_count(args):
    v = args.get("ma_per_count")
    return float(v) if v else float(bus_cfg().get("current_ma_per_count", 6.5))


def open_bus(baud=None):
    if not Ctx.usb:
        pids = robotd_pids()
        if pids:
            raise BusError(f"robotd 在跑（PID {pids}），它占着舵机总线；先在总览页停掉它")
    try:
        return Bus(Ctx.port, int(baud or Ctx.baud))
    except Exception as e:  # noqa: BLE001
        raise BusError(f"打不开 {Ctx.port}：{e}")


class bus_session:
    """一次总线操作，用完就关：机器人上 robotd 随时可能要这个口，电脑上 FD 上位机也可能要。"""

    def __init__(self, baud=None):
        self.baud = baud

    def __enter__(self):
        self.bus = open_bus(self.baud)
        return self.bus

    def __exit__(self, *exc):
        self.bus.close()


# ---- 总览 --------------------------------------------------------------------------------------

def op_hello(args):
    bus = bus_cfg()
    unit = args.get("unit", "robotd-dev")
    info = dict(hostname=socket.gethostname(), python=sys.version.split()[0], usb=Ctx.usb,
                port=Ctx.port, baud=Ctx.baud, port_exists=os.path.exists(Ctx.port) or Ctx.usb,
                config_path=Ctx.cfg_path, config_found=Ctx.config is not None)
    if Ctx.config is not None:
        info["bus"] = {k: bus.get(k) for k in ("servo", "joint_ids", "joint_signs", "joint_offsets", "imu",
                                                "imu_axes", "current_ma_per_count", "port")}
        info["battery"] = Ctx.config.get("battery")
    if not Ctx.usb:
        info["robotd_pids"] = robotd_pids()
        info["robotd_unit"] = unit
        r = subprocess.run(["systemctl", "--user", "is-active", unit], capture_output=True, text=True)
        info["robotd_unit_state"] = r.stdout.strip()
        info["robotd_socket"] = os.path.exists("/tmp/robotd.sock")
        busy = {"visiond": "visiond", "tofd": "tofd", "voiced": "voiced", "lifed": "lifed", "duckcam": "duckcam",
                "padd": "padd"}
        info["services"] = sorted({name for p, c, cmd in procs() for key, name in busy.items()
                                   if c == key or (key + ".py") in cmd})
    return info


def op_robotd_stop(args):
    unit = args.get("unit", "robotd-dev")
    if user_unit(unit, "is-active").stdout.strip() == "active":
        r = user_unit(unit, "stop")
        if r.returncode != 0:
            raise RuntimeError(f"systemctl --user stop {unit} 失败：{r.stderr.strip()}")
        how = f"已 systemctl --user stop {unit}"
    else:
        pid = int(args.get("pid") or 0)
        if not pid or pid not in robotd_pids():
            raise RuntimeError(f"robotd 不是 {unit} 起的；要按网页上显示的 PID 停，当前 {robotd_pids()}")
        os.kill(pid, 15)
        how = f"已向 PID {pid} 发 SIGTERM"
    for _ in range(30):
        if not robotd_pids():
            break
        time.sleep(0.2)
    left = robotd_pids()
    return dict(msg=how + ("；robotd 已退出" if not left else f"；还在：{left}") + "。注意：停 robotd 不会卸力，扭矩还开着。",
                robotd_pids=left)


def op_robotd_start(args):
    unit = args.get("unit", "robotd-dev")
    if robotd_pids():
        return dict(msg="robotd 已经在跑", robotd_pids=robotd_pids())
    r = user_unit(unit, "start")
    if r.returncode != 0:
        raise RuntimeError(f"systemctl --user start {unit} 失败：{r.stderr.strip()}")
    for _ in range(25):
        if os.path.exists("/tmp/robotd.sock") and robotd_pids():
            break
        time.sleep(0.2)
    return dict(msg=f"已启动 {unit}", robotd_pids=robotd_pids())


def robotd_rpc(method, params=None, timeout=3.0):
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(timeout)
    s.connect("/tmp/robotd.sock")
    req = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params is not None:
        req["params"] = params
    s.sendall((json.dumps(req) + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        d = s.recv(65536)
        if not d:
            break
        buf += d
    s.close()
    return json.loads(buf.split(b"\n")[0] or b"{}")


def op_relax_all(args):
    """急停卸力：robotd 在跑就让它 relax，否则广播写扭矩 0。"""
    if not Ctx.usb and robotd_pids():
        r = robotd_rpc("robot.relax")
        return dict(msg="已发 robot.relax 给 robotd", reply=r)
    with bus_session(args.get("baud")) as bus:
        bus.w8(BROADCAST, TORQUE, 0)
        bus.w8(BROADCAST, TORQUE, 0)
    return dict(msg="已广播 扭矩开关=0（全部卸力）")


# ---- 舵机 --------------------------------------------------------------------------------------

def read_all(bus, sid):
    """读 0..70，按 REGS 解出来。分两段，失败了按 10 字节一段补读。"""
    raw = {}
    for start, n in ((0, 40), (40, 31)):
        d = bus.read(sid, start, n)
        if d is None:
            for s in range(start, start + n, 10):
                k = min(10, start + n - s)
                d2 = bus.read(sid, s, k)
                if d2 is not None:
                    raw.update({s + i: b for i, b in enumerate(d2)})
        else:
            raw.update({start + i: b for i, b in enumerate(d)})
    regs = {}
    for addr, n, _name, _kind, signed in REGS:
        if any(a not in raw for a in range(addr, addr + n)):
            continue
        v = raw[addr] if n == 1 else raw[addr] | (raw[addr + 1] << 8)
        regs[addr] = smag(v) if signed else v
    return regs


def summary(regs, mpc):
    g = regs.get
    pos = g(56)
    return dict(fw=f"{g(0)}.{g(1)}" if g(0) is not None else None, model=g(3), baud_idx=g(6),
                mode=g(33), unload=g(19), limits=[g(9), g(11)], p=g(21), d=g(22), lock=g(55),
                torque=g(40), pos=pos, multi_turn=pos is not None and not 0 <= pos <= 4095,
                volt=None if g(62) is None else g(62) / 10, temp=g(63), status=g(65),
                goal_curr_ma=None if g(44) is None else round(g(44) * mpc), speed=g(46),
                curr_ma=None if g(69) is None else round(g(69) * mpc))


def op_scan(args):
    lo, hi = int(args.get("lo", 0)), int(args.get("hi", 253))
    bauds = args.get("bauds") or [args.get("baud") or Ctx.baud]
    mpc = ma_per_count(args)
    found = []
    for b in bauds:
        with bus_session(b) as bus:
            bus.s.timeout = 0.006
            ids = [sid for sid in range(lo, hi + 1) if sid != BROADCAST and
                   (bus.ping(sid) is not None or bus.ping(sid) is not None)]
            bus.s.timeout = 0.02
            for sid in ids:
                if sid == IMU_ID:
                    d = bus.read(sid, 0, 8)
                    found.append(dict(id=sid, baud=int(b), imu=True,
                                      model=None if d is None else d[3] | (d[4] << 8),
                                      fw=None if d is None else f"{d[0]}.{d[1]}"))
                    continue
                found.append(dict(id=sid, baud=int(b), **summary(read_all(bus, sid), mpc)))
    return dict(found=found, bauds=[int(b) for b in bauds], range=[lo, hi])


def op_read_regs(args):
    sid = int(args["id"])
    with bus_session(args.get("baud")) as bus:
        regs = read_all(bus, sid)
    if not regs:
        raise BusError(f"ID {sid} 没应答")
    return dict(id=sid, regs=[dict(addr=a, size=n, name=name, kind=k, signed=s, value=regs.get(a))
                              for a, n, name, k, s in REGS], summary=summary(regs, ma_per_count(args)))


def op_read_raw(args):
    """原始字节（不按表解码），排查寄存器格式用。"""
    sid, addr, n = int(args["id"]), int(args["addr"]), int(args.get("n", 2))
    with bus_session(args.get("baud")) as bus:
        d = bus.read(sid, addr, n)
    if d is None:
        raise BusError(f"ID {sid} 没应答")
    return dict(id=sid, addr=addr, bytes=list(d), hex=d.hex(" "))


def write_reg(bus, sid, addr, value):
    if addr not in REG_BY_ADDR:
        raise BusError(f"地址 {addr} 不在寄存器表里")
    _, n, name, kind, signed = REG_BY_ADDR[addr]
    if kind == "ro":
        raise BusError(f"{addr} {name} 是只读的")
    v = enc(value) if signed else int(value)
    if kind == "ee":
        bus.must(bus.w8(sid, LOCK, 0), "解锁 EEPROM")
        time.sleep(0.02)
    err = bus.w8(sid, addr, v) if n == 1 else bus.w16(sid, addr, v)
    bus.must(err, f"写 {addr}")
    if kind == "ee":
        time.sleep(0.03)
        target = int(value) if addr == ID_ADDR else sid
        if addr != BAUD_ADDR:
            bus.w8(target, LOCK, 1)
    return err


def write_id(bus, old, new):
    """改 ID。应答是从新 ID 回来的（或者不回），所以不等应答：写完 PING 新 ID 核对，再在新 ID 上上锁。"""
    bus.must(bus.w8(old, LOCK, 0), "解锁 EEPROM")
    time.sleep(0.03)
    bus.write(old, ID_ADDR, [new], tries=1)
    time.sleep(0.08)
    if bus.ping(new) is None and bus.ping(new) is None:
        return False
    bus.w8(new, LOCK, 1)
    return True


def op_write_reg(args):
    sid, addr, value = int(args["id"]), int(args["addr"]), int(args["value"])
    if addr in (ID_ADDR, BAUD_ADDR):
        raise BusError("改 ID / 波特率请用对应的按钮（要核对冲突、换波特率重连）")
    with bus_session(args.get("baud")) as bus:
        write_reg(bus, sid, addr, value)
        time.sleep(0.02)
        regs = read_all(bus, sid)
    return dict(id=sid, addr=addr, readback=regs.get(addr))


def op_set_id(args):
    old, new = int(args["id"]), int(args["new_id"])
    if not 0 <= new <= 252 or new == IMU_ID:
        raise BusError("新 ID 要在 0..252 之间，且不能是 IMU 的 200")
    with bus_session(args.get("baud")) as bus:
        if bus.ping(old) is None:
            raise BusError(f"ID {old} 没应答")
        if bus.ping(new) is not None or bus.ping(new) is not None:
            raise BusError(f"ID {new} 已经有设备在用")
        ok = write_id(bus, old, new)
        lock = bus.r8(new, LOCK) if ok else None
    if not ok:
        raise BusError(f"写完后 ID {new} 没应答——给舵机断电重上后再扫一遍")
    return dict(msg=f"ID {old} → {new}，锁={lock}", id=new)


def op_set_baud(args):
    sid, to = int(args["id"]), int(args["to"])
    if to not in BAUDS:
        raise BusError(f"不支持的波特率 {to}")
    with bus_session(args.get("baud")) as bus:
        if bus.ping(sid) is None:
            raise BusError(f"ID {sid} 在 {bus.baud} 下没应答")
        if bus.baud == to:
            return dict(msg="已经是这个波特率")
        write_reg(bus, sid, BAUD_ADDR, BAUDS.index(to))
        time.sleep(0.1)
    note = ""
    if not Ctx.usb and to == 1000000:
        note = "（OPi 的 UART7 设 1M 实际 750k，机器人总线上将找不到它）"
    with bus_session(to) as bus:
        ok = bus.ping(sid) is not None
        if ok:
            bus.w8(sid, LOCK, 1)
            reg = bus.r8(sid, BAUD_ADDR)
    if not ok:
        return dict(msg=f"已写入；{to} 下没应答{note}——给舵机断电重上电后在 {to} 下再扫（锁会在下次改别的时补上）",
                    ok=False)
    return dict(msg=f"ID {sid} 改为 {to} bps，寄存器6={reg}，已上锁", ok=True)


def prepare_motion(bus, sid, mpc, current_ma, speed):
    """上扭矩前的检查与准备：返回当前位置。读数越界（多圈）就拒绝；先把目标设成当前位置。"""
    mode = bus.r8(sid, MODE)
    if mode not in (0, 4):
        raise BusError(f"ID {sid} 运行模式是 {mode}，只在位置模式 0 / 4 下动它")
    p = bus.pos(sid)
    if p is None:
        raise BusError(f"ID {sid} 读不到位置")
    goal_curr = max(1, min(2047, round(current_ma / mpc)))
    # 目标 + 44 + 46 一起写（42..47）：HD-1910 不重写 46 会零电流卡住；写 42 会自动上扭矩
    blk = b"".join(struct.pack("<H", enc(v)) for v in (p, goal_curr, speed))
    bus.must(bus.write(sid, GOAL_POS, list(blk)), "写目标")
    return p, goal_curr


def goal(bus, sid, target, goal_curr, speed):
    blk = b"".join(struct.pack("<H", enc(v)) for v in (target, goal_curr, speed))
    return bus.write(sid, GOAL_POS, list(blk))


def watch_move(bus, sid, start, target, mpc, seconds, max_ma, max_dev, trace, t_ref, done_tol=None):
    """盯着走向 target：过流（连续 3 次）或跑出 [起点, 目标] ± max_dev 计数就返回原因，否则 None。
    trace 追加 [秒（自 t_ref）, 位置, 电流 mA]。"""
    lo, hi = min(start, target) - max_dev, max(start, target) + max_dev
    over = 0
    t0 = time.time()
    p = start
    while time.time() - t0 < seconds:
        time.sleep(0.04)
        d = bus.read(sid, 56, 15)
        if d is None:
            continue
        p = smag(d[0] | d[1] << 8)
        ma = abs(smag(d[13] | d[14] << 8)) * mpc
        trace.append([round(time.time() - t_ref, 2), p, round(ma)])
        if not lo <= p <= hi:
            return f"位置 {p} 跑出范围 [{lo}, {hi}]"
        over = over + 1 if ma > max_ma else 0
        if over >= 3:
            return f"电流 {ma:.0f} mA 持续超过 {max_ma} mA"
        if done_tol is not None and abs(p - target) <= done_tol:
            return None
    return None


def op_wiggle(args):
    """以当前位置为中心 ±amp° 摆一下再回来，最后松扭矩。跨 0/4095 的一侧跳过。"""
    sid = int(args["id"])
    amp = float(args.get("amp_deg", 6))
    mpc = ma_per_count(args)
    current_ma = float(args.get("current_ma", 400))
    max_ma = float(args.get("max_ma", 1500))
    speed = int(args.get("speed", 100))
    max_dev = round(float(args.get("max_dev_deg", 15)) / DEG_PER_COUNT)
    a = round(amp / DEG_PER_COUNT)
    with bus_session(args.get("baud")) as bus:
        p0 = bus.pos(sid)
        if p0 is None:
            raise BusError(f"ID {sid} 读不到位置")
        if not 0 <= p0 <= 4095:
            raise BusError(f"ID {sid} 读数 {p0} 超出 0..4095（多圈计数），断舵机电再上电后再试")
        targets = [t for t in (p0 + a, p0 - a) if 0 <= t <= 4095] + [p0]
        skipped = 2 - (len(targets) - 1)
        p0, gc = prepare_motion(bus, sid, mpc, current_ma, speed)
        t_ref, trace = time.time(), []
        result, reached = None, []
        try:
            bus.must(bus.w8(sid, TORQUE, 1), "上扭矩")
            start = p0
            for t in targets:
                goal(bus, sid, t, gc, speed)
                result = watch_move(bus, sid, start, t, mpc, 0.9, max_ma, max_dev, trace, t_ref)
                p = bus.pos(sid)
                reached.append(dict(target=t, pos=p, err_deg=None if p is None else round((p - t) * DEG_PER_COUNT, 2)))
                if result:
                    break
                start = p if p is not None else t
        finally:
            bus.w8(sid, TORQUE, 0)
            bus.w8(sid, TORQUE, 0)
    ok = result is None and all(r["err_deg"] is not None and abs(r["err_deg"]) <= 2.5 for r in reached)
    return dict(ok=ok, aborted=result, reached=reached, skipped_sides=skipped, trace=trace,
                peak_ma=max([t[2] for t in trace] or [0]),
                msg=("中止：" + result if result else ("到位" if ok else "有一步没到位（误差 > 2.5°）")) + "；已松扭矩")


def op_center(args):
    """转到同圈最近的中位 2048+4096k 并保持扭矩（装舵盘用）。监督：过流 / 越走越远就切扭矩。"""
    sid = int(args["id"])
    mpc = ma_per_count(args)
    current_ma = float(args.get("current_ma", 400))
    max_ma = float(args.get("max_ma", 1500))
    speed = int(args.get("speed", 80))
    with bus_session(args.get("baud")) as bus:
        p0, gc = prepare_motion(bus, sid, mpc, current_ma, speed)
        target = 2048 + 4096 * round((p0 - 2048) / 4096)
        bus.must(bus.w8(sid, TORQUE, 1), "上扭矩")
        goal(bus, sid, target, gc, speed)
        why = watch_move(bus, sid, p0, target, mpc, 8.0, max_ma, 300, [], time.time(), done_tol=3)
        p = bus.pos(sid)
        if why or p is None or abs(p - target) > 8:
            bus.w8(sid, TORQUE, 0)
            bus.w8(sid, TORQUE, 0)
            return dict(ok=False, pos=p, target=target,
                        msg=f"没到中位（{why or f'停在 {p}'}），已松扭矩")
    return dict(ok=True, pos=p, target=target, msg=f"到中位 {p}（目标 {target}），扭矩保持中——装好舵盘后点「松扭矩」")


def op_set_center_here(args):
    """把舵机现在的位置记成 2048：改 EEPROM 31（位置校正，BIT15 符号位）。

    2026-10-31 HL-2915（固件 3.45）台架实测：读数 = (原始 − 31) mod 4096，0/4095 的跳变点和目标位置
    一起平移，断电保存。写 40=128（STS 的一键中位校正）在它上面没反应。HD-1910 还没测过。"""
    sid = int(args["id"])
    with bus_session(args.get("baud")) as bus:
        bus.must(bus.w8(sid, TORQUE, 0), "松扭矩")  # 扭矩开着改偏置，舵机会朝新坐标里的旧目标冲
        v31, p = bus.r16(sid, 31), bus.pos(sid)
        if v31 is None or p is None:
            raise BusError(f"ID {sid} 读不到 31 / 位置")
        if not 0 <= p <= 4095:
            raise BusError(f"读数 {p} 超出 0..4095（多圈计数），断舵机电再上电后再设")
        old = smag(v31)
        new = old + (p - 2048)
        new = (new + 2048) % 4096 - 2048  # 等价的偏置取 −2048..2047
        write_reg(bus, sid, 31, new)
        time.sleep(0.05)
        after = bus.pos(sid)
        back = bus.r16(sid, 31)
    ok = after is not None and abs(after - 2048) <= 3 and back is not None and smag(back) == new
    return dict(ok=ok, old_offset=old, new_offset=new, pos_before=p, pos_after=after,
                msg=(f"位置校正 {old} → {new}，现在读数 {after}" if ok
                     else f"没成功：31 读回 {None if back is None else smag(back)}，读数 {after}") + "；扭矩已松")


def op_torque(args):
    sid, on = int(args["id"]), int(args.get("on", 0))
    with bus_session(args.get("baud")) as bus:
        if on:
            p0, gc = prepare_motion(bus, sid, ma_per_count(args), float(args.get("current_ma", 400)),
                                    int(args.get("speed", 100)))
            if not 0 <= p0 <= 4095:
                bus.w8(sid, TORQUE, 0)
                raise BusError(f"读数 {p0} 超出 0..4095（多圈计数），不上扭矩")
        bus.must(bus.w8(sid, TORQUE, on), "写扭矩开关")
        return dict(msg=f"ID {sid} 扭矩 {'开（保持当前位置）' if on else '关'}", torque=bus.r8(sid, TORQUE))


# ---- 关节映射（装机后）：网页逐只摆动、点选关节，然后改 ID 对号，或者写 joint_ids ----------------

TEMP_BASE = 160  # 改号中途的临时 ID = 160 + 最终 ID；中断后扫到 16x / 17x，就知道它要去哪


def op_remap_ids(args):
    """按 moves=[[旧, 新], ...] 一起改号。先全部挪到临时号，再落到最终号，所以互换、轮换都行。"""
    moves = [(int(a), int(b)) for a, b in args.get("moves") or [] if int(a) != int(b)]
    if not moves:
        return dict(ok=True, log=[], msg="不用改：每只都已经是目标 ID")
    olds, news = [m[0] for m in moves], [m[1] for m in moves]
    if len(set(olds)) != len(olds) or len(set(news)) != len(news):
        raise BusError("旧 ID 或新 ID 有重复")
    if any(not 0 <= n <= 39 for n in news):
        raise BusError("新 ID 要在 0..39 之间（临时号用 160 + 新 ID）")
    log = []
    with bus_session(args.get("baud")) as bus:
        missing = [o for o in olds if bus.ping(o) is None and bus.ping(o) is None]
        if missing:
            raise BusError(f"这些 ID 没应答：{missing}，先重新扫描")
        for n in news:
            if n not in olds and (bus.ping(n) is not None or bus.ping(n) is not None):
                raise BusError(f"ID {n} 已经被一个不在映射里的设备占着")
            if bus.ping(TEMP_BASE + n) is not None:
                raise BusError(f"临时号 {TEMP_BASE + n} 已经有设备（上次改号中断留下的？先把它改回 {n}）")
        for phase, pairs in (("临时", [(o, TEMP_BASE + n) for o, n in moves]),
                             ("最终", [(TEMP_BASE + n, n) for o, n in moves])):
            for a, b in pairs:
                ok = write_id(bus, a, b)
                log.append(f"{phase}：{a} → {b}" + ("" if ok else "（没应答）"))
                if not ok:
                    raise BusError("；".join(log) + f"。ID {b} 写完没应答，停在这里：给舵机断电重上后再扫描"
                                   f"（{TEMP_BASE}+x 的是临时号，x 是它该去的 ID）")
        locks = {n: bus.r8(n, LOCK) for n in news}
    return dict(ok=True, log=log, locks=locks, msg=f"改了 {len(moves)} 只，已全部应答并上锁")


def op_write_joint_ids(args):
    """改机器人配置里 [bus] joint_ids 这一行：先备份，写完读回核对，不对就还原。"""
    import re
    ids = [int(i) for i in args.get("ids") or []]
    if len(ids) != 15 or len(set(ids)) != 15:
        raise BusError("joint_ids 要 15 个、互不相同")
    path = Ctx.cfg_path
    if Ctx.usb or not path or not os.path.exists(path):
        raise BusError(f"没有配置文件可写（{path}）")
    text = open(path, encoding="utf-8").read()
    line = "joint_ids = [" + ", ".join(str(i) for i in ids) + "]"
    note = time.strftime("# %Y-%m-%d") + " bringup: joint_ids mapped by wiggling each servo\n"
    text = re.sub(r"^# [0-9-]+ bringup: joint_ids mapped by wiggling each servo\n", "", text, flags=re.M)
    new, n = re.subn(r"^joint_ids\s*=\s*\[[^\]]*\]", lambda m: note + line, text, count=1, flags=re.M)
    if not n:
        new, n = re.subn(r"^\[bus\][ \t]*\n", lambda m: m.group(0) + note + line + "\n", text, count=1, flags=re.M)
    if not n:
        raise BusError(f"{path} 里没有 [bus] 段")
    backup = path + time.strftime(".bak-%Y%m%d-%H%M%S")
    while os.path.exists(backup):
        backup += "x"
    shutil.copy2(path, backup)
    with open(path, "w", encoding="utf-8") as f:
        f.write(new)
    m = re.search(r"^joint_ids\s*=\s*\[([^\]]*)\]", open(path, encoding="utf-8").read(), flags=re.M)
    cfg, _ = load_config(path)  # Python < 3.11 没有 tomllib，就只核对这一行
    if cfg is not None:
        got = [int(i) for i in cfg.get("bus", {}).get("joint_ids") or []]
    else:
        got = [int(x) for x in m.group(1).split(",") if x.strip()] if m else None
    if got != ids:
        shutil.copy2(backup, path)
        raise BusError(f"写完读回不对，已从 {backup} 还原")
    if cfg is not None:
        Ctx.config = cfg
    return dict(ok=True, path=path, backup=backup, line=line,
                msg=f"已写入 {path}（备份 {os.path.basename(backup)}）；robotd 要重启才生效")


# ---- 新舵机批量配置向导（USB 调试板，同 tools/servo/setup_ids.py 的链式做法） --------------------
#
# 出厂全是 ID 1 / 1M。链式：任何时刻只有刚接上的那一只顶着 ID 1，所以 ID 倒着分（15→1），
# 已配好的留在链上。每只配完立即改成最终波特率（500k），之后它就不在 1M 上了。
# 网页一步一步调：wiz_probe（谁上线了）→ wiz_configure（改号+EEPROM+波特率）→（第一只）
# 断电重上 wiz_verify → center / torque 装舵盘。

FRESH_ID = 1
UNLOAD, MIN_ANGLE, MAX_ANGLE = 19, 9, 11


def clean_ping(bus, sid, tries=6):
    """连续 tries 次都拿到合法应答才算：同 ID 撞车时偶尔也能凑出一个合法包。"""
    return all(bus.ping(sid) is not None for _ in range(tries))


def raw_activity(bus, sid):
    """线上有没有任何字节回来（撞车时是碎片也算）。"""
    for _ in range(3):
        bus._send(sid, 0x01)
        time.sleep(0.01)
        if bus.s.read(32):
            return True
    return False


def op_wiz_probe(args):
    """target 已在（原地重配）/ 有一只出厂 ID 1（fresh）/ 撞车 / 没有。顺带查已配好的还在不在。"""
    target = int(args["target"])
    final = int(args.get("final_baud", 500000))
    factory = [int(b) for b in args.get("factory_bauds") or [1000000, final]]
    assigned = [int(i) for i in args.get("assigned") or []]
    with bus_session(final) as bus:
        gone = [i for i in assigned if not clean_ping(bus, i, tries=3)]
    if gone:
        return dict(state="lost", gone=gone, msg=f"之前配好的 ID {gone} 现在不应答了：链路某处松了，先查线")
    for b in dict.fromkeys([final] + factory):
        with bus_session(b) as bus:
            if target != FRESH_ID and clean_ping(bus, target, tries=4):
                return dict(state="target", baud=b, msg=f"ID {target} 在 {b} bps 上已有舵机：是改过号的这只，原地重新配置")
    collide = None
    for b in factory:
        with bus_session(b) as bus:
            if clean_ping(bus, FRESH_ID):
                model = bus.r16(FRESH_ID, 3)
                mode = bus.r8(FRESH_ID, MODE)
                return dict(state="fresh", baud=b, model=model, mode=mode,
                            msg=f"找到一只 ID {FRESH_ID} 的舵机（{b} bps，型号 {model}，模式 {mode}）")
            if raw_activity(bus, FRESH_ID):
                collide = b
    if collide:
        return dict(state="collision", baud=collide,
                    msg=f"ID {FRESH_ID} 上有多只同时应答（{collide} bps 回包是碎片）：一次只能新接一只")
    return dict(state="none", msg="没找到新舵机：查电源、3 芯线方向、插到位没有")


def op_wiz_configure(args):
    """cur_id@baud → new_id：改号、单圈限位 0/4095、模式 0、卸载条件，上锁；再改成最终波特率并核对。"""
    cur, new = int(args["cur_id"]), int(args["new_id"])
    baud, final = int(args["baud"]), int(args.get("final_baud", 500000))
    unload = args.get("unload")
    steps = []
    with bus_session(baud) as bus:
        if not clean_ping(bus, cur, tries=3):
            raise BusError(f"ID {cur} 在 {baud} bps 上不应答")
        if cur != new:
            if clean_ping(bus, new, tries=2):
                raise BusError(f"ID {new} 已被占用")
            bus.must(bus.w8(cur, LOCK, 0), "解锁")
            time.sleep(0.05)
            bus.w8(cur, ID_ADDR, new)
            time.sleep(0.08)
            if not clean_ping(bus, new, tries=4):
                raise BusError(f"改完 ping 不到 ID {new}")
            if cur == FRESH_ID and clean_ping(bus, FRESH_ID, tries=4):
                raise BusError(f"ID {FRESH_ID} 上还有舵机在应答：当时接了不止一只，这次改动可能落到了别的舵机上")
            steps.append(f"ID {cur} → {new}")
        bus.must(bus.w8(new, LOCK, 0), "解锁")
        time.sleep(0.05)
        bus.w16(new, MIN_ANGLE, 0)
        time.sleep(0.03)
        bus.w16(new, MAX_ANGLE, 4095)
        time.sleep(0.03)
        bus.w8(new, MODE, 0)
        time.sleep(0.03)
        steps.append("限位 0/4095、模式 0")
        if unload is not None and unload != "":
            bus.w8(new, UNLOAD, int(unload))
            time.sleep(0.03)
            steps.append(f"卸载条件 {int(unload)}（0x{int(unload):02x}）")
        if baud != final:
            bus.w8(new, BAUD_ADDR, BAUDS.index(final))
            time.sleep(0.1)
            steps.append(f"波特率 {baud} → {final}")
        else:
            bus.w8(new, LOCK, 1)
            time.sleep(0.05)
    with bus_session(final) as bus:
        if not clean_ping(bus, new, tries=4):
            raise BusError(f"改波特率后 ID {new} 在 {final} bps 上不应答——断电重上后点\"检测\"，它会作为已改号的舵机被认出来")
        bus.w8(new, LOCK, 1)
        time.sleep(0.05)
        regs = read_all(bus, new)
    s = summary(regs, ma_per_count(args))
    bad = []
    if s["mode"] != 0:
        bad.append(f"模式读回 {s['mode']}")
    if s["limits"] != [0, 4095]:
        bad.append(f"限位读回 {s['limits']}")
    if unload not in (None, "") and s["unload"] != int(unload):
        bad.append(f"卸载条件读回 {s['unload']}")
    if s["lock"] != 1:
        bad.append("锁没上")
    if s["status"]:
        bad.append(f"状态位 {s['status']}")
    return dict(ok=not bad, problems=bad, id=new, baud=final, steps=steps, summary=s)


def op_wiz_verify(args):
    """断电重上电后：ID / 波特率 / 模式 / 限位 / 卸载条件都还在 = EEPROM 真的存住了。"""
    sid, final = int(args["id"]), int(args.get("final_baud", 500000))
    unload = args.get("unload")
    with bus_session(final) as bus:
        if not clean_ping(bus, sid, tries=4):
            raise BusError(f"掉电后 ID {sid} 在 {final} bps 上不应答：EEPROM 没存住，先别做剩下的")
        s = summary(read_all(bus, sid), ma_per_count(args))
    bad = []
    if s["mode"] != 0 or s["limits"] != [0, 4095]:
        bad.append(f"模式 {s['mode']}、限位 {s['limits']}")
    if unload not in (None, "") and s["unload"] != int(unload):
        bad.append(f"卸载条件 {s['unload']}")
    return dict(ok=not bad, problems=bad, summary=s,
                msg="掉电后配置都还在，EEPROM 保存正常" if not bad else "掉电后有配置没存住：" + "；".join(bad))


# ---- 整机测试 ----------------------------------------------------------------------------------

def joint_list():
    bus = bus_cfg()
    ids = [int(i) for i in bus.get("joint_ids") or []]
    return ids, bool(bus.get("imu", False))


def op_test_bus(args):
    """每个 ID 都要应答 + robotd 式 SYNC_READ（56 起 15 字节，IMU 排最后）N 轮的错误率。"""
    ids = [int(i) for i in args.get("ids") or []]
    imu = bool(args.get("imu", True))
    if not ids:
        ids, imu_cfg = joint_list()
        imu = imu and imu_cfg
    if not ids:
        raise BusError("配置里没有 joint_ids，先在舵机页扫描")
    rounds = int(args.get("rounds", 500))
    mpc = ma_per_count(args)
    order = ids + ([IMU_ID] if imu else [])
    with bus_session(args.get("baud")) as bus:
        bus.s.timeout = 0.02
        missing = [sid for sid in order if all(bus.ping(sid) is None for _ in range(3))]
        servos = []
        for sid in ids:
            d = bus.read(sid, 56, 15)
            if d is None:
                servos.append(dict(id=sid, ok=False))
                continue
            pos = smag(d[0] | d[1] << 8)
            servos.append(dict(id=sid, ok=True, pos=pos, multi_turn=not 0 <= pos <= 4095, volt=d[6] / 10,
                               temp=d[7], status=d[9], curr_ma=round(abs(smag(d[13] | d[14] << 8)) * mpc)))
        bus.s.timeout = 0.03
        fails = {sid: 0 for sid in order}
        bad_rounds = 0
        t0 = time.time()
        for _ in range(rounds):
            got = bus.sync_read(order, 56, 15)
            miss = [sid for sid in order if sid not in got]
            for sid in miss:
                fails[sid] += 1
            bad_rounds += bool(miss)
        dt = time.time() - t0
    total = rounds * len(order)
    err_rate = sum(fails.values()) / total if total else 0
    volts = [s["volt"] for s in servos if s.get("ok")]
    problems = []
    if missing:
        problems.append(f"不应答：{missing}")
    if err_rate > float(args.get("max_err", 0.005)):
        problems.append(f"SYNC_READ 错误率 {err_rate:.2%}")
    hot = [s["id"] for s in servos if s.get("ok") and s["status"]]
    if hot:
        problems.append(f"状态位非 0：{hot}")
    mt = [s["id"] for s in servos if s.get("multi_turn")]
    if mt:
        problems.append(f"多圈读数（断舵机电清）：{mt}")
    return dict(ok=not problems, problems=problems, order=order, missing=missing, servos=servos,
                rounds=rounds, err_rate=err_rate, bad_rounds=bad_rounds,
                fails={str(k): v for k, v in fails.items() if v}, ms_per_round=round(dt / max(1, rounds) * 1000, 2),
                volt_min=min(volts) if volts else None, volt_max=max(volts) if volts else None)


def half(bits):
    s = -1.0 if bits & 0x8000 else 1.0
    e, f = (bits >> 10) & 0x1F, bits & 0x3FF
    if e == 0:
        return s * f * 2.0 ** -24
    if e == 0x1F:
        return float("nan") if f else s * float("inf")
    return s * (1 + f / 1024) * 2.0 ** (e - 15)


def gravity_sensor(x, y, z):
    """SFLP 四元数（x,y,z，w=√(1−…)）下，世界 −z 在传感器系里的方向（同 duck-control imu.rs）。"""
    w = math.sqrt(max(0.0, 1 - x * x - y * y - z * z))
    v = (0.0, 0.0, -1.0)
    t = ((y * v[2] - z * v[1]) * 2, (z * v[0] - x * v[2]) * 2, (x * v[1] - y * v[0]) * 2)
    c = (y * t[2] - z * t[1], z * t[0] - x * t[2], x * t[1] - y * t[0])
    return [v[i] - w * t[i] + c[i] for i in range(3)]


def to_trunk(g, axes):
    """[bus] imu_axes（如 "y,x,-z"：躯干 x = 传感器 y …）把传感器系向量换到躯干系。"""
    out = []
    for tok in axes.split(","):
        tok = tok.strip()
        sgn = -1.0 if tok.startswith("-") else 1.0
        out.append(sgn * g["xyz".index(tok.lstrip("+-"))])
    return out


def op_test_imu(args):
    seconds = float(args.get("seconds", 2.0))
    axes = args.get("axes") or bus_cfg().get("imu_axes") or "z,y,-x"
    with bus_session(args.get("baud")) as bus:
        bus.s.timeout = 0.02
        head = bus.read(IMU_ID, 0, 8)
        if head is None:
            raise BusError("ID 200（IMU 板）没应答")
        diag0 = bus.read(IMU_ID, 90, 8)
        samples = []
        t0 = time.time()
        reads = fails = 0
        while time.time() - t0 < seconds:
            reads += 1
            d = bus.read(IMU_ID, 56, 15, tries=1)
            if d is None:
                fails += 1
                continue
            samples.append((time.time() - t0, d))
            time.sleep(0.008)
        diag1 = bus.read(IMU_ID, 90, 8)
    if len(samples) < 5:
        raise BusError(f"IMU 读数太少（{len(samples)}/{reads}）")
    model = head[3] | head[4] << 8
    # 采样计数（字节 12，每个融合输出 +1，8 位回绕）
    cnt = 0
    for (_, a), (_, b) in zip(samples, samples[1:]):
        cnt += (b[12] - a[12]) % 256
    rate = cnt / (samples[-1][0] - samples[0][0])
    gyro = [[struct.unpack_from("<h", d, 2 * k)[0] * 0.0175 for k in range(3)] for _, d in samples]
    gmean = [sum(g[k] for g in gyro) / len(gyro) for k in range(3)]
    gstd = [math.sqrt(sum((g[k] - gmean[k]) ** 2 for g in gyro) / len(gyro)) for k in range(3)]
    last = samples[-1][1]
    q = [half(struct.unpack_from("<H", last, 6 + 2 * k)[0]) for k in range(3)]
    qn = math.sqrt(sum(v * v for v in q))
    flags = last[13]
    gs = gravity_sensor(*q) if qn <= 1.02 else [0, 0, 0]
    gt = to_trunk(gs, axes)
    pitch = math.degrees(math.atan2(gt[0], math.hypot(gt[1], gt[2])))
    roll = math.degrees(math.atan2(-gt[1], -gt[2]))
    dg = lambda d, k: None if d is None else d[2 * k] | d[2 * k + 1] << 8  # noqa: E731
    diag = None
    if diag0 and diag1:
        diag = dict(rx_err=(dg(diag1, 0) - dg(diag0, 0)) % 65536, expired=(dg(diag1, 1) - dg(diag0, 1)) % 65536,
                    replies=(dg(diag1, 2) - dg(diag0, 2)) % 65536, rx_err_total=dg(diag1, 0),
                    expired_total=dg(diag1, 1))
    problems = []
    if model != IMU_MODEL:
        problems.append(f"型号 0x{model:04X} 不是 0x{IMU_MODEL:04X}")
    if flags & 3 != 3:
        problems.append(f"标志 {flags:#04x}：传感器未配置或姿态无效")
    if not 90 <= rate <= 150:
        problems.append(f"融合输出 {rate:.0f} Hz（应约 120）")
    if not 0.98 <= qn <= 1.02 and qn > 0:
        problems.append(f"|q| = {qn:.3f}")
    if qn == 0:
        problems.append("四元数全 0（SFLP 还没输出）")
    if max(gstd) > float(args.get("max_gyro_std", 1.0)):
        problems.append(f"静止时陀螺噪声 {max(gstd):.2f} dps（要放稳再测）")
    if max(abs(v) for v in gmean) > 5:
        problems.append(f"陀螺零偏 {max(abs(v) for v in gmean):.1f} dps")
    if diag and diag["rx_err"]:
        problems.append(f"IMU 板串口接收错 {diag['rx_err']} 次")
    if fails > reads * 0.01:
        problems.append(f"读失败 {fails}/{reads}")
    return dict(ok=not problems, problems=problems, fw=f"{head[0]}.{head[1]}", model=f"0x{model:04X}",
                rate_hz=round(rate, 1), flags=flags, quat=[round(v, 4) for v in q], quat_norm=round(qn, 4),
                gyro_mean_dps=[round(v, 2) for v in gmean], gyro_std_dps=[round(v, 3) for v in gstd],
                gravity_sensor=[round(v, 3) for v in gs], axes=axes, pitch_deg=round(pitch, 1),
                roll_deg=round(roll, 1), reads=reads, read_fails=fails, diag=diag)


# ---- 音频 --------------------------------------------------------------------------------------

RATE = 48000


def alsa_devs():
    rc = os.path.expanduser("~/.asoundrc")
    txt = open(rc).read() if os.path.exists(rc) else ""
    if "pcm.duck_out" in txt and "pcm.duck_in" in txt:
        return "duck_out", "duck_in"
    return "plughw:CARD=aic3104,DEV=0", "plughw:CARD=aic3104,DEV=0"


def write_wav(path, segments):
    """segments: [(秒, 频率 Hz 或 0, dBFS)]，双声道同样内容（功放只接左声道）。"""
    frames = array.array("h")
    for secs, freq, dbfs in segments:
        n = int(secs * RATE)
        amp = 32767 * 10 ** (dbfs / 20) if freq else 0
        fade = int(0.01 * RATE)
        for i in range(n):
            g = min(1.0, i / fade, (n - 1 - i) / fade) if freq else 0
            v = int(amp * g * math.sin(2 * math.pi * freq * i / RATE))
            frames.append(v)
            frames.append(v)
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(frames.tobytes())


def play_and_record(wav_path, record_s):
    """先开放音（静音开头）再开录音：先录后放，录音会整段顶到 ±32767（10-25 踩过）。"""
    out_dev, in_dev = alsa_devs()
    if "aic3104" not in open("/proc/asound/cards").read():
        raise RuntimeError("声卡 aic3104 没注册（看 i2c-2 和 aic3104-a733.service）")
    player = subprocess.Popen(["aplay", "-q", "-D", out_dev, wav_path], stderr=subprocess.PIPE)
    time.sleep(0.25)
    try:
        r = subprocess.run(["arecord", "-q", "-D", in_dev, "-f", "S16_LE", "-r", str(RATE), "-c", "2",
                            "-t", "raw", "-s", str(int(record_s * RATE))],
                           capture_output=True, timeout=record_s + 10)
    finally:
        try:
            player.wait(timeout=record_s + 5)
        except subprocess.TimeoutExpired:
            player.kill()
    if r.returncode != 0 or not r.stdout:
        raise RuntimeError("arecord 失败：" + r.stderr.decode(errors="replace").strip())
    a = array.array("h")
    a.frombytes(r.stdout[: len(r.stdout) // 4 * 4])
    skip = int(0.15 * RATE) * 2  # 开头的上电瞬态
    return list(a[skip::2]), list(a[skip + 1::2]), (out_dev, in_dev), player.returncode


def windows(x, n=2400):
    return [x[i:i + n] for i in range(0, len(x) - n + 1, n)]


def chan_stats(x):
    if not x:
        return dict(dc=0, rms=0, floor=0, peak=0, clip=0)
    dc = sum(x) / len(x)
    w = sorted(math.sqrt(sum((v - dc) ** 2 for v in seg) / len(seg)) for seg in windows(x)) or [0]
    rms = math.sqrt(sum((v - dc) ** 2 for v in x) / len(x))
    clip = sum(1 for v in x if abs(v) >= 32700) / len(x)
    return dict(dc=round(dc), rms=round(rms, 1), floor=round(w[len(w) // 10], 1), peak=round(w[-1], 1),
                clip=round(clip, 4))


def op_test_mic(args):
    secs = float(args.get("seconds", 4))
    wav = "/tmp/bringup-silence.wav"
    write_wav(wav, [(secs + 1.0, 0, 0)])
    left, right, devs, _ = play_and_record(wav, secs)
    chans = dict(left=chan_stats(left), right=chan_stats(right))
    problems = []
    for name, s in chans.items():
        cn = "左" if name == "left" else "右"
        if s["rms"] < 2:
            problems.append(f"{cn}声道几乎全 0（codec 没起来 / i2c-2）")
        if abs(s["dc"]) > float(args.get("max_dc", 3000)):
            problems.append(f"{cn}声道直流偏移 {s['dc']}")
        if s["clip"] > 0.02:
            problems.append(f"{cn}声道 {s['clip']:.1%} 削顶")
        if s["peak"] < float(args.get("min_ratio", 4)) * max(s["floor"], 1):
            problems.append(f"{cn}声道没听到声音（峰 {s['peak']} / 底噪 {s['floor']}）——录音时要拍手或说话")
    return dict(ok=not problems, problems=problems, devices=devs, seconds=secs, **chans)


def goertzel(seg, freq):
    k = 2 * math.cos(2 * math.pi * freq / RATE)
    s1 = s2 = 0.0
    for v in seg:
        s1, s2 = v + k * s1 - s2, s1
    return (s1 * s1 + s2 * s2 - k * s1 * s2) / (len(seg) ** 2)


def op_test_speaker(args):
    """喇叭放 1 kHz，同时用麦克风录：亮的窗口里 1 kHz 能量比安静窗口高多少 dB。不靠对齐时间。"""
    freq = float(args.get("freq", 1000))
    dbfs = float(args.get("dbfs", -12))
    tone = float(args.get("seconds", 1.5))
    wav = "/tmp/bringup-tone.wav"
    write_wav(wav, [(0.5, 0, 0), (tone, freq, dbfs), (0.8, 0, 0)])
    left, right, devs, rc = play_and_record(wav, tone + 1.0)
    res = {}
    for name, x in (("left", left), ("right", right)):
        p = sorted(goertzel(seg, freq) for seg in windows(x))
        if not p:
            continue
        loud = sum(p[len(p) // 2:]) / max(1, len(p) - len(p) // 2)
        quiet = sum(p[: max(1, len(p) // 4)]) / max(1, len(p) // 4)
        res[name] = dict(tone_db=round(10 * math.log10(max(loud, 1e-9) / max(quiet, 1e-9)), 1),
                         tone_amp=round(math.sqrt(max(loud, 0)) * 2, 1))
    best = max((v["tone_db"] for v in res.values()), default=0)
    need = float(args.get("min_db", 15))
    problems = [] if best >= need else [f"麦克风只录到 1 kHz 高出 {best} dB（要 ≥ {need}）：喇叭/功放/U6 DIN"]
    if rc not in (0, None):
        problems.append(f"aplay 退出码 {rc}")
    return dict(ok=not problems, problems=problems, freq=freq, dbfs=dbfs, devices=devs, best_db=best, **res)


# ---- ToF ---------------------------------------------------------------------------------------

def tofd_bus_default():
    unit = os.path.expanduser("~/.config/systemd/user/tofd-dev.service")
    if os.path.exists(unit):
        txt = open(unit).read()
        i = txt.find("--bus ")
        if i >= 0:
            return txt[i + 6:].split()[0]
    return "/dev/i2c-2"


def read_tof(sock_path, seconds):
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(3.0)
    s.connect(sock_path)
    s.sendall(b'{"jsonrpc":"2.0","id":1,"method":"tof.stream"}\n')
    frames, buf, status = [], b"", None
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            d = s.recv(65536)
        except socket.timeout:
            break
        if not d:
            break
        buf += d
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                m = json.loads(line)
            except ValueError:
                continue
            if m.get("method") == "tof.frame":
                frames.append((time.time(), m["params"]))
            elif "result" in m:
                status = m["result"]
            elif "error" in m:
                raise RuntimeError(f"tofd：{m['error']}")
    s.close()
    return frames, status


def op_test_tof(args):
    seconds = float(args.get("seconds", 2.5))
    sock = "/tmp/tofd.sock"
    child, started = None, False
    if not os.path.exists(sock) or not any(c == "tofd" for _, c, _ in procs()):
        exe = os.path.expanduser(args.get("tofd") or "~/robot/tofd")
        if not os.path.exists(exe):
            raise RuntimeError(f"tofd 没在跑，也没有 {exe}")
        sock = "/tmp/bringup-tofd.sock"
        if os.path.exists(sock):
            os.unlink(sock)
        bus = args.get("bus") or tofd_bus_default()
        child = subprocess.Popen([exe, "--bus", bus, "--socket", sock], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.PIPE)
        started = True
        for _ in range(60):
            if os.path.exists(sock) or child.poll() is not None:
                break
            time.sleep(0.1)
    try:
        if child is not None and child.poll() is not None:
            raise RuntimeError("tofd 起不来：" + child.stderr.read().decode(errors="replace").strip()[-400:])
        # 自己起的 tofd 要先初始化传感器（固件上传几秒），多等一会儿
        frames, status = read_tof(sock, seconds + (6.0 if started else 0.0))
    finally:
        if child is not None:
            child.terminate()  # 只停自己起的这个（Popen 持有的确切 PID）
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
    if started:
        frames = [f for f in frames if f[0] >= frames[-1][0] - seconds] if frames else []
    if len(frames) < 3:
        return dict(ok=False, problems=[f"只收到 {len(frames)} 帧"], status=status, started_tofd=started)
    hz = (len(frames) - 1) / (frames[-1][0] - frames[0][0])
    valid_counts, centre = [], []
    for _, p in frames:
        st, dist = p.get("status") or [], p.get("distance_mm") or []
        valid = [i for i, v in enumerate(st) if v in (5, 9)]
        valid_counts.append(len(valid))
        mid = sorted(dist[r * 8 + c] for r in range(2, 6) for c in range(2, 6) if r * 8 + c in valid)
        if mid:
            centre.append(mid[len(mid) // 2])
    last = frames[-1][1]
    vmed = sorted(valid_counts)[len(valid_counts) // 2]
    problems = []
    if hz < 10:
        problems.append(f"帧率 {hz:.1f} Hz（应约 15）")
    if vmed < int(args.get("min_valid", 32)):
        problems.append(f"有效区只有 {vmed}/64")
    return dict(ok=not problems, problems=problems, hz=round(hz, 1), frames=len(frames), valid_median=vmed,
                centre_mm=sorted(centre)[len(centre) // 2] if centre else None,
                grid=last.get("distance_mm"), grid_status=last.get("status"), started_tofd=started,
                sensor=(status or {}).get("sensor") if isinstance(status, dict) else None)


# ---- 摄像头 ------------------------------------------------------------------------------------

def bmp_grey(w, h, pix):
    row = (w + 3) & ~3
    pal = b"".join(bytes((i, i, i, 0)) for i in range(256))
    data = b"".join(bytes(pix[y * w:(y + 1) * w]) + b"\0" * (row - w) for y in range(h - 1, -1, -1))
    off = 14 + 40 + len(pal)
    return (b"BM" + struct.pack("<IHHI", off + len(data), 0, 0, off)
            + struct.pack("<IiiHHIIiiII", 40, w, h, 1, 8, 0, len(data), 2835, 2835, 256, 0) + pal + data)


def find_duckcam():
    for p in ("~/robot/duckcam", "~/duckcam", "~/visiond/duckcam", "~/robot/visiond/duckcam"):
        p = os.path.expanduser(p)
        if os.path.exists(p):
            return p
    return shutil.which("duckcam")


def op_test_camera(args):
    names = {}
    base = "/sys/class/video4linux"
    if os.path.isdir(base):
        for d in os.listdir(base):
            try:
                names[d] = open(os.path.join(base, d, "name")).read().strip()
            except OSError:
                pass
    vin = sorted((n for n, v in names.items() if v.startswith("vin_video")), key=lambda n: int(n[5:]))
    mods = open("/proc/modules").read() if os.path.exists("/proc/modules") else ""
    if not vin:
        hint = "" if "vin_v4l2" in mods else "：vin 驱动没加载，要 sudo modprobe -a vin_io imx219 vin_v4l2"
        return dict(ok=False, problems=["没有 vin_video 设备" + hint], nodes=names)
    busy = [f"{c}({p})" for p, c, cmd in procs() if c == "duckcam" or "visiond.py" in cmd]
    if busy:
        return dict(ok=False, problems=[f"摄像头被占用：{busy}，先停 visiond 再测"], device="/dev/" + vin[0])
    exe = find_duckcam()
    if not exe:
        return dict(ok=False, problems=["找不到 duckcam（tools/visiond/duckcam.c 在 OPi 上编译）"], device="/dev/" + vin[0])
    n = int(args.get("frames", 45))
    r = subprocess.run([exe, "--half", "--frames", str(n), "--dev", "/dev/" + vin[0]], capture_output=True,
                       timeout=30)
    data, frames, i = r.stdout, [], 0
    while i + 32 <= len(data) and data[i:i + 4] == b"DCAM":
        w, h, seq, ts, exp, gain = struct.unpack_from("<IIIqII", data, i + 4)
        frames.append((w, h, seq, ts, exp, gain, i + 32))
        i += 32 + w * h
    if not frames:
        err = r.stderr.decode(errors="replace").strip().splitlines()
        return dict(ok=False, problems=["duckcam 没出帧：" + (err[-1] if err else f"退出码 {r.returncode}")],
                    device="/dev/" + vin[0])
    w, h, seq, ts, exp, gain, off = frames[-1]
    pix = data[off:off + w * h]
    step = 4
    tw, th = w // step, h // step
    thumb = bytes(pix[(y * step) * w + x * step] for y in range(th) for x in range(tw))
    sample = pix[::97]
    mean = sum(sample) / len(sample)
    std = math.sqrt(sum((v - mean) ** 2 for v in sample) / len(sample))
    tail = frames[len(frames) // 3:]
    fps = (len(tail) - 1) / ((tail[-1][3] - tail[0][3]) / 1e9) if len(tail) > 1 and tail[-1][3] > tail[0][3] else None
    problems = []
    if fps is not None and fps < 10:
        problems.append(f"帧率 {fps:.1f}")
    if not 8 <= mean <= 245:
        problems.append(f"亮度均值 {mean:.0f}（全黑/全白）")
    if std < 3:
        problems.append(f"画面几乎没有纹理（标准差 {std:.1f}）")
    return dict(ok=not problems, problems=problems, device="/dev/" + vin[0], frames=len(frames), size=[w, h],
                fps=None if fps is None else round(fps, 1), exposure_us=exp, gain=gain, mean=round(mean, 1),
                std=round(std, 1), thumb_bmp=base64.b64encode(bmp_grey(tw, th, thumb)).decode())


OPS = {name[3:]: fn for name, fn in globals().items() if name.startswith("op_")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="~/robot/active.toml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--usb", action="store_true", help="电脑上接 USB 调试板，不查 robotd、没有外设测试")
    a = ap.parse_args()
    Ctx.usb = a.usb
    Ctx.baud = a.baud
    if not a.usb:
        Ctx.config, Ctx.cfg_path = load_config(a.config)
    Ctx.port = a.port or bus_cfg().get("port") or "/dev/ttyS7"
    out = sys.stdout
    out.write(json.dumps({"hello": True, "ops": sorted(OPS)}) + "\n")
    out.flush()
    for line in sys.stdin.buffer:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except ValueError:
            continue
        rid = req.get("id")
        fn = OPS.get(req.get("op"))
        try:
            if fn is None:
                raise RuntimeError(f"没有这个操作：{req.get('op')}")
            resp = {"id": rid, "ok": True, "result": fn(req.get("args") or {})}
        except Exception as e:  # noqa: BLE001
            resp = {"id": rid, "ok": False, "error": f"{e}" if isinstance(e, (BusError, RuntimeError))
                    else f"{type(e).__name__}: {e}"}
        out.write(json.dumps(resp) + "\n")
        out.flush()


if __name__ == "__main__":
    main()
