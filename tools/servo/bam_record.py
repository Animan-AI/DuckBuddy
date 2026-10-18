# -*- coding: utf-8 -*-
"""HL-2915 摆杆台录制 —— 产出 BAM（Better Actuator Models）可直接拟合的日志。

训练仓库 microduck_rl 的执行器模型用 BAM 辨识；BAM 自带的飞特录制脚本是给 STS3215 的
（另一张内存表、1M、Linux 串口），这里按同样的轨迹和日志格式重写给 HLS 内存表 + 调试板。

台架：舵机固定在桌边，输出轴水平；舵盘上装一根刚性摆杆，末端挂配重。
摆杆**自然下垂为 0 位**，要能向两侧自由摆过 ±110°（不碰桌子、不缠线）。
需要量：摆杆自身质量 --arm-mass、末端配重 --mass（kg），轴心到配重中心的距离 --length（m）。

用法（调试板 COM11，只接这一只舵机，12V 供电；先关掉 FD 上位机）：
    python tools/servo/bam_record.py --check                       # 只读：看 0 位、电压、温度
    python tools/servo/bam_record.py --mass 0.1 --arm-mass 0.02 --length 0.1 --logdir training/data/bam-hl2915/m100
    python tools/servo/bam_record.py ... --kps 16 --trajectories sin_sin   # 只录一组

每组（Kp × 轨迹）6 秒，存一个 JSON。录制中 Ctrl-C 会立即关扭矩。
寄存器与机器人一致：目标电流 44（默认 1300 mA）、运行速度 46 = 250、加速度 41 = 0、
实时 Kp 50 / Kd 51（Kd 默认 8，与 deploy/robots/opi-hl2915-rev-a.toml 相同）。
"""
import argparse
import datetime
import json
import math
import os
import sys
import time

import serial

from fts_bus import (ACCEL, GOAL_CURR, GOAL_SPEED, MAX_ANGLE, MIN_ANGLE, MODE, PRES_POS,
                     TORQUE, smag)

KP, KD = 50, 51
COUNTS_PER_RAD = 4096 / (2 * math.pi)
RAD_S_PER_SPEED = 0.732 * 2 * math.pi / 60       # 58 号：0.732 rpm/计数
MA_PER_COUNT = 6.5
RUNNING_SPEED = 250
MAX_TEMP = 65


# ---- BAM 的轨迹（bam/trajectory.py 原样移植，6 秒，0 = 下垂）---------------------
def cubic(keyframes, t):
    if t <= keyframes[0][0]:
        return keyframes[0][1]
    if t >= keyframes[-1][0]:
        return keyframes[-1][1]
    for (t0, x0, v0), (t1, x1, v1) in zip(keyframes, keyframes[1:]):
        if t0 <= t <= t1:
            h, s = t1 - t0, (t - t0) / (t1 - t0)
            h00, h10 = 2*s**3 - 3*s**2 + 1, s**3 - 2*s**2 + s
            h01, h11 = -2*s**3 + 3*s**2, s**3 - s**2
            return h00*x0 + h10*h*v0 + h01*x1 + h11*h*v1


TRAJECTORIES = {
    # 2 s 抬到 -90° 后断扭矩自由下落：反驱 / 低速摩擦
    "lift_and_drop": lambda t: (cubic([[0, 0, 0], [2, -math.pi/2, 0]], t), t < 2.0),
    # sin(t²) 越摆越快：主力轨迹，扫过整个速度范围（末段会碰到舵机转速上限）
    "sin_time_square": lambda t: (math.sin(t*t), True),
    # 慢速 0 → 90° → 72°：静摩擦与负载
    "up_and_down": lambda t: (cubic([[0, 0, 0], [3, math.pi/2, 0], [6, 0.8*math.pi/2, 0]], t), True),
    # 多频叠加，6 s 内最大约 ±101°
    "sin_sin": lambda t: (math.sin(t)*math.pi/2 + math.sin(5*t)*0.5*math.sin(2*t), True),
    # 全程无扭矩
    "nothing": lambda t: (0.0, False),
}
DURATION = 6.0
MAX_EXCURSION = 1.8        # rad，sin_sin 6 s 内最大 1.76


class Servo:
    """单只舵机的快速读写：读一次 56..70 共 15 字节；目标位置走 SYNC_WRITE（无应答）。"""

    def __init__(self, port, baud, sid):
        self.s = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2)
        self.id = sid

    def _send(self, sid, inst, params):
        body = [sid, len(params) + 2, inst] + list(params)
        self.s.write(bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))

    def _reply(self):
        hdr = self.s.read(4)
        if len(hdr) < 4 or hdr[:2] != b"\xff\xff" or hdr[2] != self.id:
            return None
        rest = self.s.read(hdr[3])
        if len(rest) < hdr[3] or (~(sum(hdr[2:]) + sum(rest[:-1]))) & 0xFF != rest[-1]:
            return None
        return rest[1:-1]

    def read(self, addr, n):
        self.s.reset_input_buffer()
        self._send(self.id, 0x02, (addr, n))
        return self._reply()

    def write(self, addr, data):
        self.s.reset_input_buffer()
        self._send(self.id, 0x03, [addr] + list(data))
        return self._reply() is not None

    def w8(self, addr, v):
        return self.write(addr, [v & 0xFF])

    def w16(self, addr, v):
        return self.write(addr, [v & 0xFF, (v >> 8) & 0xFF])

    def goal(self, counts):
        counts = max(0, min(4095, int(round(counts))))
        self._send(0xFE, 0x83, [42, 2, self.id, counts & 0xFF, counts >> 8])

    def state(self):
        d = self.read(PRES_POS, 15)
        if d is None or len(d) < 15:
            return None
        return dict(pos=smag(d[0] | d[1] << 8), speed=smag(d[2] | d[3] << 8),
                    volts=d[6] * 0.1, temp=d[7], status=d[9],
                    current=smag(d[13] | d[14] << 8) * MA_PER_COUNT)

    def torque(self, on):
        for _ in range(3):
            if self.w8(TORQUE, 1 if on else 0):
                return
        raise IOError("扭矩开关写入失败")


def state_or_die(servo):
    for _ in range(3):
        st = servo.state()
        if st is not None:
            return st
    raise IOError(f"ID {servo.id} 无应答")


def find_zero(servo):
    """扭矩关、摆杆自然下垂时的位置（1 s 平均）。晃动太大就拒绝。"""
    servo.torque(False)
    samples = []
    t0 = time.time()
    while time.time() - t0 < 1.0:
        samples.append(state_or_die(servo)["pos"])
    mean = sum(samples) / len(samples)
    spread = max(samples) - min(samples)
    return mean, spread


def record_one(servo, zero, kp, name, meta, logdir):
    traj = TRAJECTORIES[name]
    to_counts = lambda q: zero + q * COUNTS_PER_RAD
    servo.w8(KP, kp)

    # 1 s 预备：和 BAM 一样先停在轨迹起点
    q0, en = traj(0.0)
    servo.goal(to_counts(q0))
    servo.torque(en)
    torque_on = en
    time.sleep(1.0)

    entries = []
    start = time.time()
    while True:
        t = time.time() - start
        if t >= DURATION:
            break
        q, en = traj(t)
        if en != torque_on:
            if en:
                servo.goal(to_counts(q))
            servo.torque(en)
            torque_on = en
        if en:
            servo.goal(to_counts(q))
        t0 = time.time() - start
        st = servo.state()
        t1 = time.time() - start
        if st is None:
            continue
        if st["temp"] >= MAX_TEMP:
            raise RuntimeError(f"温度 {st['temp']}°C，停止")
        entries.append(dict(
            timestamp=(t0 + t1) / 2,
            position=(st["pos"] - zero) / COUNTS_PER_RAD,
            speed=st["speed"] * RAD_S_PER_SPEED,
            load=st["current"] / 1000.0,          # BAM 不用；这里放电流（A）备查
            input_volts=st["volts"],
            temp=float(st["temp"]),
            goal_position=q,
            torque_enable=en,
            current=st["current"] / 1000.0,
        ))

    # 慢慢回到 0（1 rad/s），再断扭矩
    st = state_or_die(servo)
    q = (st["pos"] - zero) / COUNTS_PER_RAD
    servo.goal(to_counts(q))
    servo.torque(True)
    while abs(q) > 1e-3:
        q = max(0.0, q - 0.01) if q > 0 else min(0.0, q + 0.01)
        servo.goal(to_counts(q))
        time.sleep(0.01)
    time.sleep(0.5)
    servo.torque(False)

    dt = [b["timestamp"] - a["timestamp"] for a, b in zip(entries, entries[1:])]
    rate = 1.0 / (sum(dt) / len(dt)) if dt else 0.0
    vin = sum(e["input_volts"] for e in entries) / max(1, len(entries))
    log = dict(meta, kp=kp, vin=round(vin, 2), trajectory=name, entries=entries)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%Mm%S")
    path = os.path.join(logdir, f"{stamp}_kp{kp}_{name}.json")
    json.dump(log, open(path, "w"))
    peak = max((abs(e["current"]) for e in entries), default=0)
    print(f"  {len(entries)} 点 / {rate:.0f} Hz，{vin:.1f} V，峰值电流 {peak:.2f} A -> {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--check", action="store_true", help="只读，不上扭矩")
    ap.add_argument("--mass", type=float, help="末端配重 kg")
    ap.add_argument("--arm-mass", type=float, help="摆杆自身 kg（含舵盘到配重之间的一切）")
    ap.add_argument("--length", type=float, help="轴心到配重中心 m")
    ap.add_argument("--logdir")
    ap.add_argument("--kps", default="8,16,24,32")
    ap.add_argument("--kd", type=int, default=8)
    ap.add_argument("--current-ma", type=float, default=1300.0)
    ap.add_argument("--trajectories", default="sin_time_square,sin_sin,up_and_down,lift_and_drop")
    a = ap.parse_args()

    servo = Servo(a.port, a.baud, a.id)
    st = servo.state()
    if st is None:
        sys.exit(f"{a.port} @ {a.baud} 上 ID {a.id} 无应答（波特率/ID 不对？用 set_baud.py --scan 看看）")

    mode = servo.read(MODE, 1)
    lim = servo.read(MIN_ANGLE, 4)
    print(f"ID {a.id}: 位置 {st['pos']}，{st['volts']:.1f} V，{st['temp']}°C，状态 0x{st['status']:02x}，"
          f"模式 {mode[0] if mode else '?'}，限位 {lim[0] | lim[1] << 8 if lim else '?'}"
          f"..{lim[2] | lim[3] << 8 if lim else '?'}")

    zero, spread = find_zero(servo)
    margin = MAX_EXCURSION * COUNTS_PER_RAD
    print(f"下垂 0 位 {zero:.0f}（1 s 内抖动 {spread} 计数）；需要 {zero - margin:.0f}..{zero + margin:.0f} 在 0..4095 内")
    ok_range = zero - margin >= 0 and zero + margin <= 4095
    if not ok_range:
        print("!! 0 位离中点太远，大幅轨迹会越过 0/4095。把舵盘相对舵机转一个角度重装，让下垂时读数接近 2048。")
    if spread > 10:
        print("!! 摆杆还在晃，等它停稳再测。")
    if a.check:
        return
    if not ok_range or spread > 10:
        sys.exit(1)
    if mode is None or mode[0] != 0:
        sys.exit("舵机不在位置模式（33 ≠ 0）")
    if None in (a.mass, a.arm_mass, a.length, a.logdir):
        sys.exit("录制需要 --mass --arm-mass --length --logdir")

    os.makedirs(a.logdir, exist_ok=True)
    kps = [int(k) for k in a.kps.split(",")]
    names = a.trajectories.split(",")
    for n in names:
        if n not in TRAJECTORIES:
            sys.exit(f"未知轨迹 {n}；可选 {', '.join(TRAJECTORIES)}")

    # 与机器人相同的 RAM 设置（掉电即失，每次都写）
    assert servo.w16(GOAL_CURR, int(round(a.current_ma / MA_PER_COUNT)))
    assert servo.w16(GOAL_SPEED, RUNNING_SPEED)
    assert servo.w8(ACCEL, 0)
    assert servo.w8(KD, a.kd)

    meta = dict(mass=a.mass, arm_mass=a.arm_mass, length=a.length, motor="hl2915",
                kd=a.kd, current_limit_ma=a.current_ma, zero_counts=zero, servo_id=a.id)
    try:
        for kp in kps:
            for n in names:
                print(f"Kp {kp}，{n}")
                record_one(servo, zero, kp, n, meta, a.logdir)
                time.sleep(3.0 if n == "lift_and_drop" else 1.0)
                # 每组之间重新确认 0 位没漂（舵盘打滑会让后面的数据全错）
                z, sp = find_zero(servo)
                if sp <= 10 and abs(z - zero) > 30:
                    raise RuntimeError(f"0 位从 {zero:.0f} 变成 {z:.0f}，舵盘或摆杆松了？")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        servo.torque(False)
        print("扭矩已关")


if __name__ == "__main__":
    main()
