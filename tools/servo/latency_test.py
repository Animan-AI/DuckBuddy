# -*- coding: utf-8 -*-
"""HL-2915 指令延迟：写目标位置（SYNC_WRITE，与 robotd 相同）到舵机响应要多久。

在摆杆台上做（摆杆下垂 = 0 位）：在 0 位保持，交替发 ±step 的阶跃，约 1 kHz 轮询
位置和电流，记下每次阶跃的
  - 电流起跳：|电流 - 阶跃前均值| > 60 mA（电机开始出力）
  - 开始动：位置离开起点 ≥ 3 计数（0.26°）
  - 到一半：走完 50%
时间从写指令那一刻算，已减去半个读周期（读回的是请求和应答之间的状态）。

用法（调试板 COM11，摆杆台；先关掉 FD 上位机）：
    python tools/servo/latency_test.py --kps 16,32 --out training/data/bam-hl2915/latency.json
"""
import argparse
import json
import statistics
import sys
import time

from bam_record import (ACCEL, COUNTS_PER_RAD, GOAL_CURR, GOAL_SPEED, KD, KP, MA_PER_COUNT,
                        RUNNING_SPEED, Servo, find_zero, state_or_die)


def step_once(servo, start, target, window):
    """发一次阶跃，返回 [(t, pos, current)]，t 相对写指令。"""
    base = []
    for _ in range(20):
        st = servo.state()
        if st:
            base.append(st["current"])
    t_write = time.perf_counter()
    servo.goal(target)
    trace = []
    while True:
        t0 = time.perf_counter()
        st = servo.state()
        t1 = time.perf_counter()
        if st:
            trace.append(((t0 + t1) / 2 - t_write, st["pos"], st["current"], t1 - t0))
        if t1 - t_write > window:
            break
    return statistics.mean(base) if base else 0.0, trace


def analyse(start, target, i_base, trace):
    sign = 1 if target > start else -1
    half = start + (target - start) / 2
    t_cur = t_move = t_half = None
    for t, pos, cur, _ in trace:
        if t_cur is None and abs(cur - i_base) > 60:
            t_cur = t
        if t_move is None and (pos - start) * sign >= 3:
            t_move = t
        if t_half is None and (pos - half) * sign >= 0:
            t_half = t
    rtt = statistics.median(r for *_, r in trace) if trace else 0.0
    return t_cur, t_move, t_half, rtt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--kps", default="16,32")
    ap.add_argument("--kd", type=int, default=8)
    ap.add_argument("--current-ma", type=float, default=1300.0)
    ap.add_argument("--step-deg", type=float, default=10.0)
    ap.add_argument("--reps", type=int, default=12, help="每个 Kp 的阶跃次数（正负交替）")
    ap.add_argument("--out")
    a = ap.parse_args()

    servo = Servo(a.port, a.baud, a.id)
    state_or_die(servo)
    zero, spread = find_zero(servo)
    if spread > 10:
        sys.exit("摆杆还在晃，等停稳")
    zero = round(zero)
    step = round(a.step_deg / 180 * 3.14159265 * COUNTS_PER_RAD)

    assert servo.w16(GOAL_CURR, int(round(a.current_ma / MA_PER_COUNT)))
    assert servo.w16(GOAL_SPEED, RUNNING_SPEED)
    assert servo.w8(ACCEL, 0)
    assert servo.w8(KD, a.kd)

    results = {}
    try:
        servo.goal(zero)
        servo.torque(True)
        for kp in [int(k) for k in a.kps.split(",")]:
            servo.w8(KP, kp)
            time.sleep(0.8)
            rows = []
            for r in range(a.reps):
                target = zero + (step if r % 2 == 0 else -step)
                st = state_or_die(servo)
                start = st["pos"]
                i_base, trace = step_once(servo, start, target, 0.25)
                t_cur, t_move, t_half, rtt = analyse(start, target, i_base, trace)
                rows.append(dict(start=start, target=target, t_current=t_cur, t_move=t_move,
                                 t_half=t_half, rtt=rtt,
                                 trace=[(round(t, 5), p, round(c, 1)) for t, p, c, _ in trace]))
                f = lambda x: f"{x * 1000:5.1f}" if x is not None else "  -  "
                print(f"Kp {kp:2d} #{r:2d} {start}->{target}: 电流起跳 {f(t_cur)} ms，开始动 {f(t_move)} ms，"
                      f"到一半 {f(t_half)} ms（读一次 {rtt * 1000:.2f} ms）")
                # 回 0 位再停稳
                servo.goal(zero)
                time.sleep(0.6)
            results[kp] = rows
            for key, name in (("t_current", "电流起跳"), ("t_move", "开始动"), ("t_half", "到一半")):
                v = [x[key] for x in rows if x[key] is not None]
                if v:
                    print(f"  Kp {kp} {name}: 中位 {statistics.median(v) * 1000:.1f} ms，"
                          f"范围 {min(v) * 1000:.1f}..{max(v) * 1000:.1f} ms（{len(v)}/{len(rows)}）")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        servo.torque(False)
        print("扭矩已关")

    if a.out and results:
        json.dump(dict(zero=zero, step_counts=step, kd=a.kd, current_limit_ma=a.current_ma,
                       baud=a.baud, results=results), open(a.out, "w"))
        print(f"-> {a.out}")


if __name__ == "__main__":
    main()
