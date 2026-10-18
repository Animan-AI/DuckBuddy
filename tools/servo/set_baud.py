# -*- coding: utf-8 -*-
"""把总线上的飞特 HLS 舵机改成新波特率（写 EEPROM，掉电保存）。

Orange Pi A733 的 UART7 只能准确输出 500k（1M 实际是 750k），所以舵机总线统一用 500k。

用法（先关掉 FD 上位机，它会独占串口）：
    python tools/servo/set_baud.py                 # 默认 COM11，只允许总线上有 1 只舵机
    python tools/servo/set_baud.py --all           # 总线上所有舵机一起改（整链）
    python tools/servo/set_baud.py --to 1000000    # 改回 1M
    python tools/servo/set_baud.py --scan          # 只扫描，不改

只写 55（锁标志）和 6（波特率）两个地址，不碰扭矩/位置，舵机不会动。
"""
import argparse
import sys
import time

from fts_bus import Bus, LOCK, BAUD_ADDR

BAUDS = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]  # 寄存器 6 的编号 0..7


def scan_all(port, bauds):
    """在每个波特率下扫一遍，返回 {baud: [ids]}（只列有舵机的）。"""
    found = {}
    for b in bauds:
        bus = Bus(port, b)
        try:
            ids = bus.scan(0, 253, tries=1)
        finally:
            bus.close()
        if ids:
            found[b] = ids
        print(f"  {b:>7} bps: {ids if ids else '-'}")
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--to", type=int, default=500000, choices=BAUDS)
    ap.add_argument("--all", action="store_true", help="允许一次改多只")
    ap.add_argument("--scan", action="store_true", help="只扫描")
    a = ap.parse_args()

    print(f"扫描 {a.port} ...")
    found = scan_all(a.port, BAUDS)
    if a.scan:
        return 0
    if not found:
        print("没找到舵机：检查供电、线序，以及 FD 上位机是否还开着。")
        return 1

    old = {(b, i) for b, ids in found.items() for i in ids}
    ids = sorted({i for _, i in old})
    if len(ids) != len(old):
        print("同一个 ID 出现在多个波特率下，请一只一只改。")
        return 1
    if len(ids) > 1 and not a.all:
        print(f"总线上有 {len(ids)} 只舵机 {ids}；整链改请加 --all。")
        return 1

    idx = BAUDS.index(a.to)
    todo = [(b, i) for b, i in sorted(old) if b != a.to]
    for b, i in todo:
        bus = Bus(a.port, b)
        try:
            bus.w8(i, LOCK, 0)          # 解锁 EEPROM
            time.sleep(0.02)
            bus.w8(i, BAUD_ADDR, idx)   # 写波特率编号
            time.sleep(0.05)
        finally:
            bus.close()
        print(f"  ID {i}: {b} -> {a.to} 已写入")

    # 新波特率下逐只校验并上锁（含之前已是目标波特率、上次没来得及上锁的）；
    # 个别型号要重新上电才切换，给出提示
    time.sleep(0.2)
    bus = Bus(a.port, a.to)
    bad = []
    try:
        for i in ids:
            if bus.ping(i) is None:
                bad.append(i)
                continue
            if bus.r8(i, LOCK) != 1:
                bus.w8(i, LOCK, 1)
                time.sleep(0.02)
            v = bus.r8(i, BAUD_ADDR)
            print(f"  ID {i}: {a.to} bps 应答正常，寄存器6={v}，锁={bus.r8(i, LOCK)}")
    finally:
        bus.close()

    if bad:
        print(f"ID {bad} 在 {a.to} bps 下没应答：给舵机断电重上电后再运行一次本脚本（会自动补上锁）。")
        return 2
    print("完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
