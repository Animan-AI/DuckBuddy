# -*- coding: utf-8 -*-
"""飞特 HLS 舵机链式配置向导：不用每只都断开重连。

出厂 15 只全是 ID=1，同一条总线上多只同 ID 会一起应答、互相覆盖，没法定向寻址。
但只要保证**任何时刻只有新接的那一只顶着 ID 1**，就能一直串下去：

    第 1 只单独接 -> ID 1 唯一 -> 改成 15
    第 2 只串上去 -> 链上 {15, 1}，ID 1 仍唯一 -> 改成 14
    ...
    第 15 只串上去 -> ID 1 唯一，且目标就是 1 -> 不用改

所以 ID 从高到低倒着分配，最后一只天然保留 ID 1。沿线束是 15->14->...->1 单调排列。

**每次运行都从头走一遍全部 --count 只**（不做断点续传）：
    - 已经顶着目标 ID 的舵机 -> 原地重新配置（限位/模式/电流/回中全重做），不用拔；
    - 目标 ID 空缺 -> 等你把新的串到链尾，改 ID。
这样中断了重跑、或者某只的回中当时跳过了，直接再跑一遍即可，之前做过的不会撞号。

电流安全：1.25mm 端子(JST GH/SH) 每触点额定约 1A，舵机默认目标电流 450 LSB
= 2.93A。本工具默认压到 100 LSB = 0.65A，且任何时刻只给正在配置的那一只上扭矩，
其余全部保持扭矩关闭，总电流始终是一只的量。

用法:
    python setup_ids.py --port COM11 --count 15
    python setup_ids.py --port COM11 --verify          # 全部接完后核对
    python setup_ids.py --port COM11 --center 14,15    # 只补回中
    python setup_ids.py --port COM11 --isolated        # 老的逐只单独接模式
"""
import argparse
import io
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8',
                              errors='replace', line_buffering=True)
import fts_bus as F

FRESH_ID = 1        # 出厂 ID，新接上的那只一定顶着它


def clean_ping(bus, sid, tries=6):
    """连续 tries 次都拿到合法应答才算数。

    多只同 ID 撞车时偶尔也能凑出一个格式正确的包，单次 ping 不可靠；
    连续多次全中的概率极低，用它来区分「唯一一只」和「撞车」。
    """
    for _ in range(tries):
        if bus.ping(sid) is None:
            return False
    return True


def raw_activity(bus, sid=FRESH_ID):
    """线上有没有任何字节回来（撞车时是碎片，也算有）。"""
    for _ in range(3):
        bus.s.reset_input_buffer()
        body = [sid, 0x02, 0x01]
        bus.s.write(bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))
        bus.s.flush()
        time.sleep(0.02)
        if bus.s.read(32):
            return True
    return False


def wait_for_new(bus, assigned, target, prompt):
    """等「下一只」上线。两种都算到货，返回是哪种：

    'target' —— 目标 ID 上出现了舵机（接的是**已改过号**的那只，比如中途重跑、
                或者接得比循环进来晚），原地重配，不改号；
    'fresh'  —— ID 1 上出现唯一应答的新舵机，走改号流程。
    """
    while True:
        input(prompt)
        gone = [i for i in assigned if not clean_ping(bus, i, tries=3)]
        if gone:
            print('  !! 之前配好的 ID %s 现在不应答了 —— 链路某处松了，先查线。' % gone)
            continue
        if clean_ping(bus, target, tries=4):
            print('  ID %d 上有舵机应答 —— 是已改过号的这只，原地重新配置。' % target)
            return 'target'
        if clean_ping(bus, FRESH_ID):
            return 'fresh'
        if raw_activity(bus):
            print('  !! ID %d 上有多只舵机同时应答（回包是碎片）。' % FRESH_ID)
            print('     一次只能新接一只——把多接的那只拔掉。')
        else:
            where = ('ID %d 上没有应答。' % target if target == FRESH_ID else
                     'ID %d 和 ID 1 上都没有应答。' % target)
            print('  !! %s' % where)
            print('     检查：电源、3 芯线方向、插到位没；接好后**再按一次回车**重新检测。')
            print('     （接的是已改过号的舵机也没问题，它一上线我就会认出来）')


def configure(bus, cur_id, new_id, args):
    if cur_id != new_id:
        print('  改 ID: %d -> %d' % (cur_id, new_id))
        bus.w8(cur_id, F.LOCK, 0)          # 解锁，EEPROM 写入才掉电保存
        time.sleep(0.05)
        bus.w8(cur_id, F.ID_ADDR, new_id)
        time.sleep(0.08)
        if not clean_ping(bus, new_id, tries=4):
            print('  !! 改完 ping 不到 ID %d' % new_id)
            return False
        if clean_ping(bus, FRESH_ID, tries=4):
            print('  !! ID %d 上还有舵机在应答 —— 说明当时接了不止一只，'
                  '这次改动可能落到了别的舵机上。' % FRESH_ID)
            return False
        bus.w8(new_id, F.LOCK, 1)
        time.sleep(0.05)
        print('     OK')
    else:
        print('  目标就是 ID %d，不用改' % new_id)

    bus.w8(new_id, F.LOCK, 0)
    time.sleep(0.05)
    bus.w16(new_id, F.MIN_ANGLE, 0)
    time.sleep(0.03)
    bus.w16(new_id, F.MAX_ANGLE, 0 if args.multiturn else 4095)
    time.sleep(0.03)
    bus.w8(new_id, F.MODE, 0)              # 位置伺服模式
    time.sleep(0.03)
    bus.w8(new_id, F.LOCK, 1)
    time.sleep(0.05)

    # 下面三个都在 SRAM，掉电即失，机器人固件每次上电都要重写
    bus.w16(new_id, F.GOAL_CURR, args.current)
    time.sleep(0.03)
    bus.w16(new_id, F.GOAL_SPEED, args.speed)
    time.sleep(0.03)
    bus.w8(new_id, F.ACCEL, args.accel)
    time.sleep(0.03)

    gc = bus.r16(new_id, F.GOAL_CURR)
    stat = bus.r8(new_id, F.STATUS)
    print('  限位=%d/%d  模式=%d  目标电流=%d(%.2fA)  速度=%d  电压=%.1fV  状态=%d'
          % (bus.r16(new_id, F.MIN_ANGLE), bus.r16(new_id, F.MAX_ANGLE),
             bus.r8(new_id, F.MODE), gc, gc * 0.0065,
             F.smag(bus.r16(new_id, F.GOAL_SPEED)),
             bus.r8(new_id, F.PRES_VOLT) / 10.0, stat))
    if stat:
        print('  !! 状态位非 0（BIT0电压 BIT1磁编码 BIT2温度 BIT3电流）')
    return True


def ask_yes_no(prompt):
    """必须明确回答 y 或 n。直接回车不算——静默跳过会让人以为动作执行了。"""
    while True:
        a = input(prompt).strip().lower()
        if a in ('y', 'yes'):
            return True
        if a in ('n', 'no'):
            return False
        print('    请输入 y 或 n（直接回车不算）')


def goto_center(bus, sid, args):
    """开到中位并保持扭矩，等舵机臂装好再松。只有这一只会通电。

    目标取「离当前位置最近的 2048+4096k」而不是裸 2048：位置反馈是多圈计数时，
    裸 2048 可能让舵机绕好几圈。全程监督：误差从最近点又拉大 1000 counts、或电流
    连续 3 次超 300 LSB，立即切扭矩——失控的舵机 0.1 秒内被摁停，而不是转满
    10 秒还一边转一边等人按回车（ID 4 那次就是这样，2.44A 转了十来秒）。
    """
    # 防御：这只若刚被断电过（掉电验证、或大电流自复位），44/46 已回上电默认
    # 450/250，不补写的话回中会在 2.92A 限流下跑——ID 13 就这样中过招。
    bus.w16(sid, F.GOAL_CURR, args.current)
    time.sleep(0.03)
    bus.w16(sid, F.GOAL_SPEED, args.speed)
    time.sleep(0.03)
    p = bus.pos(sid)
    target = F.CENTER + 4096 * round((p - F.CENTER) / 4096.0)
    print('  当前位置 = %d，目标 = %d（同圈最近的中位），要走 %d counts (%.1f°)'
          % (p, target, target - p, abs(target - p) * 0.087))
    if not ask_yes_no('  开到中位并保持扭矩？舵机臂/连杆已装的话会带着关节走。'
                      '输入 y 或 n: '):
        print('  跳过，本只不通电')
        return
    bus.w16(sid, F.GOAL_POS, F.enc(p))     # 先把目标设成当前位置，避免上扭矩瞬间跳
    time.sleep(0.03)
    bus.w8(sid, F.TORQUE, 1)
    time.sleep(0.05)
    bus.w16(sid, F.GOAL_POS, F.enc(target))

    ok = False; best = None; bad_cur = 0; i = 0; t0 = time.time()
    while time.time() - t0 < 10:
        time.sleep(0.1)
        i += 1
        p = bus.pos(sid)
        cur = F.smag(bus.r16(sid, F.PRES_CURR))
        err = abs(p - target)
        if best is None or err < best:
            best = err
        if err <= 3:
            ok = True
            break
        if err > best + 1000:
            print('    !! 偏离目标 %d counts —— 切扭矩防失控' % err)
            break
        if abs(cur) > 300:
            bad_cur += 1
            if bad_cur >= 3:
                print('    !! 电流持续 %.2f A —— 切扭矩' % (abs(cur) * 0.0065))
                break
        else:
            bad_cur = 0
        if i % 3 == 0:
            print('    pos=%6d  电流=%5d (%.2f A)' % (p, cur, abs(cur) * 0.0065))

    if ok:
        print('    到位 pos=%d，保持扭矩' % p)
        input('  把舵机臂按中立位装上，装好后按回车松扭矩... ')
    else:
        print('  !! 没到位（最后 pos=%d）—— 这只先别装臂，人工查一下' % p)
    bus.w8(sid, F.TORQUE, 0)
    print('  扭矩已松开。')


def verify(bus, count):
    ids = [i for i in range(0, 41) if clean_ping(bus, i, tries=3)]
    print('总线上应答的 ID (%d 个): %s' % (len(ids), sorted(ids)))
    missing = [i for i in range(1, count + 1) if i not in ids]
    extra = [i for i in ids if i < 1 or i > count]
    if missing:
        print('缺少: %s' % missing)
    if extra:
        print('意外的 ID: %s' % extra)
    if not ids:
        print('（无应答。若原始回包是碎片，说明仍有同 ID 冲突）')
        return
    print()
    print(' ID | 位置  | 电压  | 温度 | 目标电流 | 速度 | 限位      | 模式 | 状态')
    print('----+-------+-------+------+----------+------+-----------+------+-----')
    for sid in sorted(ids):
        print(' %2d | %5d | %4.1fV | %3d  |  %5d   | %4d | %4d/%-4d | %d    | %d'
              % (sid, bus.pos(sid), bus.r8(sid, F.PRES_VOLT) / 10.0,
                 bus.r8(sid, F.PRES_TEMP), bus.r16(sid, F.GOAL_CURR),
                 F.smag(bus.r16(sid, F.GOAL_SPEED)),
                 bus.r16(sid, F.MIN_ANGLE), bus.r16(sid, F.MAX_ANGLE),
                 bus.r8(sid, F.MODE), bus.r8(sid, F.STATUS)))
    if not missing and not extra:
        print()
        print('%d 只全部就位，ID 无冲突。' % count)


def run_center(bus, args):
    """只做「开到中位、装舵机臂」这一步，给已经配好 ID 的舵机补做。"""
    if args.center.strip().lower() == 'all':
        ids = [i for i in range(0, 41) if clean_ping(bus, i, tries=3)]
    else:
        ids = [int(x) for x in args.center.replace('，', ',').split(',') if x.strip()]
    print('要回中的 ID: %s' % ids)
    print('目标电流 %d LSB = %.2f A，一次只给一只上扭矩。'
          % (args.current, args.current * 0.0065))
    for sid in ids:
        print()
        print('--- ID %d ---' % sid)
        if not clean_ping(bus, sid, tries=4):
            print('  !! ping 不到 ID %d，跳过' % sid)
            continue
        # 这三个都在 SRAM，掉电即失，先补上再动
        bus.w16(sid, F.GOAL_CURR, args.current)
        time.sleep(0.03)
        bus.w16(sid, F.GOAL_SPEED, args.speed)
        time.sleep(0.03)
        bus.w8(sid, F.ACCEL, args.accel)
        time.sleep(0.03)
        goto_center(bus, sid, args)
    return 0


def run_chain(bus, args):
    print('=' * 66)
    print('飞特 HLS 舵机链式配置   端口=%s   共 %d 只' % (args.port, args.count))
    print('每次运行都从头走一遍：ID 从 %d 倒着分到 1。' % args.count)
    print('已顶着目标 ID 的原地重新配置（含回中），不用拔；')
    print('缺哪只，就把新的串到链尾。任何时刻只有一只上扭矩。')
    print('目标电流 %d LSB = %.2f A。' % (args.current, args.current * 0.0065))
    print('建议断掉舵机电源再串下一只：1.25mm 端子热插拔有浪涌拉弧。')
    print('=' * 66)

    # 开机扫一遍：ID 2..40 范围内在的都算「链上已有」。ID 1 可能是刚接的新舵机，
    # 不算「已配好」。
    present = [i for i in range(2, 41)
               if clean_ping(bus, i, tries=3)]
    stray = [i for i in present if i > args.count]
    if stray:
        print()
        print('!! 链上有超出 --count=%d 的 ID: %s' % (args.count, stray))
        print('   --count 设得不对，或者混进了别的机器人的舵机。先搞清楚再继续。')
        return 1
    configured = [i for i in present if i <= args.count]
    fresh_now = clean_ping(bus, FRESH_ID, tries=3)
    if configured or fresh_now:
        print()
        print('开机检测到: 已配好 %s%s'
              % (sorted(configured, reverse=True) or '无',
                 '，另有一只出厂 ID 1 的新舵机' if fresh_now else ''))
        if configured:
            print('它们会被原地重做——限位/模式/电流/回中全部重新走一遍。')

    # 每次运行的**第一次新分配 ID**都做一次掉电验证：ID 存在 EEPROM，但
    # 「锁标志」没解锁时写入是存不住的，不验证一把就批量做，可能 15 只全白做。
    assigned_here = 0
    assigned = list(configured)

    for n, target in enumerate(range(args.count, 0, -1)):
        print()
        print('--- 第 %d/%d 只  ->  ID %d ---' % (n + 1, args.count, target))
        if clean_ping(bus, target, tries=4):
            print('  链上已有舵机顶着 ID %d，原地重新配置，不改 ID。' % target)
            ok = configure(bus, target, target, args)
        else:
            if not assigned:
                tip = '  只接这一只，接好后按回车（Ctrl+C 退出）... '
            else:
                tip = ('  把下一只串到链尾（已配的 %d 只保持接着），'
                       '接好后按回车... ' % len(assigned))
            mode = wait_for_new(bus, assigned, target, tip)
            if mode == 'target':
                ok = configure(bus, target, target, args)
            else:
                ok = configure(bus, FRESH_ID, target, args)
                if ok:
                    assigned_here += 1
                    if assigned_here == 1:
                        print()
                        print('  >>> 第一只务必断电再上电，确认 ID 真的存住了 <<<')
                        input('  断电重新上电后按回车验证... ')
                        if clean_ping(bus, target, tries=4):
                            print('  ID %d 掉电后仍在，EEPROM 保存正常，继续。' % target)
                        else:
                            print('  !! 掉电后 ping 不到 ID %d —— EEPROM 没保存住。'
                                  % target)
                            print('     先解决这个，别白做剩下 %d 只。' % (args.count - 1))
                            return 1
        if not ok:
            print('  这只失败了，检查后重跑即可（已配好的不会撞号）。')
            continue
        goto_center(bus, target, args)
        if target not in assigned:
            assigned.append(target)
        print('  这只完成。链上已配: %s' % sorted(assigned, reverse=True))

    print()
    print('全部走完。核对：')
    print('    python setup_ids.py --port %s --count %d --verify'
          % (args.port, args.count))
    return 0


def run_isolated(bus, args):
    """老模式：每只单独接，ID 从 start-id 递增。"""
    print('逐只单独接模式：ID 从 %d 递增到 %d，每只都要拔掉上一只。'
          % (args.start_id, args.start_id + args.count - 1))
    assigned = []
    for n in range(args.count):
        target = args.start_id + n
        print()
        print('--- 第 %d/%d 只  ->  ID %d ---' % (n + 1, args.count, target))
        while True:
            input('  只接这一只（其余全部拔掉），按回车... ')
            if clean_ping(bus, FRESH_ID):
                break
            print('  !! ID %d 上没有唯一应答。' % FRESH_ID)
        if not configure(bus, FRESH_ID, target, args):
            continue
        goto_center(bus, target, args)
        assigned.append(target)
        print('  拔掉这只，接下一只。')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM11')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--count', type=int, default=15)
    ap.add_argument('--start-id', type=int, default=1,
                    help='仅 --isolated 模式用')
    ap.add_argument('--current', type=int, default=100,
                    help='目标电流 LSB (6.5mA/LSB)，默认 100 = 0.65A')
    ap.add_argument('--speed', type=int, default=100,
                    help='运行速度 LSB (0.732RPM/LSB)，默认 100')
    ap.add_argument('--accel', type=int, default=30)
    ap.add_argument('--multiturn', action='store_true',
                    help='设成多圈绝对位置(限位 0/0)，默认单圈 0..4095')
    ap.add_argument('--isolated', action='store_true',
                    help='老模式：每只单独接、ID 递增')
    ap.add_argument('--center', metavar='IDS',
                    help='只把指定 ID 开到中位装舵机臂，不改 ID。'
                         '如 --center 14,15 或 --center all')
    ap.add_argument('--verify', action='store_true')
    args = ap.parse_args()

    try:
        bus = F.Bus(args.port, args.baud)
    except Exception as e:
        print('串口打开失败: %s' % e)
        print('(FD 上位机开着会独占串口，先关掉)')
        return 1

    try:
        if args.verify:
            verify(bus, args.count)
            return 0
        if args.center:
            return run_center(bus, args)
        return run_isolated(bus, args) if args.isolated else run_chain(bus, args)
    except KeyboardInterrupt:
        print('\n中断，广播松开所有扭矩...')
        try:
            bus.w8(F.BROADCAST, F.TORQUE, 0)
        except Exception:
            pass
        return 1
    finally:
        bus.close()


if __name__ == '__main__':
    sys.exit(main())
