# -*- coding: utf-8 -*-
"""飞特 FT-SCS 总线协议最小实现（HLS 系列）。

内存表：http://doc.feetech.cn/#/f?q=250689e628d7
注意 HLS 的表和 STS/SMS **不一样**，最坑的是 44 号地址：
HLS 的 44 是「目标电流」= 电机最大运行电流，为 0 时舵机完全没有扭矩。
"""
import time
import serial

# ---- 地址 ----------------------------------------------------------------
ID_ADDR     = 5      # 1B, EEPROM 主ID
BAUD_ADDR   = 6      # 1B, EEPROM 0..7 = 1M/500k/250k/128k/115200/76800/57600/38400
MIN_ANGLE   = 9      # 2B, EEPROM. 多圈绝对位置控制时与 MAX_ANGLE 都写 0
MAX_ANGLE   = 11     # 2B, EEPROM
PROT_CURR   = 28     # 2B, EEPROM 保护电流, 6.5mA/LSB
MODE        = 33     # 1B, EEPROM 0=位置伺服 1=恒速 2=恒流 3=PWM开环
TORQUE      = 40     # 1B 0=关 1=开 2=阻尼
ACCEL       = 41     # 1B, 8.7 度/秒^2, 0=最大加速度
GOAL_POS    = 42     # 2B, BIT15 为方向位
GOAL_CURR   = 44     # 2B, 6.5mA/LSB —— 为 0 则电机无扭矩
GOAL_SPEED  = 46     # 2B, 0.732RPM/LSB —— 为 0 表示停止运行
TORQUE_LIM  = 48     # 2B, 0.1%
LOCK        = 55     # 1B 0=解锁(EEPROM 掉电保存) 1=上锁(EEPROM 掉电不保存)
PRES_POS    = 56     # 2B, 只读, BIT15 为方向位
PRES_VOLT   = 62     # 1B, 只读, 0.1V
PRES_TEMP   = 63     # 1B, 只读, °C
STATUS      = 65     # 1B, 只读, 位=1 表示对应异常
PRES_CURR   = 69     # 2B, 只读, 6.5mA/LSB

BROADCAST = 0xFE
CENTER    = 2048     # 单圈量程 0..4095 的中点


def smag(v):
    """符号位+数值 -> 有符号整数。飞特用 BIT15 做方向位，不是二进制补码。"""
    return -(v & 0x7FFF) if v & 0x8000 else v


def enc(v):
    """有符号整数 -> 符号位+数值。"""
    return (0x8000 | abs(int(v))) if v < 0 else int(v)


class Bus:
    def __init__(self, port='COM11', baud=1000000, timeout=0.03):
        self.s = serial.Serial(port, baud, timeout=timeout, write_timeout=0.5)

    def close(self):
        self.s.close()

    def _txrx(self, sid, inst, params=()):
        body = [sid, len(params) + 2, inst] + list(params)
        self.s.reset_input_buffer()
        self.s.write(bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))
        self.s.flush()
        time.sleep(0.003)
        hdr = self.s.read(4)
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF or hdr[2] != sid:
            return None, None
        rest = self.s.read(hdr[3])
        if len(rest) < hdr[3]:
            return None, None
        return rest[0], rest[1:-1]

    def ping(self, sid):
        err, _ = self._txrx(sid, 0x01)
        return err

    def read(self, sid, addr, n):
        _, d = self._txrx(sid, 0x02, (addr, n))
        return d

    def write(self, sid, addr, data):
        return self._txrx(sid, 0x03, [addr] + list(data))

    # ---- 便捷封装 --------------------------------------------------------
    def r8(self, sid, addr):
        d = self.read(sid, addr, 1)
        return None if d is None else d[0]

    def r16(self, sid, addr):
        d = self.read(sid, addr, 2)
        return None if d is None else (d[0] | (d[1] << 8))

    def w8(self, sid, addr, v):
        return self.write(sid, addr, [v & 0xFF])

    def w16(self, sid, addr, v):
        v = int(v) & 0xFFFF
        return self.write(sid, addr, [v & 0xFF, (v >> 8) & 0xFF])

    def scan(self, lo=0, hi=253, tries=2):
        """返回能给出合法 PING 应答的 ID 列表。"""
        out = []
        for sid in range(lo, hi + 1):
            for _ in range(tries):
                if self.ping(sid) is not None:
                    out.append(sid)
                    break
        return out

    def pos(self, sid):
        v = self.r16(sid, PRES_POS)
        return None if v is None else smag(v)
