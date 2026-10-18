# -*- coding: utf-8 -*-
"""把 MJCF 里的舵机质量从 XL330(18g) 改成 HL-2915-C001(实测 28g)，重算每个连杆的
质量 / 质心 / 惯量张量；再把舵机以外的质量整体缩放，使整机等于实测总重。

实测（2026-10-05，含 3S 1500mAh 电池 119.6g）：整机 850g，单只舵机 28g。
只有总重、没有分件重量，所以非舵机部分（打印件、电子板、电池、线束）按同一比例
缩放：各连杆非舵机部分的质心不动，惯量随质量等比。拿到分件重量后改成逐连杆设定。

质量与质心为精确计算；舵机的自身惯量用等效各向同性近似（34x20x23mm 均质长方体
三个主惯量的平均），因为 geom 的朝向各不相同而最终仍要以实测为准。平行轴项是精确的。
"""
import re, os

# the original Microduck model (hardware/original/mjcf, from Pollen's repository) in, this directory out
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(OUT_DIR, "..", "..", "original", "mjcf")
FILES = ["robot_walk.xml", "robot_allcollisions.xml", "robot_allcollisions_rollers.xml"]

M_OLD = 0.018           # XL330, kg
M_NEW = 0.028           # HL-2915-C001，实测, kg
MEASURED_TOTAL = 0.850  # 整机实测（含电池）, kg
# 34 x 20 x 23 mm 均质长方体，单位质量时三个主惯量的平均值
A, B, C = 0.034, 0.020, 0.023
I_UNIT = ((B*B+C*C) + (A*A+C*C) + (A*A+B*B)) / 3 / 12

INERTIAL = re.compile(r'<inertial pos="([^"]+)" mass="([^"]+)" fullinertia="([^"]+)"\s*/>')


def nums(s):
    return [float(x) for x in s.split()]


def pa(m, r):
    """点质量 m 在相对参考点 r 处的惯量张量贡献，返回 (ixx,iyy,izz,ixy,ixz,iyz)。"""
    x, y, z = r
    return (m*(y*y+z*z), m*(x*x+z*z), m*(x*x+y*y), -m*x*y, -m*x*z, -m*y*z)


def servo_positions(blk):
    """本 body 直属的 xl330 geom 位置（嵌套 body 已被切到别的块里）。"""
    out = []
    for m in re.finditer(r'<!-- Part xl330[_0-9]* -->(.*?)(?=<!-- Part |\Z)', blk, re.S):
        g = re.search(r'<geom[^>]*pos="([^"]+)"', m.group(1))
        out.append(nums(g.group(1)) if g else [0.0, 0.0, 0.0])
    return out


def blocks(t):
    """按 <body 边界切块：每个 <body ...> 到下一个 <body 之间，含其 inertial 与 Part 注释。"""
    return re.split(r'(?=<body )', t)


def rest_scale(t, total=MEASURED_TOTAL):
    """非舵机部分的缩放系数，及舵机数、原非舵机总质量。"""
    rest, n = 0.0, 0
    for blk in blocks(t):
        mi = INERTIAL.search(blk) if blk.startswith("<body ") else None
        if mi:
            k = len(servo_positions(blk))
            rest += float(mi.group(2)) - k*M_OLD
            n += k
    return (total - n*M_NEW) / rest, n, rest


def total_mass(t):
    return sum(float(INERTIAL.search(b).group(2))
               for b in blocks(t) if b.startswith("<body ") and INERTIAL.search(b))


def process(path, out, total=MEASURED_TOTAL):
    """path 的 XL330 换成 HL-2915、其余按比例缩放到 total，写到 out，返回逐连杆报告。"""
    t = open(path, encoding="utf-8").read()
    S, _, _ = rest_scale(t, total)
    report = []
    new_parts = []
    for blk in blocks(t):
        mb = re.match(r'<body name="([a-z_0-9]+)"', blk)
        mi = INERTIAL.search(blk) if mb else None
        if not mi:
            new_parts.append(blk)
            continue
        name = mb.group(1)
        servos = servo_positions(blk)
        n = len(servos)

        c0 = nums(mi.group(1))
        m0 = float(mi.group(2))
        I0 = nums(mi.group(3))          # ixx iyy izz ixy ixz iyz，关于 c0

        # 1) 剥掉 XL330，得到"其余部分"的质量 / 质心 / 惯量（关于 cr）
        mr = m0 - n*M_OLD
        cr = [(m0*c0[k] - M_OLD*sum(p[k] for p in servos)) / mr for k in range(3)]
        Ir = list(I0)
        for p in servos:
            a = pa(M_OLD, [p[k]-c0[k] for k in range(3)])
            for k in range(6):
                Ir[k] -= a[k] + (M_OLD*I_UNIT if k < 3 else 0.0)
        a = pa(mr, [cr[k]-c0[k] for k in range(3)])
        Ir = [Ir[k] - a[k] for k in range(6)]

        # 2) 其余部分按实测总重缩放
        mr, Ir = mr*S, [x*S for x in Ir]

        # 3) 装上 HL-2915，合成到新质心 c1
        m1 = mr + n*M_NEW
        c1 = [(mr*cr[k] + M_NEW*sum(p[k] for p in servos)) / m1 for k in range(3)]
        a = pa(mr, [cr[k]-c1[k] for k in range(3)])
        I1 = [Ir[k] + a[k] for k in range(6)]
        for p in servos:
            a = pa(M_NEW, [p[k]-c1[k] for k in range(3)])
            for k in range(6):
                I1[k] += a[k] + (M_NEW*I_UNIT if k < 3 else 0.0)

        new = ('<inertial pos="{:.7g} {:.7g} {:.7g}" mass="{:.6g}" '
               'fullinertia="{:.6g} {:.6g} {:.6g} {:.6g} {:.6g} {:.6g}"/>').format(
            c1[0], c1[1], c1[2], m1, *I1)
        blk = blk.replace(mi.group(0), new, 1)
        report.append((name, n, m0*1000, m1*1000, sum((c0[k]-c1[k])**2 for k in range(3))**0.5*1000))
        new_parts.append(blk)

    open(out, "w", encoding="utf-8").write("".join(new_parts))
    return report


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    for f in FILES:
        rep = process(os.path.join(SRC_DIR, f), os.path.join(OUT_DIR, f))
        if f == FILES[0]:
            print(f"{'连杆':<20}{'舵机':>5}{'原质量g':>10}{'新质量g':>10}{'质心移动mm':>12}")
            for n, k, a, b, d in rep:
                print(f"{n:<20}{k:>5}{a:>10.1f}{b:>10.1f}{d:>12.2f}")
            print()
        print(f"{f}: 更新 {len(rep)} 个连杆")

    t = open(os.path.join(SRC_DIR, FILES[0]), encoding="utf-8").read()
    S, n, rest = rest_scale(t)
    new_total = total_mass(open(os.path.join(OUT_DIR, FILES[0]), encoding="utf-8").read())
    print(f"\n舵机 {n} 只 x {M_NEW*1000:.1f} g；非舵机部分 {rest*1000:.1f} g x {S:.3f} = {rest*S*1000:.1f} g")
    print(f"整机质量: {total_mass(t)*1000:.1f} g -> {new_total*1000:.1f} g（实测 {MEASURED_TOTAL*1000:.0f} g）")
