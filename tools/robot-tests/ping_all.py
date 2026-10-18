# Per-servo bus ping (robotd must be stopped): N single reads of present position from each ID, no
# retries; plus voltage (62), temperature (63) and status (65) per servo.
import os, sys, termios, time, select
fd = os.open("/dev/ttyS7", os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
a[4] = a[5] = termios.B500000; a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)
def txrx(sid, inst, params, timeout=0.01):
    body = [sid, len(params) + 2, inst] + params
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]))
    buf = b""; t0 = time.time()
    while time.time() - t0 < timeout:
        r, _, _ = select.select([fd], [], [], 0.002)
        if r: buf += os.read(fd, 256)
        i = buf.find(bytes([0xFF, 0xFF, sid]))
        if i >= 0 and len(buf) >= i + 4 and len(buf) >= i + 4 + buf[i + 3]:
            pkt = buf[i + 2:i + 4 + buf[i + 3]]
            if (~sum(pkt[:-1])) & 0xFF != pkt[-1]: return "bad"
            return buf[i + 5:i + 4 + buf[i + 3] - 1]
    return None
N = int(sys.argv[1]) if len(sys.argv) > 1 else 300
ids = list(range(1, 16)) + [200]
fail = {i: 0 for i in ids}; bad = {i: 0 for i in ids}
t0 = time.time()
for _ in range(N):
    for i in ids:
        # the IMU board (ID 200) answers a plain READ slower than the servos: at 10 ms it read as
        # 18-41 % lost (2026-10-17) while robotd's sync_read, where it answers last, lost none
        d = txrx(i, 2, [56, 2], 0.03 if i == 200 else 0.01)
        if d is None: fail[i] += 1
        elif d == "bad": bad[i] += 1
print(f"{N} rounds in {time.time() - t0:.1f} s")
for i in ids:
    extra = ""
    if i != 200:
        v = txrx(i, 2, [62, 4], 0.03)
        if isinstance(v, bytes) and len(v) == 4: extra = f"  {v[0] / 10:.1f} V  {v[1]} C  status 0x{v[3]:02x}"
    print(f"ID{i:3d}  no reply {fail[i]:3d}/{N}  bad checksum {bad[i]}{extra}")
