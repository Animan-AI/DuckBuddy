# -*- coding: utf-8 -*-
"""visiond: Xiaobai's eyes on the OPi (docs/project/vision-tof-design-20261017-CN.md).

Step one is QR Wi-Fi provisioning: show the duck your phone's "share hotspot" QR code and it joins
that network. lifed asks (`vision.scan_qr`) when it hears "小白，连网"; this runs the camera
(duckcam: vendor ISP for exposure, grey frames), decodes QR codes with OpenCV, and for a Wi-Fi code
(WIFI:T:WPA;S:name;P:secret;;) joins it through NetworkManager — the robot's user may since
2026-10-17 (/etc/polkit-1/rules.d/50-microduck-nm.rules). The outcome goes back to lifed as an
event; the password is never logged or sent anywhere.

A network joined this way is saved at autoconnect priority -10, under the home network's 0: at home
the duck goes back home, away it takes the hotspot.

    python3 visiond.py [--sock /tmp/visiond.sock] [--duckcam ~/visiond/duckcam]
    python3 visiond.py --scan 30 [--no-join]     # one scan from the command line
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import struct
import subprocess
import threading
import time
import urllib.request

LIFED = "/tmp/lifed.sock"
PHOTO = "/tmp/visiond-photo.jpg"          # the last photo (duckweb serves it)
LOOKS = "/tmp/visiond-looks"              # the photos of the last looks, for the phone page (RAM, gone at boot)
LOOKS_KEPT = 10
WATCH_SETTLE_S = 0.8                      # the ISP's exposure settling, before faces are looked for
HEADER = struct.Struct("<4sIIIqII")       # duckcam's: magic, w, h, sequence, t_ns, exposure_us, gain
JOIN_PRIORITY = -10


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


# ── Wi-Fi QR codes ──────────────────────────────────────────────────────────────────────────────

def _split_fields(body: str) -> dict:
    """`T:WPA;S:my\\;net;P:pw;;` -> {"T": "WPA", "S": "my;net", "P": "pw"} (\\ escapes ; , : \\ ")."""
    out, key, buf, i = {}, None, [], 0
    while i < len(body):
        c = body[i]
        if c == "\\" and i + 1 < len(body):
            buf.append(body[i + 1])
            i += 2
            continue
        if c == ":" and key is None:
            key, buf = "".join(buf), []
        elif c == ";":
            if key is not None:
                out[key.upper()] = "".join(buf)
            key, buf = None, []
        else:
            buf.append(c)
        i += 1
    if key is not None:
        out[key.upper()] = "".join(buf)
    return out


def parse_wifi_qr(text: str) -> dict | None:
    """A phone's Wi-Fi share code as {"ssid", "password", "security", "hidden"}; None if it is not one."""
    if not text or not text.upper().startswith("WIFI:"):
        return None
    f = _split_fields(text[5:])
    ssid = f.get("S", "")
    if not ssid:
        return None
    sec = (f.get("T") or "nopass").upper()
    if sec in ("", "NOPASS", "NONE"):
        sec = "nopass"
    return {"ssid": ssid, "password": f.get("P", ""), "security": sec,
            "hidden": f.get("H", "").lower() == "true"}


# ── Joining ─────────────────────────────────────────────────────────────────────────────────────

def nmcli(*args, timeout=40.0, secret: str | None = None):
    """Run nmcli; the secret (if any) is masked in what is logged."""
    shown = " ".join("***" if secret and a == secret else a for a in args)
    p = subprocess.run(["nmcli", *args], capture_output=True, text=True, timeout=timeout)
    if p.returncode != 0:
        err = (p.stderr or p.stdout).strip().replace(secret, "***") if secret else (p.stderr or p.stdout).strip()
        log(f"nmcli {shown}: exit {p.returncode}: {err[:200]}")
    return p


def current_ssid() -> str | None:
    """The Wi-Fi it is on now, or None."""
    try:
        p = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID", "device", "wifi", "list", "--rescan", "no"],
                           capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None
    for line in p.stdout.splitlines():
        if line.startswith("yes:"):
            return line[4:].replace("\\:", ":")
    return None


def split_terse(line: str) -> list[str]:
    """One line of `nmcli -t`: fields split on ':', with backslash escapes undone."""
    out, buf, i = [], [], 0
    while i < len(line):
        c = line[i]
        if c == "\\" and i + 1 < len(line):
            buf.append(line[i + 1])
            i += 2
            continue
        if c == ":":
            out.append("".join(buf))
            buf = []
        else:
            buf.append(c)
        i += 1
    out.append("".join(buf))
    return out


def ssid_on_air(ssid: str) -> str:
    """The name as it can be broadcast: an SSID is at most 32 bytes, and a phone with a longer name
    sends it cut short while its share code keeps it whole (2026-10-17: "风一样的男子，简称风子" is 33
    bytes in UTF-8, on the air "风一样的男子，简称风")."""
    return ssid.encode("utf-8")[:32].decode("utf-8", "ignore")


def match_network(ssid: str, visible: list[str]) -> str | None:
    """The visible network the code means: the same name, or the name cut to 32 bytes."""
    if ssid in visible:
        return ssid
    cut = ssid_on_air(ssid)
    if cut != ssid:
        for v in visible:
            if v and (v == cut or (cut.startswith(v) and len(v.encode("utf-8")) >= 28)):
                return v
    return None


def match_bytes(ssid: bytes, visible) -> bytes | None:
    """match_network on raw SSIDs: a cut-short name may end in half a character."""
    visible = list(visible)
    if ssid in visible:
        return ssid
    if len(ssid) > 32:
        cut = ssid[:32]
        for v in visible:
            if v and (v == cut or (cut.startswith(v) and len(v) >= 28)):
                return v
    return None


class AirWatch:
    """Which networks were on the air lately, rescanned every few seconds from the moment the duck
    is asked to connect. An iPhone shows its hotspot only while its settings page is open, and you
    leave that page to show the code (2026-10-17: seen right after a hotspot restart, gone when the
    duck tried to join) — so the duck remembers what it saw while you were getting the code ready."""

    def __init__(self):
        self.seen: dict[bytes, tuple[str, float]] = {}       # SSID -> (BSSID, last seen)
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None

    def scan_once(self):
        p = nmcli("-t", "-f", "SSID-HEX,BSSID", "device", "wifi", "list", "--rescan", "yes", timeout=20)
        now = time.monotonic()
        for line in p.stdout.splitlines():
            f = split_terse(line)
            if len(f) >= 2 and f[0]:
                try:
                    self.seen[bytes.fromhex(f[0])] = (f[1], now)
                except ValueError:
                    pass

    def start(self):
        def run():
            while not self.stop.is_set():
                try:
                    self.scan_once()
                except Exception as e:  # noqa: BLE001 - a failed scan is just a missed look
                    log(f"wifi scan: {e!r}")
                self.stop.wait(3.0)
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop.set()

    def find(self, ssid: str, max_age_s: float = 180.0) -> tuple[bytes, str, float] | None:
        """(SSID as on the air, BSSID, seconds since seen) of the network a code names, or None."""
        now = time.monotonic()
        recent = {k: v for k, v in self.seen.items() if now - v[1] <= max_age_s}
        got = match_bytes(ssid.encode("utf-8"), recent)
        if got is None:
            return None
        bssid, t = recent[got]
        return got, bssid, now - t


def active_connection() -> str | None:
    p = nmcli("-t", "-f", "NAME,TYPE", "connection", "show", "--active", timeout=10)
    for line in p.stdout.splitlines():
        f = split_terse(line)
        if len(f) >= 2 and f[1] == "802-11-wireless":
            return f[0]
    return None


def join_wifi(net: dict, air: AirWatch | None = None, stop: threading.Event | None = None) -> tuple[bool, str]:
    """Join the network from a QR code; saved as `duck-<ssid>` at autoconnect priority -10.
    Returns (ok, why) with why one of: joined, not_found, failed, cancelled, WEP is not supported."""
    if net["security"] == "WEP":
        return False, "WEP is not supported"
    name = f"duck-{net['ssid']}"
    before = active_connection()
    nmcli("connection", "delete", "id", name, timeout=15)          # ours from an earlier code, if any
    if air is None:
        air = AirWatch().start()
    # seen while the code was being shown, or within the next 20 s
    t0 = time.monotonic()
    hit = air.find(net["ssid"])
    while hit is None and time.monotonic() - t0 < 20.0:
        if stop is not None and stop.wait(1.0):
            return False, "cancelled"
        time.sleep(0.0 if stop is not None else 1.0)
        hit = air.find(net["ssid"])
    if hit is None and not net["hidden"]:
        log(f"{net['ssid']!r} not on the air (since the scan began, and 20 s more)")
        return False, "not_found"
    if hit is not None:
        on_air, bssid, age = hit
        log(f"{net['ssid']!r} on the air as {on_air!r} at {bssid}, {age:.0f} s ago")
        # it must be on the air now to join: wait for it to come back (the iPhone's settings page)
        t0 = time.monotonic()
        while age > 6.0 and time.monotonic() - t0 < 30.0:
            if stop is not None and stop.wait(1.0):
                return False, "cancelled"
            time.sleep(0.0 if stop is not None else 1.0)
            _, bssid, age = air.find(net["ssid"]) or (on_air, bssid, age + 1.0)
        args = ["device", "wifi", "connect", bssid, "name", name]   # by BSSID: the SSID may be cut mid-character
    else:
        args = ["device", "wifi", "connect", ssid_on_air(net["ssid"]), "name", name, "hidden", "yes"]
    if net["security"] != "nopass":
        args += ["password", net["password"]]
    p = nmcli(*args, timeout=60, secret=net["password"] or None)
    if p.returncode != 0:
        nmcli("connection", "delete", "id", name, timeout=15)      # no half-made profile left behind
        if before and active_connection() is None:
            log(f"back to {before!r}")
            nmcli("connection", "up", "id", before, timeout=40)
        gone = "No network with" in (p.stderr or p.stdout or "")
        return False, "not_found" if gone else "failed"
    # not tied to one access point; below the home network
    nmcli("connection", "modify", "id", name, "802-11-wireless.bssid", "",
          "connection.autoconnect-priority", str(JOIN_PRIORITY))
    return True, "joined"


# ── Camera and scanning ─────────────────────────────────────────────────────────────────────────

def frames(duckcam: str, every: int = 3, half: bool = True):
    """Grey frames from duckcam: (t_s, exposure_us, gain, image) until the generator is closed."""
    import numpy as np
    args = [duckcam, "--every", str(every)] + (["--half"] if half else [])
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    try:
        while True:
            hdr = p.stdout.read(HEADER.size)
            if len(hdr) < HEADER.size:
                return
            magic, w, h, _seq, t_ns, exp_us, gain = HEADER.unpack(hdr)
            if magic != b"DCAM":
                raise RuntimeError("duckcam stream out of step")
            buf = p.stdout.read(w * h)
            if len(buf) < w * h:
                return
            yield t_ns / 1e9, exp_us, gain, np.frombuffer(buf, np.uint8).reshape(h, w)
    finally:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def decode_qr(det, img) -> str:
    """A QR code's text from a full-resolution grey frame, or "". The whole frame halved first (a
    code held close); then the middle 1280x720 at full resolution — on the robot (2026-10-17) a
    33-module code decoded from 150 px across, and a phone's 5 cm code at 30 cm is ~133 px halved,
    ~266 px full."""
    import cv2
    h, w = img.shape
    text, _, _ = det.detectAndDecode(cv2.resize(img, (w // 2, h // 2), interpolation=cv2.INTER_AREA))
    if text:
        return text
    cw, ch = min(w, 1280), min(h, 720)
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    text, _, _ = det.detectAndDecode(img[y0:y0 + ch, x0:x0 + cw])
    return text or ""


def capture_nv12(duckcam: str, settle_s: float = 1.2):
    """One colour frame (NV12, flat array, width, height) once the ISP's exposure has settled."""
    import numpy as np
    p = subprocess.Popen([duckcam, "--nv12", "--every", "5"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    t0, last = time.monotonic(), None
    try:
        while True:
            hdr = p.stdout.read(HEADER.size)
            if len(hdr) < HEADER.size:
                break
            magic, w, h, _seq, _t, _exp, _gain = HEADER.unpack(hdr)
            if magic != b"DCNV":
                raise RuntimeError("duckcam stream out of step")
            buf = p.stdout.read(w * h * 3 // 2)
            if len(buf) < w * h * 3 // 2:
                break
            last = (np.frombuffer(buf, np.uint8), w, h)
            if time.monotonic() - t0 >= settle_s:
                break
    finally:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    if last is None:
        raise RuntimeError("no frame")
    return last


def judge_remote(brain: str, kind: str, state: dict, images: dict, timeout: float) -> dict | None:
    """Jev's answers through duckbrain's /judge (frp), or None."""
    body = json.dumps({"kind": kind, "state": state, "images": images}).encode()
    req = urllib.request.Request(brain.rstrip("/") + "/judge", data=body, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.loads(r.read())
    except (OSError, ValueError) as e:
        log(f"judge {kind}: {e!r}")
        return None
    if not out.get("ok"):
        log(f"judge {kind}: {out.get('error')}")
        return None
    return out.get("answers")


def scan_qr(duckcam: str, timeout_s: float, stop: threading.Event | None = None) -> str | None:
    """The first QR code the camera reads within `timeout_s`, as text."""
    import cv2
    det = cv2.QRCodeDetector()
    t0 = time.monotonic()
    n = 0
    gen = frames(duckcam, every=6, half=False)       # 5 frames a second, full resolution
    try:
        for _t, _exp, _gain, img in gen:
            n += 1
            text = decode_qr(det, img)
            if text:
                log(f"QR read after {time.monotonic() - t0:.1f} s ({n} frames)")
                return text
            if time.monotonic() - t0 > timeout_s or (stop and stop.is_set()):
                break
    finally:
        gen.close()
    log(f"no QR code in {time.monotonic() - t0:.1f} s ({n} frames)")
    return None


# ── Service ─────────────────────────────────────────────────────────────────────────────────────

def to_lifed(event: dict, path: str = LIFED):
    try:
        s = socket.socket(socket.AF_UNIX)
        s.settimeout(1.0)
        s.connect(path)
        s.sendall((json.dumps({"jsonrpc": "2.0", "method": "life.event", "params": event},
                              ensure_ascii=False) + "\n").encode())
        s.close()
    except OSError as e:
        log(f"lifed unreachable: {e}")


class Visiond:
    def __init__(self, a):
        self.a = a
        self.busy = threading.Lock()
        self.stop = threading.Event()
        self.status = {"mode": "idle"}
        self.finder = None                # YuNet, loaded on the first watch
        self.watch_stop = threading.Event()
        self.watch_until, self.watch_hz = 0.0, 2.0
        self.latest = None                # the watch's last settled frame: (nv12, w, h, monotonic)

    def provision(self, timeout_s: float, join: bool):
        """Scan for a QR code; join the Wi-Fi it names. The result goes to lifed."""
        with self.busy:
            self.stop.clear()
            self.status = {"mode": "scan_qr", "since": time.time()}
            air = AirWatch().start() if join else None      # what is on the air while you get the code ready
            try:
                text = scan_qr(self.a.duckcam, timeout_s, self.stop)
                if self.stop.is_set():
                    ev = {"type": "qr", "result": "cancelled"}
                    log("scan cancelled")
                elif text is None:
                    ev = {"type": "qr", "result": "timeout"}
                else:
                    net = parse_wifi_qr(text)
                    if net is None:
                        ev = {"type": "qr", "result": "not_wifi"}
                        log("the QR code is not a Wi-Fi code")
                    elif match_network(net["ssid"], [current_ssid() or ""]):
                        # already on it (a test with the home network): joining again would drop the link
                        ev = {"type": "qr", "result": "already", "ssid": net["ssid"]}
                        log(f"Wi-Fi code for {net['ssid']!r}: already on it, not rejoining")
                    elif not join:
                        ev = {"type": "qr", "result": "read", "ssid": net["ssid"]}
                        log(f"Wi-Fi code for {net['ssid']!r} ({net['security']}); not joining (--no-join)")
                    else:
                        log(f"joining {net['ssid']!r} ({net['security']})")
                        to_lifed({"type": "qr", "result": "joining", "ssid": net["ssid"]}, self.a.lifed)
                        ok, why = join_wifi(net, air, self.stop)
                        result = "joined" if ok else (why if why in ("not_found", "cancelled") else "failed")
                        ev = {"type": "qr", "result": result, "ssid": net["ssid"], "why": why}
                        log(f"{net['ssid']!r}: {why}")
            except Exception as e:  # noqa: BLE001 - report, never crash the service
                log(f"scan failed: {e!r}")
                ev = {"type": "qr", "result": "failed", "why": "camera"}
            if air is not None:
                air.close()
            self.status = {"mode": "idle", "last": ev, "at": time.time()}
            to_lifed(ev, self.a.lifed)
            return ev

    def photo(self) -> dict:
        """One grey photo of what the camera sees, after 1.5 s for the exposure to settle."""
        if not self.busy.acquire(blocking=False):
            return {"ok": False, "reason": "busy"}
        try:
            import cv2
            img, exp, gain, t0 = None, 0, 0, time.monotonic()
            gen = frames(self.a.duckcam, every=5, half=True)
            try:
                for _t, exp, gain, img in gen:
                    if time.monotonic() - t0 > 1.5:
                        break
            finally:
                gen.close()
            if img is None:
                return {"ok": False, "reason": "no frame"}
            cv2.imwrite(PHOTO, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return {"ok": True, "path": PHOTO, "exposure_us": exp, "gain": gain}
        except Exception as e:  # noqa: BLE001
            return {"ok": False, "reason": repr(e)}
        finally:
            self.busy.release()

    def snap(self, width: int = 1024) -> dict:
        """One colour photo for the brain to look at (lifed's `look`): kept under LOOKS as <id>.jpg,
        only the last LOOKS_KEPT, and returned as base64."""
        import base64
        import jevlook
        # asked for: the eyes' watch's frame, or wait for a frame being taken (ToF's look for Jev,
        # ~1.5 s) - not for a QR scan
        try:
            got = self.frame_now(6.0)
            if got is None:
                return {"ok": False, "reason": "busy"}
            jpeg = jevlook.nv12_to_jpeg(got[0], got[1], got[2], out_w=width, quality=80)
        except Exception as e:  # noqa: BLE001
            return {"ok": False, "reason": repr(e)}
        os.makedirs(LOOKS, exist_ok=True)
        pid = str(int(time.time() * 1000))
        with open(os.path.join(LOOKS, pid + ".jpg"), "wb") as f:
            f.write(jpeg)
        for old in sorted(os.listdir(LOOKS))[:-LOOKS_KEPT]:
            os.remove(os.path.join(LOOKS, old))
        log(f"snap {pid}: {len(jpeg) // 1024} KB")
        return {"ok": True, "id": pid, "jpeg_b64": base64.b64encode(jpeg).decode()}

    def watch(self, seconds: float, hz: float):
        """The eyes open (attention-gaze-design §1.3): colour frames from duckcam, ~`hz` of them looked at
        once the exposure has settled - faces (YuNet, faces.DETECT_SIZE) and what moves (motion.py) - and sent to
        lifed as camera angles (lifed adds the head's pose): `face` for the largest face, `motion` for
        the largest moving patch away from it. lifed renews it (vision.watch again: a later end, a new
        rate) while it is awake; it ends with `watch_end`. While it runs it is the camera's one owner:
        a photo is its latest frame (frame_now), a QR scan stops it."""
        import cv2
        import faces
        import motion
        import numpy as np
        try:
            if self.finder is None:
                self.finder = faces.FaceFinder(self.a.face_model)
        except Exception as e:  # noqa: BLE001 - no model, no watch; the rest of the eyes still work
            log(f"watch: no face finder ({e!r})")
            self.busy.release()
            return
        self.watch_stop.clear()
        self.watch_until, self.watch_hz = time.monotonic() + seconds, hz
        found = moved = frames = 0
        # 5 frames a second from the camera (the most a watch looks at), ~hz of them looked at
        p = subprocess.Popen([self.a.duckcam, "--nv12", "--every", "6"], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL)
        t0, last, prev = time.monotonic(), 0.0, None
        self.status = {"mode": "watch", "since": time.time()}
        threads = cv2.getNumThreads()
        cv2.setNumThreads(1)        # OpenCV's threads mostly spin: one core does it for a quarter of the CPU
        try:
            while time.monotonic() < self.watch_until and not self.watch_stop.is_set():
                hdr = p.stdout.read(HEADER.size)
                if len(hdr) < HEADER.size:
                    break
                magic, w, h, _seq, _t, _exp, _gain = HEADER.unpack(hdr)
                if magic != b"DCNV":
                    raise RuntimeError("duckcam stream out of step")
                buf = p.stdout.read(w * h * 3 // 2)
                if len(buf) < w * h * 3 // 2:
                    break
                now = time.monotonic()
                if now - t0 < WATCH_SETTLE_S:
                    continue
                nv12 = np.frombuffer(buf, np.uint8).reshape(h * 3 // 2, w)
                self.latest = (nv12, w, h, now)
                if now - last < 0.8 / self.watch_hz:
                    continue
                last = now
                frames += 1
                grey = motion.small_grey(nv12[: h - 16])
                blob, prev = motion.moving_blob(prev, grey), grey
                hits = self.finder.find(faces.small_bgr(nv12, w, h))
                fx = None
                if hits:
                    found += 1
                    x, y, bw, bh, score = hits[0]
                    yaw, pitch = faces.face_angles(x + bw / 2, y + bh / 2, *faces.DETECT_SIZE)
                    fx = ((x + bw / 2) / faces.DETECT_SIZE[0], bw / faces.DETECT_SIZE[0])
                    to_lifed({"type": "face", "yaw_cam": round(yaw, 4), "pitch_cam": round(pitch, 4),
                              "size": round(fx[1], 3), "conf": round(score, 2), "n": len(hits)}, self.a.lifed)
                if blob is not None:
                    cx, cy, _frac, size = blob
                    # the face's own movement is the face; a hand or a ball beside it is something else
                    if fx is None or abs(cx / motion.SMALL[0] - fx[0]) > 1.5 * fx[1]:
                        moved += 1
                        yaw, pitch = faces.face_angles(cx, cy, *motion.SMALL)
                        to_lifed({"type": "motion", "yaw_cam": round(yaw, 4), "pitch_cam": round(pitch, 4),
                                  "size": round(size, 4)}, self.a.lifed)
        except Exception as e:  # noqa: BLE001 - report, never crash the service
            log(f"watch failed: {e!r}")
        finally:
            cv2.setNumThreads(threads)
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
            self.latest = None
            self.status = {"mode": "idle", "last_watch": {"frames": frames, "faces": found, "motion": moved,
                                                          "at": time.time()}}
            self.busy.release()
        log(f"watch {time.monotonic() - t0:.0f} s: {frames} frames, {found} with a face, {moved} with motion")
        to_lifed({"type": "watch_end", "frames": frames, "faces": found, "motion": moved}, self.a.lifed)

    def frame_now(self, wait_s: float):
        """One settled colour frame (nv12, w, h): the running watch's latest, or one of its own under the
        camera lock (waiting up to `wait_s` for it). None: the camera is busy (a QR scan)."""
        if self.status.get("mode") == "scan_qr":
            return None
        end = time.monotonic() + 1.5            # a watch just started: its first settled frame
        while self.status.get("mode") == "watch" and time.monotonic() < end:
            lt = self.latest
            if lt is not None and time.monotonic() - lt[3] < 0.6:
                return lt[:3]
            time.sleep(0.05)
        if not (self.busy.acquire(timeout=wait_s) if wait_s > 0 else self.busy.acquire(blocking=False)):
            return None
        try:
            return capture_nv12(self.a.duckcam)
        finally:
            self.busy.release()

    def handle(self, m: dict):
        method, p = m.get("method"), m.get("params") or {}
        if method == "vision.scan_qr":
            if self.status.get("mode") == "watch":
                self.watch_stop.set()                # the eyes' watch gives way to a scan
            elif self.busy.locked():
                return {"accepted": False, "reason": "already scanning"}
            threading.Thread(target=self.provision, args=(float(p.get("timeout_s", 30.0)), bool(p.get("join", True))),
                             daemon=True).start()
            return {"accepted": True}
        if method == "vision.photo":
            return self.photo()
        if method == "vision.watch":
            seconds = max(0.5, min(float(p.get("seconds", 4.0)), 20.0))
            hz = max(0.5, min(float(p.get("hz", 2.0)), 5.0))
            if self.status.get("mode") == "watch" and not self.watch_stop.is_set():
                self.watch_until = max(self.watch_until, time.monotonic() + seconds)
                self.watch_hz = hz
                return {"accepted": True, "renewed": True}
            if not self.busy.acquire(blocking=False):
                return {"accepted": False, "reason": "busy"}
            threading.Thread(target=self.watch, args=(seconds, hz), daemon=True).start()
            return {"accepted": True}
        if method == "vision.snap":
            return self.snap(int(p.get("width", 1024)))
        if method == "vision.stop":
            self.stop.set()
            self.watch_stop.set()
            return {"accepted": True}
        if method == "vision.state":
            return self.status
        return None

    def watch_tof(self):
        """Someone coming close in front of the head (tofwatch): a `person` event for lifed."""
        import tofwatch

        def on_person(dist):
            log(f"tof: something at {dist:.2f} m")
            to_lifed({"type": "near", "dist": round(dist, 2), "via": "tof"}, self.a.lifed)
            threading.Thread(target=self.look_at_person, args=(round(dist, 2),), daemon=True).start()
        tofwatch.watch(self.a.tof, on_person, threading.Event(), log)

    def look_at_person(self, dist: float):
        """Something came close: one colour frame for Jev — is it a person, what are they doing,
        where is the face. Without Jev (off, busy camera, no answer), the bare ToF event."""
        import base64
        import jevlook
        answers = None
        if self.a.jev_images == "on":
            # the camera only while the frame is taken, not while Jev thinks (2026-10-17: holding a cup out
            # for "你看这是什么" set off this look first, and the asked-for photo found the camera busy)
            t0, jpeg = time.monotonic(), None
            try:
                got = self.frame_now(0.0)
                jpeg = None if got is None else jevlook.nv12_to_jpeg(*got)
            except Exception as e:  # noqa: BLE001 - the eyes failing must not silence the greeting
                log(f"look failed: {e!r}")
            if jpeg is not None:
                answers = judge_remote(self.a.brain, "person", {"tof_distance_m": dist},
                                       {"view": base64.b64encode(jpeg).decode()}, timeout=6.0)
                log(f"look: {time.monotonic() - t0:.1f} s, {len(jpeg) // 1024} KB -> "
                    f"{ {k: v.get('noul', v.get('choice')) for k, v in (answers or {}).items()} }")
        ev = jevlook.person_event(answers, dist)
        self.status["last_look"] = {k: v for k, v in ev.items() if k != "type"}
        to_lifed(ev, self.a.lifed)

    def serve(self):
        path = self.a.sock
        if self.a.tof:
            threading.Thread(target=self.watch_tof, daemon=True).start()
        if os.path.exists(path):
            os.unlink(path)
        srv = socket.socket(socket.AF_UNIX)
        srv.bind(path)
        os.chmod(path, 0o660)
        srv.listen(4)
        log(f"visiond up: {path}, camera {self.a.duckcam}")
        while True:
            c, _ = srv.accept()
            threading.Thread(target=self.client, args=(c,), daemon=True).start()

    def client(self, c):
        with c, c.makefile("rwb") as f:
            for line in f:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                res = self.handle(m)
                if m.get("id") is not None:
                    out = {"jsonrpc": "2.0", "id": m["id"]}
                    out.update({"result": res} if res is not None else
                               {"error": {"code": -32601, "message": f"unknown method {m.get('method')}"}})
                    f.write((json.dumps(out, ensure_ascii=False) + "\n").encode())
                    f.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sock", default="/tmp/visiond.sock")
    ap.add_argument("--duckcam", default=os.path.expanduser("~/visiond/duckcam"))
    ap.add_argument("--face-model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "models",
                                                         "face_detection_yunet_2023mar.onnx"))
    ap.add_argument("--lifed", default=LIFED, help="where outcomes go ('' or a dead path: nowhere)")
    ap.add_argument("--tof", default="/tmp/tofd.sock", help="tofd's socket, for someone coming close ('' = off)")
    ap.add_argument("--brain", default="http://127.0.0.1:8210", help="duckbrain (Jev's /judge), through frp")
    ap.add_argument("--jev-images", choices=("on", "off"), default="on",
                    help="when someone comes close, show Jev one colour frame (it goes to TypeSafe's API)")
    ap.add_argument("--scan", type=float, help="scan for a QR code this long, print it and exit")
    ap.add_argument("--no-join", action="store_true", help="with --scan: read the code, do not join")
    a = ap.parse_args()
    if a.scan:
        v = Visiond(a)
        print(json.dumps(v.provision(a.scan, join=not a.no_join), ensure_ascii=False))
        return
    Visiond(a).serve()


if __name__ == "__main__":
    main()
