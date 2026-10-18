# -*- coding: utf-8 -*-
"""duckweb: a page on the phone for what Xiaobai hears, says and means (2026-10-17).

Served by the robot on the local network (http://<robot>:8088, standard library only). It shows the
conversation — your words as it heard them, its duck-speak and what it meant (the brain's `meaning`,
until now only in lifed's log) — its state, battery, heat, whether the online brain is reachable,
and the network it is on; and it lets you type to it, ask it to scan a Wi-Fi QR code, or see what
the camera sees.

It only asks lifed and visiond (their sockets); it never talks to robotd. Typed words are taken as
said with its name, so the command words work from here as they do by voice.

    python3 duckweb.py [--port 8088] [--lifed /tmp/lifed.sock] [--vision /tmp/visiond.sock]
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# the duck's name on the page comes from its persona (tools/persona, ~/persona on the robot)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "persona"))
from persona import load as load_persona  # noqa: E402


def rpc(path: str, method: str, params: dict | None = None, timeout: float = 3.0):
    """One JSON-RPC call over a unix socket; None if nobody answers."""
    try:
        s = socket.socket(socket.AF_UNIX)
        s.settimeout(timeout)
        s.connect(path)
        s.sendall((json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}},
                              ensure_ascii=False) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        s.close()
        return json.loads(buf.split(b"\n")[0]).get("result")
    except (OSError, ValueError):
        return None


def network() -> dict:
    """The Wi-Fi it is on and its address."""
    out = {"ssid": None, "ip": None}
    try:
        p = subprocess.run(["nmcli", "-t", "-f", "ACTIVE,SSID", "device", "wifi"], capture_output=True, text=True,
                           timeout=3)
        for line in p.stdout.splitlines():
            if line.startswith("yes:"):
                out["ssid"] = line[4:].replace("\\:", ":")
        p = subprocess.run(["ip", "-4", "-o", "addr", "show", "wlan0"], capture_output=True, text=True, timeout=3)
        parts = p.stdout.split()
        if "inet" in parts:
            out["ip"] = parts[parts.index("inet") + 1].split("/")[0]
    except (OSError, subprocess.SubprocessError):
        pass
    return out


class Handler(BaseHTTPRequestHandler):
    server_version = "duckweb/1"

    def log_message(self, fmt, *args):     # quiet: the page polls every second
        pass

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    def do_GET(self):
        a = self.server.args
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.replace("小白", NAME).encode(), "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._json({"life": rpc(a.lifed, "life.state"), "vision": rpc(a.vision, "vision.state"),
                        "net": network(), "now": time.time()})
        elif self.path.startswith("/api/look/"):
            pid = self.path.rsplit("/", 1)[-1].split(".")[0]
            path = os.path.join("/tmp/visiond-looks", pid + ".jpg")
            if pid.isdigit() and os.path.exists(path):
                with open(path, "rb") as f:
                    self._send(200, f.read(), "image/jpeg")
            else:
                self._send(404, b"gone", "text/plain")
        elif self.path.startswith("/api/photo.jpg"):
            path = "/tmp/visiond-photo.jpg"
            if os.path.exists(path):
                with open(path, "rb") as f:
                    self._send(200, f.read(), "image/jpeg")
            else:
                self._send(404, b"no photo yet", "text/plain")
        else:
            self._send(404, b"not here", "text/plain")

    def do_POST(self):
        a = self.server.args
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            body = {}
        if self.path == "/api/say":
            text = str(body.get("text", "")).strip()[:100]
            if not text:
                return self._json({"ok": False, "reason": "empty"}, 400)
            res = rpc(a.lifed, "life.event", {"type": "say", "text": text, "addressed": True, "named": True})
            self._json({"ok": res is not None})
        elif self.path == "/api/scan":
            res = rpc(a.lifed, "life.event", {"type": "say", "text": "连网", "addressed": True, "named": True})
            self._json({"ok": res is not None})
        elif self.path == "/api/volume":
            try:
                v = max(20, min(100, int(body.get("value"))))
            except (TypeError, ValueError):
                return self._json({"ok": False, "reason": "value"}, 400)
            res = rpc(a.lifed, "life.event", {"type": "volume", "set": v})
            self._json({"ok": res is not None})
        elif self.path == "/api/missions":
            mode = str(body.get("mode", ""))
            if mode not in ("dry", "real"):
                return self._json({"ok": False, "reason": "mode"}, 400)
            res = rpc(a.lifed, "life.event", {"type": "missions_mode", "mode": mode})
            self._json({"ok": res is not None, "mode": (res or {}).get("mode")})
        elif self.path == "/api/photo":
            res = rpc(a.vision, "vision.photo", {}, timeout=15.0)
            self._json({"ok": bool(res and res.get("ok")), "reason": (res or {}).get("reason")})
        else:
            self._json({"ok": False, "reason": "unknown"}, 404)


PAGE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>小白</title>
<style>
:root { --bg:#f6f4ef; --card:#fff; --ink:#24221f; --mute:#8a857c; --duck:#fff3c4; --you:#dcefff; --line:#e7e2d8;
        --ok:#2f8f4e; --warn:#c77700; --bad:#c43c3c; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#1b1a18; --card:#262421; --ink:#eeeae2; --mute:#9a958b; --duck:#4a3f1c; --you:#1f3b52; --line:#3a3732; }
}
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--ink); font: 16px/1.45 -apple-system, "PingFang SC", "Noto Sans SC", sans-serif; }
header { position:sticky; top:0; background:var(--bg); padding:12px 16px 8px; border-bottom:1px solid var(--line); z-index:1; }
h1 { margin:0 0 6px; font-size:20px; }
.chips { display:flex; flex-wrap:wrap; gap:6px; }
.chip { background:var(--card); border:1px solid var(--line); border-radius:999px; padding:2px 10px; font-size:13px; color:var(--mute); }
.chip b { color:var(--ink); font-weight:600; }
.ok { color:var(--ok)!important } .warn { color:var(--warn)!important } .bad { color:var(--bad)!important }
main { padding:12px 16px 120px; max-width:720px; margin:0 auto; }
.convo { font-size:13px; color:var(--mute); margin:4px 0 10px; }
.msg { display:flex; margin:8px 0; }
.msg.you { justify-content:flex-end; }
.bubble { max-width:82%; padding:8px 12px; border-radius:14px; background:var(--card); border:1px solid var(--line); }
.you .bubble { background:var(--you); }
.duck .bubble { background:var(--duck); }
.say { font-size:13px; color:var(--mute); }
.meaning { font-size:16px; }
.meta { font-size:11px; color:var(--mute); margin-top:2px; }
footer { position:fixed; left:0; right:0; bottom:0; background:var(--bg); border-top:1px solid var(--line); padding:10px 16px; }
.row { display:flex; gap:8px; max-width:720px; margin:0 auto; }
input { flex:1; min-width:0; font-size:16px; padding:10px 12px; border-radius:10px; border:1px solid var(--line); background:var(--card); color:var(--ink); }
button { font-size:15px; padding:10px 12px; border-radius:10px; border:1px solid var(--line); background:var(--card); color:var(--ink); }
button.primary { background:#f2c94c; border-color:#e0b53a; color:#24221f; }
.tools { display:flex; gap:8px; max-width:720px; margin:8px auto 0; }
.tools button { flex:1; font-size:14px; padding:8px; }
.vol { display:flex; align-items:center; gap:8px; max-width:720px; margin:8px auto 0; font-size:14px; color:var(--mute); }
.vol input { flex:1; }
.mission { max-width:720px; margin:8px auto; padding:10px 12px; border:1px solid var(--line); border-radius:12px; background:var(--card); font-size:14px; }
.mhead { margin-bottom:4px; }
.mmode { max-width:720px; margin:8px auto 0; font-size:14px; }
.mmode button { font-size:13px; padding:4px 10px; margin-left:6px; }
.mmode b.bad { color:#c0392b; }
.mstep { padding:2px 0; }
.mstep.done { color:var(--mute); }
.mstep.running { font-weight:600; }
.mstep.failed { color:#c0392b; }
.mnote { color:var(--mute); font-size:12px; }
img.look { display:block; width:100%; max-width:320px; border-radius:10px; margin:6px 0; border:1px solid var(--line); }
#photo { display:none; width:100%; border-radius:12px; margin:8px 0; border:1px solid var(--line); }
.empty { color:var(--mute); text-align:center; margin-top:40px; }
</style></head>
<body>
<header>
  <h1>🦆 小白</h1>
  <div class="chips" id="chips"></div>
</header>
<main>
  <div class="convo" id="convo"></div>
  <div id="mission"></div>
  <img id="photo" alt="小白看到的">
  <div id="log"><div class="empty">还没有对话。叫一声"小白"，或者在下面打字。</div></div>
</main>
<footer>
  <div class="row">
    <input id="text" placeholder="对小白说…" enterkeyhint="send" autocomplete="off">
    <button class="primary" id="send">发送</button>
  </div>
  <div class="tools">
    <button id="scan">📶 扫码连网</button>
    <button id="look">📷 看看它看到的</button>
  </div>
  <div class="vol">🔈<input type="range" id="vol" min="20" max="100" step="5" value="100" aria-label="音量">🔊<span id="volv"></span></div>
</footer>
<script>
const STATE = {OFF:"没上力", WAKING:"醒来中", IDLE:"发呆", ATTENDING:"在听你说", PLAYING:"走路中",
               RESTING:"休息中", SLEEPING:"睡觉中", FALLEN:"摔倒了", TIRED:"没电了"};
const EMO = {"开心":"😄","好奇":"🤔","撒娇":"🥺","害羞":"😳","困":"😪","难过":"😢","惊讶":"😮"};
const VIA = {brain:"大脑", small:"小脑", vision:"眼睛", eyes:"大脑看图", mission:"任务"};
const MSTATE = {proposed:"等你说「开始」", running:"进行中", paused:"暂停", done:"完成", failed:"没做到", cancelled:"取消了"};
const MSTEP = {pending:"·", running:"▶", done:"✓", failed:"✗"};
function modeLine(L) {
  const real = L.missions_mode === "real";
  const tof = L.tof_m == null ? "无数据" : (L.tof_m === "far" ? "前方空旷" : `前方 ${L.tof_m} m`);
  return `<div class="mmode">🧭 任务模式：<b class="${real ? "bad" : ""}">${real ? "实机（会真的走）" : "演练（不会动）"}</b>
    <button id="mswitch">${real ? "切回演练" : "切到实机"}</button> <span class="mnote">ToF：${tof}</span></div>`;
}
async function switchMode(L) {
  const toReal = L.missions_mode !== "real";
  if (toReal && !confirm("实机模式下，说出路线并说「开始」后小白会真的走。确认它站在空旷的平地上、你在旁边，并且随时可以说「停」或碰手柄？")) return;
  await post("/api/missions", {mode: toReal ? "real" : "dry"});
  setTimeout(refresh, 300);
}
function missionCard(M) {
  if (!M) return "";
  const rows = M.steps.map(s => `<div class="mstep ${s.status}">${MSTEP[s.status] || "·"} ${esc(s.words)}${s.note ? ` <span class="mnote">${esc(s.note)}</span>` : ""}</div>`).join("");
  const sim = M.sim ? ` · 模拟位置 (${M.sim.x}, ${M.sim.y}) m，朝向 ${M.sim.yaw_deg}°，${M.sim.posture === "sitting" ? "坐着" : "站着"}` : "";
  return `<div class="mission"><div class="mhead">🧭 任务${M.mode === "dry" ? "（演练，不会真的动）" : ""}：<b>${MSTATE[M.state] || M.state}</b>${M.why ? "（" + esc(M.why) + "）" : ""}</div>${rows}
    <div class="mnote">位置的把握：±${M.sigma_m} m，±${M.sigma_deg}°${sim}</div></div>`;
}
const INTENT = {sleep:"睡觉", rest:"休息", sit:"坐下", stand:"站起来", wake:"醒来", net:"连网", bye:"道别", chat:"聊天", unclear:"听不清"};
const ACT = {ignore:"不理", command:"当指令", small:"小脑答", nod:"点头", brain:"大脑答"};
const SEEN = {looking_at_duck:"看着它", waving:"招手", thumbs_up:"竖大拇指", reaching:"伸手", passing:"路过", busy:"在忙", unclear:"看不清"};
function jevNote(j) {
  if (!j || j.for_duck == null) return "";
  return ` · Jev：对我说 ${Math.round(j.for_duck * 100)}% · ${INTENT[j.intent] || j.intent || "?"}${j.action ? " → " + (ACT[j.action] || j.action) : ""}`;
}
let lastKey = "";
function chip(label, value, cls) { return `<span class="chip">${label} <b class="${cls||""}">${value}</b></span>`; }
function esc(s) { return String(s ?? "").replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c])); }
function hhmm(t) { const d = new Date(t * 1000); return d.toLocaleTimeString("zh-CN", {hour:"2-digit", minute:"2-digit"}); }
async function refresh() {
  let s;
  try { s = await (await fetch("/api/state")).json(); } catch (e) {
    document.getElementById("chips").innerHTML = chip("连接", "连不上小白", "bad"); return; }
  const L = s.life || {}, net = s.net || {}, V = s.vision || {};
  const c = [];
  c.push(chip("状态", STATE[L.state] || (s.life ? L.state : "lifed 没在运行"), s.life ? "" : "bad"));
  if (L.battery_pct != null) c.push(chip("电量", Math.round(L.battery_pct) + "%", L.battery_pct < 20 ? "bad" : L.battery_pct < 35 ? "warn" : "ok"));
  if (L.max_c != null) c.push(chip("最热", Math.round(L.max_c) + "°C", L.max_c >= 65 ? "bad" : L.max_c >= 58 ? "warn" : ""));
  c.push(chip("大脑", L.brain_online === false ? "离线·用小脑" : "在线", L.brain_online === false ? "warn" : "ok"));
  c.push(chip("网络", net.ssid ? esc(net.ssid) + (net.ip ? " · " + net.ip : "") : "未连接", net.ssid ? "" : "bad"));
  if (L.clock) c.push(chip("时间", L.clock));
  const VOL = L.volume;
  if (VOL) {
    c.push(chip("音量", VOL.now + "%" + (VOL.night_cap ? "（夜间）" : "")));
    const el = document.getElementById("vol");
    if (!volDragging) el.value = VOL.user;
    document.getElementById("volv").textContent = VOL.user + "%";
  }
  const J = L.jev;
  if (J) c.push(chip("Jev", J.mode === "off" ? "关" : `${J.mode === "shadow" ? "影子" : "开"} · ${J.ok} 次${J.fallback ? " · 退回 " + J.fallback : ""}`,
                     J.mode === "off" ? "" : (J.ok ? "ok" : "warn")));
  const look = V.last_look;
  if (look && look.via === "jev") c.push(chip("刚才看到", look.present ? "有人 · " + (SEEN[look.activity] || look.activity) : "没人"));
  if (V.mode === "scan_qr" || L.scanning) c.push(chip("眼睛", "正在找二维码…", "warn"));
  document.getElementById("chips").innerHTML = c.join("");
  document.getElementById("convo").textContent = L.convo_s_left > 0
    ? `对话中，还会听 ${Math.round(L.convo_s_left)} 秒` : "叫一声“小白”开始对话";
  document.getElementById("mission").innerHTML = (L.missions_mode ? modeLine(L) : "") + missionCard(L.mission);
  const sw = document.getElementById("mswitch");
  if (sw) sw.onclick = () => switchMode(L);
  const d = L.dialog || [];
  const key = d.map(x => x.t + x.who + (x.jev ? "j" : "")).join("|");
  if (key !== lastKey) {
    lastKey = key;
    const html = d.slice().reverse().map(x => x.who === "you"
      ? `<div class="msg you"><div class="bubble"><div class="meaning">${esc(x.text)}</div><div class="meta">${hhmm(x.t)}${x.command ? " · 指令" : ""}${esc(jevNote(x.jev))}</div></div></div>`
      : `<div class="msg duck"><div class="bubble"><div class="meaning">${EMO[x.emotion] || ""} ${esc(x.meaning || x.text)}</div>
           ${x.photo ? `<img class="look" src="/api/look/${encodeURIComponent(x.photo)}.jpg" alt="它看到的" onerror="this.remove()">` : ""}
           <div class="say">鸭语：${esc(x.text)}</div><div class="meta">${hhmm(x.t)} · ${VIA[x.via] || ""}</div></div></div>`).join("");
    document.getElementById("log").innerHTML = html || '<div class="empty">还没有对话。叫一声"小白"，或者在下面打字。</div>';
  }
}
async function post(path, body) {
  try { return await (await fetch(path, {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify(body || {})})).json(); }
  catch (e) { return {ok:false}; }
}
document.getElementById("send").onclick = async () => {
  const el = document.getElementById("text"); const text = el.value.trim(); if (!text) return;
  el.value = ""; await post("/api/say", {text}); setTimeout(refresh, 300);
};
document.getElementById("text").addEventListener("keydown", e => { if (e.key === "Enter") document.getElementById("send").click(); });
let volDragging = false;
const volEl = document.getElementById("vol");
volEl.oninput = () => { volDragging = true; document.getElementById("volv").textContent = volEl.value + "%"; };
volEl.onchange = async () => { await post("/api/volume", {value: Number(volEl.value)}); volDragging = false; setTimeout(refresh, 300); };
document.getElementById("scan").onclick = async () => { await post("/api/scan"); setTimeout(refresh, 300); };
document.getElementById("look").onclick = async () => {
  const b = document.getElementById("look"); b.disabled = true; b.textContent = "📷 拍照中…";
  const r = await post("/api/photo");
  b.disabled = false; b.textContent = "📷 看看它看到的";
  const img = document.getElementById("photo");
  if (r.ok) { img.src = "/api/photo.jpg?" + Date.now(); img.style.display = "block"; }
  else { alert("拍不了：" + (r.reason || "眼睛没在运行")); }
};
refresh(); setInterval(refresh, 1000);
</script>
</body></html>
"""
# the page is written with Xiaobai's name; it shows whichever duck's persona is installed
NAME = load_persona().name



def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--lifed", default="/tmp/lifed.sock")
    ap.add_argument("--vision", default="/tmp/visiond.sock")
    a = ap.parse_args()
    srv = ThreadingHTTPServer((a.bind, a.port), Handler)
    srv.args = a
    print(time.strftime("%H:%M:%S"), f"duckweb on http://{a.bind}:{a.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
