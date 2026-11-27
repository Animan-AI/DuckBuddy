#!/usr/bin/python3
"""First-boot Wi-Fi portal. NetworkManager owns both wireless profiles."""

import html
import ipaddress
import json
import os
import re
import secrets
import socket
import socketserver
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

PORTAL = "10.42.0.1"
AP_PASSWORD = "duck2026"
RUNTIME = Path("/run/duck-hotspot")
WIFI_PROFILES = Path("/etc/NetworkManager/system-connections")
TEMP_PROFILES = Path("/run/NetworkManager/system-connections")


class NetworkError(RuntimeError):
    pass


def nmcli(*args, wait=5, check=True):
    try:
        result = subprocess.run(
            ["nmcli", "--wait", str(wait), *args], capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=wait + 3,
            env={**os.environ, "LC_ALL": "C.UTF-8", "LANGUAGE": "C"},
        )
    except subprocess.TimeoutExpired:
        if check:
            raise NetworkError("NetworkManager 操作超时") from None
        return ""
    if result.returncode and check:
        # NetworkManager stderr is kept private, including any user-supplied network details.
        raise NetworkError(f"NetworkManager 操作失败（退出码 {result.returncode}）")
    return result.stdout.strip() if result.returncode == 0 else ""


def nearby_networks(rows):
    networks = {}
    for line in rows.splitlines():
        row = re.fullmatch(r"(\d+):((?:\\.|[^:\\])*):(.*)", line)
        if row is None:
            continue
        signal = int(row[1])
        ssid = re.sub(r"\\([:\\])", r"\1", row[2])
        if ssid == "--" or not 1 <= len(ssid.encode("utf-8")) <= 32 or not ssid.isprintable():
            continue
        security = set(row[3].upper().split())
        if "802.1X" in security or "EAP" in security:
            continue
        if security & {"WPA", "WPA1", "WPA2"}:
            password_type = "WPA"
        elif "WPA3" in security:
            password_type = "SAE"
        elif not security or security == {"--"}:
            password_type = "nopass"
        else:
            continue
        if ssid not in networks or signal > networks[ssid][0]:
            networks[ssid] = (signal, password_type)
    return dict(sorted(networks.items(), key=lambda item: (-item[1][0], item[0].casefold())))


def validated_network(data):
    ssid = data.get("ssid", "")
    password = data.get("password", "")
    security = data.get("security", "WPA").upper()
    hidden = data.get("hidden") in (True, "1")
    if any(ord(char) < 32 or ord(char) == 127 for char in ssid + password):
        raise ValueError("Wi-Fi 名称和密码不能含控制字符")
    if not 1 <= len(ssid.encode("utf-8")) <= 32:
        raise ValueError("Wi-Fi 名称必须为 1–32 字节")
    if security in ("WPA", "WPA2"):
        key_mgmt = "wpa-psk"
        if not (8 <= len(password.encode("utf-8")) <= 63 or re.fullmatch(r"[a-fA-F0-9]{64}", password)):
            raise ValueError("WPA/WPA2 密码须为 8–63 字节，或 64 位十六进制密钥")
    elif security in ("SAE", "WPA3"):
        key_mgmt = "sae"
        if not 1 <= len(password.encode("utf-8")) <= 63:
            raise ValueError("请填写有效的 WPA3 密码")
    elif security in ("NOPASS", "OPEN"):
        key_mgmt, password = "", ""
    else:
        raise ValueError("请选择 WPA/WPA2、WPA3 或无密码网络")
    return ssid, password, key_mgmt, hidden


def profile_text(iface, profile_id, name, network, ap=False):
    ssid, password, key_mgmt, hidden = network
    ssid_bytes = "".join(f"{byte};" for byte in ssid.encode("utf-8"))
    lines = [
        "[connection]", f"id={name}", f"uuid={profile_id}", "type=wifi",
        f"interface-name={iface}", "autoconnect=false", "", "[wifi]", f"ssid={ssid_bytes}",
        f"mode={'ap' if ap else 'infrastructure'}",
    ]
    if ap:
        lines += ["band=bg", "channel=6"]
    elif hidden:
        lines += ["hidden=true"]
    if key_mgmt:
        psk = password.replace("\\", "\\\\").replace(" ", r"\s")
        lines += ["", "[wifi-security]", f"key-mgmt={key_mgmt}", f"psk={psk}"]
    lines += ["", "[ipv4]", f"method={'shared' if ap else 'auto'}"]
    if ap:
        lines += [f"address1={PORTAL}/24"]
    else:
        lines += ["may-fail=false"]
    lines += ["", "[ipv6]", f"method={'disabled' if ap else 'auto'}", ""]
    return "\n".join(lines)


def load_profile(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    # Expose a complete root-only keyfile to NetworkManager in one step.
    temp = path.with_suffix(".tmp")
    try:
        with temp.open("w", encoding="utf-8") as file:
            os.fchmod(file.fileno(), 0o600)
            file.write(content)
            file.flush()
            os.fsync(file.fileno())
        temp.replace(path)
        nmcli("connection", "load", str(path))
    finally:
        temp.unlink(missing_ok=True)


def find_wifi():
    for _ in range(30):
        rows = nmcli("-t", "-f", "DEVICE,TYPE,STATE", "device", "status", check=False)
        for row in rows.splitlines():
            iface, kind, state = row.split(":", 2)
            if kind == "wifi" and state not in ("unmanaged", "unavailable"):
                return iface
        time.sleep(1)
    raise NetworkError("找不到可用的 Wi-Fi 网卡")


def station_ip(iface):
    addresses = nmcli("-g", "IP4.ADDRESS", "device", "show", iface, check=False)
    for address in addresses.splitlines():
        try:
            ip = ipaddress.IPv4Interface(address).ip
        except ValueError:
            continue
        if str(ip) != PORTAL and not ip.is_link_local:
            return str(ip)
    return ""


class Portal(ThreadingHTTPServer):
    def __init__(self, setup):
        self.setup = setup
        super().__init__((PORTAL, 80), WebHandler)

    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.setup.iface.encode() + b"\0")
        super().server_bind()


class Setup:
    def __init__(self, iface, ssid):
        self.iface = iface
        self.ssid = ssid
        self.ap_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"duckbuddy-setup:{ssid}"))
        self.ap_path = TEMP_PROFILES / f"duckbuddy-setup-{self.ap_id}.nmconnection"
        self.token = secrets.token_urlsafe(24)
        self.lock = threading.Lock()
        self.portal = None
        self.networks = {}
        self.status = {"phase": "starting", "ip": "", "message": ""}

    def state(self, phase, message="", ip="", web_result=""):
        self.status = {"phase": phase, "ip": ip, "message": message, "web_result": web_result}

    def stop_portal(self):
        if self.portal is not None:
            self.portal.shutdown()
            self.portal.server_close()
            self.portal = None

    def activate(self, profile_id):
        nmcli("connection", "up", "uuid", profile_id, "ifname", self.iface, wait=25)

    def recover_ap(self, message="", web_result=""):
        self.stop_portal()
        # Scan while the radio is available as a station, before phone clients join the AP.
        # The A733 Wi-Fi driver can reject a scan once the AP is running.
        rows = nmcli("-t", "-e", "yes", "-f", "SIGNAL,SSID,SECURITY", "device", "wifi", "list",
                     "ifname", self.iface, "--rescan", "yes", wait=8, check=False)
        self.networks = nearby_networks(rows)
        load_profile(self.ap_path, profile_text(
            self.iface, self.ap_id, "DuckBuddy setup",
            (self.ssid, AP_PASSWORD, "wpa-psk", False), ap=True,
        ))
        self.activate(self.ap_id)
        # NetworkManager can report activation before its DHCP/DNS helper finishes.
        time.sleep(1)
        active = nmcli("-t", "-f", "UUID,DEVICE", "connection", "show", "--active")
        if f"{self.ap_id}:{self.iface}" not in active.splitlines():
            raise NetworkError("热点启动后掉线，请检查 NetworkManager 的 DHCP/DNS 日志")
        self.portal = Portal(self)
        threading.Thread(target=self.portal.serve_forever, daemon=True).start()
        self.state("ap", message, web_result=web_result)
        print(f"配网热点已开启：{self.ssid}，页面 http://{PORTAL}", flush=True)

    def request_join(self, data, *, from_web=False):
        if from_web:
            choice = data.get("nearby", "manual")
            if choice.startswith("ssid:"):
                ssid = choice[5:]
                if ssid not in self.networks:
                    raise ValueError("这个 Wi-Fi 已不在列表中，请重新选择或手动输入")
                data = {**data, "ssid": ssid, "security": self.networks[ssid][1], "hidden": False}
            elif choice != "manual":
                raise ValueError("请选择要连接的 Wi-Fi，或手动输入")
        net = validated_network(data)
        with self.lock:
            if self.status["phase"] in ("starting", "connecting"):
                return {"ok": False, "message": "正在启动或连接，请稍候。"}
            self.state("connecting")
            threading.Thread(target=self.join, args=(net, from_web), daemon=True).start()
        return {"ok": True, "phase": "connecting"}

    def join(self, net, from_web):
        # Leave time for the phone to render the acknowledgement before a single radio changes mode.
        time.sleep(2)
        self.stop_portal()
        previous = nmcli("-g", "GENERAL.CON-UUID", "device", "show", self.iface, check=False)
        profile_id = str(uuid.uuid4())
        path = WIFI_PROFILES / f"duckbuddy-home-{profile_id}.nmconnection"
        try:
            load_profile(path, profile_text(self.iface, profile_id, "DuckBuddy Wi-Fi", net))
            self.activate(profile_id)
            ip = station_ip(self.iface)
            if not ip:
                raise NetworkError("连接后未分配 IPv4 地址")
            nmcli("connection", "modify", "uuid", profile_id, "connection.autoconnect", "yes")
        except (NetworkError, OSError):
            nmcli("connection", "delete", "uuid", profile_id, check=False)
            path.unlink(missing_ok=True)
            message = "未能连接或获取 IP。请核对 Wi-Fi 名称、密码和加密方式，再试一次。"
            try:
                if previous not in ("", "--", self.ap_id):
                    try:
                        self.activate(previous)
                    except NetworkError:
                        pass
                    restored = station_ip(self.iface)
                    if restored:
                        self.state("failed", message, restored)
                        return
                self.recover_ap(message, web_result=profile_id if from_web else "")
            except (NetworkError, OSError) as error:
                self.state("error", "配网热点恢复失败。")
                print(f"配网热点恢复失败：{type(error).__name__}", flush=True)
                # Let systemd restart the service and re-create the setup network.
                os._exit(1)
            return
        self.state("connected", ip=ip, web_result=profile_id if from_web else "")
        print(f"Wi-Fi 已连接，局域网 IPv4：{ip}", flush=True)


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DuckBuddy Wi-Fi 配网</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#eef5f2;color:#172d26;font:16px/1.55 system-ui,sans-serif}
main{max-width:32rem;margin:auto;padding:2rem 1rem}section{background:white;padding:1.4rem;border:1px solid #bfd7cc;border-radius:1rem}
h1{font-size:1.65rem;margin:0 0 .5rem}p{margin:.4rem 0 1.1rem}label{display:block;margin:1rem 0 .3rem}
input,select,button{width:100%;font:inherit;padding:.7rem;border:1px solid #6b897c;border-radius:.45rem;background:white;color:#172d26}
.check input{width:auto}.alert{background:#fff5d7;padding:.7rem;border-radius:.4rem}
button{margin-top:1.15rem;background:#135f4a;border-color:#135f4a;color:white;font-weight:700}
</style>
<main><section>{content}</section></main></html>"""


class WebHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def display(self, content, code=200):
        page = PAGE.replace("{content}", content).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(page)))
        self.end_headers()
        self.wfile.write(page)
        self.wfile.flush()

    def portal_request(self):
        return self.headers.get("Host") in (PORTAL, f"{PORTAL}:80")

    def do_GET(self):
        if not self.portal_request() or self.path != "/":
            self.send_error(404)
            return
        setup = self.server.setup
        notice = setup.status["message"]
        alert = f'<p class="alert">{html.escape(notice)}</p>' if notice else ""
        options = "".join(
            f'<option value="ssid:{html.escape(ssid)}" data-open="{int(security == "nopass")}">'
            f'{html.escape(ssid)} · {signal}%</option>'
            for ssid, (signal, security) in setup.networks.items()
        )
        hint = "" if options else "<p>暂未搜到可选的 Wi-Fi，可以手动输入网络名称。</p>"
        self.display(f"""<h1>DuckBuddy Wi-Fi 配网</h1>{alert}
<p>选择鸭子要连接的 Wi-Fi。手机提示当前热点无法上网时，请保持连接。</p>{hint}
<form method="post" action="/join" autocomplete="off">
<input type="hidden" name="token" value="{setup.token}">
<label for="nearby">附近的 Wi-Fi</label><select id="nearby" name="nearby" required>
<option value="" disabled {"selected" if options else ""}>请选择网络</option>{options}
<option value="manual" {"selected" if not options else ""}>手动输入网络名称</option>
</select>
<div id="manual-network">
<label for="ssid">Wi-Fi 名称</label><input id="ssid" name="ssid" autocapitalize="none">
<label for="security">加密方式</label><select id="security" name="security">
<option value="WPA">WPA / WPA2（通常选这个）</option><option value="SAE">WPA3 专用</option>
<option value="nopass">无密码</option></select>
<label class="check"><input type="checkbox" name="hidden" value="1"> 隐藏的 Wi-Fi</label></div>
<div id="network-password"><label for="password">Wi-Fi 密码</label>
<input id="password" type="password" name="password" autocomplete="new-password"></div>
<button type="submit">连接 Wi-Fi</button></form>
<script>
const nearby = document.getElementById("nearby");
const security = document.getElementById("security");
const update = () => {{
  const manual = nearby.value === "manual";
  const open = manual ? security.value === "nopass" : nearby.selectedOptions[0]?.dataset.open === "1";
  document.getElementById("manual-network").hidden = !manual;
  document.getElementById("ssid").required = manual;
  document.getElementById("network-password").hidden = open;
}};
nearby.addEventListener("change", update);
security.addEventListener("change", update);
update();
</script>""")

    def do_POST(self):
        if not self.portal_request() or self.path != "/join":
            self.send_error(404)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 4096:
                raise ValueError("表单长度无效")
            body = self.rfile.read(size).decode("utf-8")
            form = {key: values[0] for key, values in parse_qs(body, keep_blank_values=True).items()}
            if not secrets.compare_digest(form.get("token", "").encode(), self.server.setup.token.encode()):
                self.send_error(403)
                return
            result = self.server.setup.request_join(form, from_web=True)
            if not result["ok"]:
                raise ValueError(result["message"])
        except ValueError as error:
            self.display(f'<h1>请检查输入</h1><p>{html.escape(str(error))}</p><a href="/">返回配网</a>', 400)
            return
        self.display("""<h1>正在连接 Wi-Fi</h1>
<p>热点会暂时消失。连接成功后，鸭子会保持在新 Wi-Fi 上。有扬声器时可以说「豆豆，IP 是多少」，
也可以在路由器设备列表里找 <strong>duck-</strong> 开头的设备。</p>
<p>如果连接失败，约一分钟内 DuckBuddy 热点会重新出现，重新连接后再试。</p>""")


class VoiceHandler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            data = json.loads(self.rfile.readline(4097))
            op = data.get("op")
            setup = self.server.setup
            if op == "status":
                result = {"ok": True, **setup.status}
            elif op == "join":
                result = setup.request_join(data)
            else:
                result = {"ok": False, "message": "未知操作"}
        except (AttributeError, TypeError, ValueError):
            result = {"ok": False, "message": "配网参数无效"}
        self.wfile.write(json.dumps(result, ensure_ascii=False).encode() + b"\n")


def main():
    os.umask(0o077)
    nmcli("radio", "wifi", "on")
    iface = find_wifi()
    suffix = Path("/etc/machine-id").read_text().strip()[-6:].upper()
    setup = Setup(iface, f"DuckBuddy-{suffix}")
    channel = RUNTIME / "setup.sock"
    channel.unlink(missing_ok=True)
    with socketserver.ThreadingUnixStreamServer(str(channel), VoiceHandler) as server:
        server.setup = setup
        channel.chmod(0o660)
        profiles = nmcli("-t", "-f", "TYPE,AUTOCONNECT", "connection", "show", check=False).splitlines()
        saved_wifi = any(row in ("802-11-wireless:yes", "wifi:yes") for row in profiles)
        # Give a saved station profile time to scan and get DHCP before taking the radio for AP.
        for _ in range(30 if saved_wifi else 3):
            ip = station_ip(iface)
            if ip:
                setup.state("connected", ip=ip)
                break
            time.sleep(1)
        else:
            setup.recover_ap()
        server.serve_forever()


if __name__ == "__main__":
    main()
