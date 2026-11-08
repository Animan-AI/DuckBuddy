#!/usr/bin/python3
"""Local BlueZ pairing for classic Bluetooth gamepads."""
import json
import time
from pathlib import Path

import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

BLUEZ = "org.bluez"
ADAPTER = BLUEZ + ".Adapter1"
DEVICE = BLUEZ + ".Device1"
AGENT = BLUEZ + ".Agent1"
PROPS = "org.freedesktop.DBus.Properties"
REQUEST = Path("/home/orangepi/.duck-pair-gamepad")
STATUS = Path("/run/duck-gamepad/status.json")
WINDOW = 90


def status(state, message, request=None):
    print(message, flush=True)
    tmp = STATUS.with_suffix(".tmp")
    tmp.write_text(json.dumps(dict(state=state, msg=message, request=request)),
                   encoding="utf-8")
    tmp.replace(STATUS)


def is_gamepad(properties):
    cod = int(properties.get("Class", 0))
    return cod & 0x1F00 == 0x0500 and cod & 0x3C in (0x04, 0x08)


class PairingAgent(dbus.service.Object):
    def __init__(self, bus, path, device_path):
        super().__init__(bus, path)
        self.device_path = device_path

    @dbus.service.method(AGENT, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        if device != self.device_path:
            raise dbus.DBusException("Unexpected device", name="org.bluez.Error.Rejected")

    @dbus.service.method(AGENT, in_signature="", out_signature="")
    def Release(self):
        pass

    @dbus.service.method(AGENT, in_signature="", out_signature="")
    def Cancel(self):
        pass


def devices(objects, adapter_path):
    return {path: props[DEVICE] for path, props in objects.items()
            if DEVICE in props and props[DEVICE].get("Adapter") == adapter_path}


def choose(manager, adapter, adapter_path):
    adapter.SetDiscoveryFilter({"Transport": "bredr"})
    adapter.StartDiscovery()
    try:
        deadline = time.monotonic() + WINDOW
        first_seen = None
        while time.monotonic() < deadline:
            candidates = [path for path, props in devices(
                manager.GetManagedObjects(), adapter_path).items()
                if is_gamepad(props) and not props.get("Paired") and "RSSI" in props]
            if len(candidates) > 1:
                raise RuntimeError("发现多只未配对手柄，请只让目标手柄快闪后重试")
            if candidates:
                path = candidates[0]
                if first_seen is None or first_seen[0] != path:
                    first_seen = (path, time.monotonic())
                if time.monotonic() - first_seen[1] >= 4:
                    return path
            else:
                first_seen = None
            time.sleep(0.5)
        raise RuntimeError("没有找到快闪的蓝牙手柄，请重新进入配对模式")
    finally:
        adapter.StopDiscovery()


def pair(bus, path):
    agent_path = "/org/duckbuddy/gamepad"
    agent = PairingAgent(bus, agent_path, path)
    manager = dbus.Interface(bus.get_object(BLUEZ, "/org/bluez"), BLUEZ + ".AgentManager1")
    manager.RegisterAgent(agent_path, "NoInputNoOutput")
    try:
        obj = bus.get_object(BLUEZ, path)
        device = dbus.Interface(obj, DEVICE)
        props = dbus.Interface(obj, PROPS)
        loop = GLib.MainLoop()
        errors = []

        def failed(error):
            errors.append(error)
            loop.quit()

        device.Pair(reply_handler=loop.quit, error_handler=failed, timeout=35)
        loop.run()
        if errors:
            raise errors[0]
        props.Set(DEVICE, "Trusted", dbus.Boolean(True))
        if props.Get(DEVICE, "Connected"):
            return "手柄已配对并连接"
        return "手柄已配对，按 Home 键回连"
    finally:
        manager.UnregisterAgent(agent_path)
        agent.remove_from_connection()


def run(request, automatic):
    try:
        bus = dbus.SystemBus()
        manager = dbus.Interface(bus.get_object(BLUEZ, "/"), "org.freedesktop.DBus.ObjectManager")
        for _ in range(15):
            objects = manager.GetManagedObjects()
            adapter_path = next((path for path, props in objects.items() if ADAPTER in props), None)
            if adapter_path:
                break
            time.sleep(1)
        else:
            raise RuntimeError("找不到蓝牙适配器")
        obj = bus.get_object(BLUEZ, adapter_path)
        props = dbus.Interface(obj, PROPS)
        if not props.Get(ADAPTER, "Powered"):
            props.Set(ADAPTER, "Powered", dbus.Boolean(True))
        existing = devices(objects, adapter_path)
        if automatic and any(is_gamepad(p) and p.get("Paired") for p in existing.values()):
            status("done", "已有配对手柄，按 Home 键回连")
            return
        status("scanning", "正在查找手柄，请让手柄进入快闪配对模式", request)
        path = choose(manager, dbus.Interface(obj, ADAPTER), adapter_path)
        status("pairing", "已发现手柄，正在配对", request)
        status("done", pair(bus, path), request)
    except (dbus.DBusException, RuntimeError) as error:
        status("failed", f"配对失败：{error}", request)


def main():
    DBusGMainLoop(set_as_default=True)
    run(None, automatic=True)
    while True:
        try:
            request = REQUEST.lstat().st_mtime_ns
            REQUEST.unlink()
        except FileNotFoundError:
            time.sleep(1)
            continue
        run(str(request), automatic=False)


if __name__ == "__main__":
    main()
