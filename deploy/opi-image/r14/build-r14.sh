#!/usr/bin/env bash
# Usage: sudo bash build-r14.sh r8.img robotd-r14-aarch64 output.img
set -euo pipefail
if [[ $# -ne 3 ]]; then
    echo '用法：build-r14.sh r8.img robotd-r14-aarch64 output.img' >&2
    exit 2
fi

here=$(cd -- "$(dirname -- "$0")" && pwd)
repo=$(cd -- "$here/../../.." && pwd)
base=$(realpath -- "$1")
runtime=$(realpath -- "$2")
output=$(realpath -m -- "$3")
pending="$output.partial"
test "$base" != "$output"
test ! -e "$output"
test ! -e "$output.sha256"
test ! -e "$pending"
policies="$repo/training/policies"
walk="$policies/hd1910_walk_kissqy_m6_p5_8498.onnx"
stand="$policies/hd1910_stand_noise10_9997.onnx"
sitstand="$policies/hd1910_sitstand_kp16_6998.onnx"
sha256sum -c <<EOF
41909e80a23ab28b457135dee09aebe56c54b7d01d9565dde74244eb7673b8e4  $base
05db51807433fbb54f167066bc7751ccd83b2580e90db4902d5ac55e6144ebf4  $runtime
44f77173a3ed14b8c53a384f9d69ae924f8fa6519338270c2b9f7b99b347b62c  $walk
dd90c0de95b1d865ea267869d78f44caa2be4f987eeacfdb1bce3a4c82d757b7  $stand
9131319b735d18a626804dc6cbc9f83cd33e963513a8edb3db7fb5ac893af698  $sitstand
EOF
cp --sparse=always -- "$base" "$pending"

root=$(mktemp -d)
loop=
cleanup() {
    if mountpoint -q "$root"; then umount "$root"; fi
    if [[ -n $loop ]]; then losetup -d "$loop"; fi
    rmdir "$root"
}
trap cleanup EXIT
offset=$(sfdisk --json "$pending" | python3 -c '
import json, sys
table = json.load(sys.stdin)["partitiontable"]
assert len(table["partitions"]) == 1
print(table["sectorsize"] * table["partitions"][0]["start"])
')
loop=$(losetup --find --show --offset "$offset" "$pending")
mount "$loop" "$root"
grep -qx DUCKBUDDY_IMAGE_REVISION=r8 "$root/etc/duckbuddy-image-release"
home="$root/home/orangepi"
robot="$home/robot"
owner=$(stat -c %u "$robot/robotd")
group=$(stat -c %g "$robot/robotd")
install_board() {
    install -D -o "$owner" -g "$group" "$@"
}
units="$home/.config/systemd/user"
setup="$units/duck-wifi-setup.service"
grep -qx 'ExecStart=/usr/bin/python3 %h/.local/lib/duck-wifi-setup.py' "$setup"
grep -qx 'ExecStart=/usr/bin/python3 %h/duckweb/duckweb.py' "$units/duckweb-dev.service"
grep -qx 'ExecStart=/usr/bin/python3 %h/visiond/visiond.py' "$units/visiond-dev.service"
install -D -m 644 "$here/hotspot/duck-hotspot-setup.py" "$root/usr/local/lib/duck-hotspot-setup.py"
install -D -m 644 "$here/hotspot/duck-hotspot-setup.service" "$root/etc/systemd/system/duck-hotspot-setup.service"
# NetworkManager runs its own dnsmasq for the hotspot.
rm -f -- "$root/etc/systemd/system/multi-user.target.wants/dnsmasq.service"
systemctl --root "$root" enable duck-hotspot-setup.service
install_board -m 644 "$here/hotspot/duck-hotspot-voice.py" "$home/.local/lib/duck-hotspot-voice.py"
sed -i 's|/duck-wifi-setup.py$|/duck-hotspot-voice.py|' "$setup"
for key in wifi-ready hotspot-ready; do
    install_board -m 644 "$here/hotspot/$key.wav" "$home/lifed/voice/$key.wav"
done
for file in duckconsole.py robot_control.py console.html console.css console.js; do
    install_board -m 644 "$here/console/$file" "$home/duckweb/$file"
done
install_board -m 755 "$here/console/duckweb.py" "$home/duckweb/duckweb.py"
install_board -m 644 "$here/console/vision_console.py" "$home/visiond/vision_console.py"
sed -i 's|/duckweb/duckweb.py$|/duckweb/duckconsole.py|' "$units/duckweb-dev.service"
sed -i 's|/visiond/visiond.py$|/visiond/vision_console.py|' "$units/visiond-dev.service"

install_board -m 755 "$robot/robotd" "$robot/robotd-runtime/hl2915/robotd"
install_board -m 755 "$runtime" "$robot/robotd-runtime/hd1910/robotd"
install_board -m 644 "$walk" "$robot/policies/$(basename "$walk")"
install_board -m 644 "$stand" "$robot/policies/hd1910_stand_noise10_9997.onnx"
install_board -m 644 "$sitstand" "$robot/policies/$(basename "$sitstand")"
install_board -m 644 "$here/walk-hd1910.toml" "$robot/walk-hd1910.toml"
install -m 755 "$here/duck-model" "$root/usr/local/bin/duck-model"

# Switch as the image builder, and remove only the backup created for this fresh template.
if compgen -G "$robot/duck.toml.bak-*" >/dev/null; then
    echo 'r8 基础镜像包含本机标定备份' >&2
    exit 1
fi
HOME="$home" sh "$root/usr/local/bin/duck-model" hd1910
rm -- "$robot"/duck.toml.bak-*
sed -i '2,9c\
# r14 默认 HD-1910（2S）。可在工作台切换舵机型号。\
# 关节 ID、方向、零位与 IMU 安装方向须在每台机器上核对。\
# 运动策略在完成标定后由 duck-services on 加载。' "$robot/duck.toml"
chown "$owner:$group" "$robot/duck.toml"
chown -h "$owner:$group" "$robot/robotd" "$robot/walk.toml"
printf 'DUCKBUDDY_IMAGE_REVISION=r14\n' > "$root/etc/duckbuddy-image-release"

python3 -B - "$root" "$here/hd1910-pd.toml" <<'PY'
import sys
import tomllib
from pathlib import Path

root = Path(sys.argv[1])
home = root / "home/orangepi"
robot = home / "robot"
units = home / ".config/systemd/user"
buscfg = tomllib.loads((robot / "duck.toml").read_text())
bus, battery = buscfg["bus"], buscfg["battery"]
profile = tomllib.loads(Path(sys.argv[2]).read_text())
assert all(bus[key] == value for key, value in profile["bus"].items())
assert battery == profile["battery"]
assert bus["port"] == "/dev/ttyS7" and bus["current_ma_per_count"] == 0.65
assert bus["joint_ids"] == list(range(1, 16)) and bus["joint_offsets"] == [0] * 15
assert bus["imu_pitch_trim_deg"] == bus["imu_roll_trim_deg"] == 0
assert battery["cells"] == [2] and battery["max_volts"] == 8.4
assert [bus[key] for key in ("position_kp", "position_kp_head", "position_kp_neck_pitch",
                            "position_kp_knees", "position_kd")] == [8, 5, 10, 10, 0]
assert bus["joint_kp_overrides"] == {"head_pitch": 8}
assert bus["current_limit_ma"] == 1300 and bus["hd1910_kissqy_profile"] and bus["hd1910_kissqy_use_local_goal"]
assert "policy" not in buscfg
assert (robot / "robotd").readlink() == Path("robotd-runtime/hd1910/robotd")
assert (robot / "walk.toml").readlink() == Path("walk-hd1910.toml")
for model in ("hd1910", "hl2915"):
    policy = tomllib.loads((robot / f"walk-{model}.toml").read_text())["policy"]
    for slot in ("walk", "stand", "sitstand"):
        assert (root / policy[slot].lstrip("/")).is_file()
for service in ("robotd-dev", "padd-dev", "lifed-dev", "voiced-dev", "tofd-dev", "visiond-dev"):
    assert not (units / "default.target.wants" / f"{service}.service").is_symlink(), service
assert "duckconsole.py" in (units / "duckweb-dev.service").read_text()
assert "vision_console.py" in (units / "visiond-dev.service").read_text()
assert (units / "default.target.wants/duck-wifi-setup.service").is_symlink()
assert (units / "default.target.wants/duckweb-dev.service").is_symlink()
assert not (home / "duckweb/persona.html").exists()
for source in (home / "duckweb/duckconsole.py", home / "visiond/vision_console.py",
               home / ".local/lib/duck-hotspot-voice.py", root / "usr/local/lib/duck-hotspot-setup.py"):
    compile(source.read_bytes(), str(source), "exec")
assert (root / "etc/systemd/system/multi-user.target.wants/duck-hotspot-setup.service").is_symlink()
assert not any((root / "etc/NetworkManager/system-connections").iterdir())
assert not (home / ".ssh/authorized_keys").exists()
assert not (root / "root/.ssh/authorized_keys").exists()
assert not (root / "var/lib/duck/firstboot.done").exists()
assert not (root / "etc/machine-id").read_bytes().strip()
print("r14 配置、三槽模型、标定门槛、后台及配网核对通过")
PY

sync
umount "$root"
e2fsck -fn "$loop"
losetup -d "$loop"
loop=
rmdir "$root"
trap - EXIT
digest=$(sha256sum "$pending" | cut -d ' ' -f 1)
mv -- "$pending" "$output"
printf '%s  %s\n' "$digest" "$(basename "$output")" > "$output.sha256"
