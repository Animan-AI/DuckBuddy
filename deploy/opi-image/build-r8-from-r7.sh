#!/usr/bin/env bash
# Usage: sudo bash build-r8-from-r7.sh r7.img r8.img [--prepared-copy]
set -euo pipefail
if [ "$#" -lt 2 ] || [ "$#" -gt 3 ] || { [ "$#" -eq 3 ] && [ "$3" != "--prepared-copy" ]; }; then
    echo "用法：build-r8-from-r7.sh r7.img r8.img [--prepared-copy]" >&2
    exit 2
fi

here=$(cd -- "$(dirname -- "$0")" && pwd)
base=$(realpath -- "$1")
output=$(realpath -m -- "$2")
pending="$output.partial"
test "$base" != "$output"
test ! -e "$output"
test ! -e "$output.sha256"
base_hash=$(awk 'NR == 1 {print $1}' "$base.sha256")
[[ "$base_hash" =~ ^[a-f0-9]{64}$ ]]
if [ "${3:-}" != "--prepared-copy" ]; then
    test ! -e "$pending"
    cp --sparse=always -- "$base" "$pending"
fi
printf '%s  %s\n' "$base_hash" "$pending" | sha256sum -c -

root=$(mktemp -d)
loop=
cleanup() {
    if mountpoint -q "$root"; then
        umount "$root" || return
    fi
    if [ -n "$loop" ]; then
        losetup -d "$loop"
    fi
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
board="$root/home/orangepi"
settings="$board/duckbrain/ai_settings.py"
wants="$board/.config/systemd/user/default.target.wants"
owner=$(stat -c '%u:%g' "$settings")

install -m 644 -o "${owner%:*}" -g "${owner#*:}" "$here/ai_settings.py" "$settings"
printf 'DUCKBUDDY_IMAGE_REVISION=r8\n' > "$root/etc/duckbuddy-image-release"

echo "核对预设、首次开机和标定状态"
python3 -B - "$board" <<'PY'
import json
import sys
import tomllib
from pathlib import Path

board = Path(sys.argv[1])
sys.path.insert(0, str(board / "duckbrain"))
import ai_settings

ai_settings.CONFIG = board / ".config/duck"
raw = ai_settings.read_saved()
assert "credentials" not in raw
assert all("api_key" not in raw[name] for name in ("model", "tts"))
settings = ai_settings.public()
assert settings["model"]["provider"] == "zhipu"
assert settings["model"]["model"] == "glm-4.6v-flash"
assert settings["tts"]["provider"] == "edge"
assert settings["tts"]["voice"] == "zh-CN-XiaoxiaoNeural"
assert not any(settings[name]["has_key"] for name in ("model", "tts"))
config = tomllib.loads((board / "robot/duck.toml").read_text())
assert config["bus"]["joint_offsets"] == [0] * 15 and "policy" not in config
assert {"walk-hl2915.toml", "walk-hd1910.toml"} <= {path.name for path in (board / "robot").glob("walk-*.toml")}
for source in (board / "duckweb/duckweb.py", board / "duckbrain/openrouter_brain.py",
               board / "lifed/lifed.py", board / "lifed/stream_reply.py"):
    compile(source.read_bytes(), str(source), "exec")
print("默认模型、TTS、零位和 Python 语法核对通过")
PY

for service in duck-wifi-setup duckbrain-openrouter duckweb-dev; do
    test -L "$wants/$service.service"
done
for service in robotd-dev padd-dev lifed-dev voiced-dev tofd-dev visiond-dev; do
    test ! -e "$wants/$service.service"
done
test ! -e "$root/boot/orangepi_first_run.txt"
test ! -e "$root/var/lib/duck/firstboot.done"
test ! -s "$root/etc/machine-id"
test ! -s "$board/.ssh/authorized_keys"
test ! -s "$root/root/.ssh/authorized_keys"
test -z "$(find "$board/.config/duck" -type f -name '*.key' -print)"
test -z "$(find "$root/etc/NetworkManager/system-connections" -type f -print)"

sync
umount "$root"
e2fsck -fn "$loop"
losetup -d "$loop"
loop=
rmdir "$root"
trap - EXIT
digest=$(sha256sum "$pending" | cut -d ' ' -f 1)
mv -- "$pending" "$output"
printf '%s  %s\n' "$digest" "$(basename -- "$output")" > "$output.sha256"
ls -l -- "$output" "$output.sha256"
