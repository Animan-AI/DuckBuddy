#!/bin/bash
# Xiaobai's brain over frp (2026-10-15): run ON THE V100 BOX, as root (sudo bash duckbrain-stcp-setup.sh).
#
# The robot reaches the brain (duckbrain: the local 27B + CosyVoice, on 127.0.0.1:8210 here) through
# the user's Aliyun frps, at home and outdoors alike, with an frp *stcp* proxy: nothing is opened to
# the internet — only a client holding the same secret key can visit it. The robot's kernel has no TUN
# and no WireGuard, which rules the VPNs out; frp runs in user space.
#
# What this does, idempotently:
#   1. backs up /etc/frp/frpc.toml;
#   2. makes a secret key for the proxy (/etc/frp/duckbrain.key, root 600) unless one exists;
#   3. adds the stcp proxy "duckbrain" -> 127.0.0.1:8210 unless it is there; checks the config;
#   4. restarts frpc (the other proxies drop for a few seconds);
#   5. writes the robot's frpc config to ~SUDO_USER/duckbrain-robot-frpc.toml (600): this box's
#      connection settings (server, port, auth, transport — no log / admin UI / proxies) plus a
#      visitor that binds the brain on the robot's 127.0.0.1:8210.
# Nothing secret is printed. Copy step 5's file to the robot's ~/.config/frp/frpc.toml (600).
set -euo pipefail
CONF=/etc/frp/frpc.toml
KEY=/etc/frp/duckbrain.key
PORT=8210
FRPC=$(command -v frpc || echo /usr/local/bin/frpc)
OWNER=${SUDO_USER:-meng}
OUT=$(getent passwd "$OWNER" | cut -d: -f6)/duckbrain-robot-frpc.toml

[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
[ -f "$CONF" ] || { echo "no $CONF"; exit 1; }

cp -p "$CONF" "$CONF.bak-$(date +%Y%m%d-%H%M%S)"
if [ ! -s "$KEY" ]; then
    umask 077
    openssl rand -hex 24 > "$KEY"
    chmod 600 "$KEY"
    echo "made a new secret key ($KEY)"
fi
SECRET=$(cat "$KEY")

if grep -Eq '^\s*name\s*=\s*"duckbrain"' "$CONF"; then
    echo "proxy duckbrain already in $CONF"
else
    cat >> "$CONF" <<EOF

# Xiaobai's brain (microduck deploy/frp/duckbrain-stcp-setup.sh, $(date +%F)): private, stcp only.
[[proxies]]
name = "duckbrain"
type = "stcp"
secretKey = "$SECRET"
localIP = "127.0.0.1"
localPort = $PORT
EOF
    echo "added proxy duckbrain -> 127.0.0.1:$PORT"
fi
"$FRPC" verify -c "$CONF"
systemctl restart frpc
sleep 2
systemctl is-active frpc

# The robot's config: this box's connection settings, nothing else, and the visitor.
umask 077
awk '
    /^[[:space:]]*\[\[/ { skip = 1; next }                       # [[proxies]] / [[visitors]]: not copied
    /^[[:space:]]*\[[^\[]/ {                                       # a [table]
        t = $0; gsub(/[[:space:]\[\]]/, "", t)
        skip = (t ~ /^(log|webServer)/)
        if (!skip) print
        next
    }
    skip { next }
    /^[[:space:]]*(log\.|webServer\.|includes|start)/ { next }
    { print }
' "$CONF" > "$OUT"
cat >> "$OUT" <<EOF

# Xiaobai's brain on the robot's 127.0.0.1:$PORT (stcp visitor of "duckbrain" on the V100 box).
[[visitors]]
name = "duckbrain-visitor"
type = "stcp"
serverName = "duckbrain"
secretKey = "$SECRET"
bindAddr = "127.0.0.1"
bindPort = $PORT
EOF
chown "$OWNER" "$OUT"
chmod 600 "$OUT"
echo "robot config written: $OUT (600) — copy it to the robot's ~/.config/frp/frpc.toml"
