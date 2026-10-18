#!/bin/sh
# The login-shell files: what an operator sees, and can type, the moment they ssh in — the
# `robotctl` completions, a banner naming the release actually live, and the robot's own name in
# the prompt.
#
# **A script rather than three functions in `install.sh`, because `install.sh` runs once.** It sets
# up a board and is never seen again, and the update path cannot reach it — it is not in the
# artifact. All three of these were added to `install.sh` alone, so none of them is on any board
# that has only ever updated: `docs/design/updater-design.md` §9.1 in its fourth shape, found by
# ssh'ing to a robot whose release contained the prompt feature and getting the stock prompt.
# `install.sh` and `hooks/postinstall` both run this now, which is the shape `setup-gstreamer.sh`
# already had.
#
# Idempotent, and meant to be run on every update: each step rewrites its file, which is what
# makes a board provisioned two months ago end up with what a board provisioned today has.
#
# Nothing here is needed for the robot to run. A board with no /etc/profile.d, no motd machinery
# or no bash-completion skips that step; `hooks/postinstall` treats a failure as cosmetic, because
# a shell prompt is not worth rolling an update back for.
set -eu

say() { printf 'setup-login: %s\n' "$*"; }

# What the robot is running, and whether it is working, printed at every ssh login.
#
# This exists because of a specific failure that cost an afternoon: a dev board silently reverted a
# branch build to the stable release — `updaterd`'s cross-boot health gate, correctly, since a bench
# board with no servo power can never report healthy — and nothing said so. Every command afterwards
# ran against code nobody had asked for, and the symptom was a feature that "did not work".
#
# So the two facts worth having before typing anything are on screen at login: the release actually
# live, and whether the last update stuck.
#
# An motd drop-in rather than /etc/profile.d: it runs once per ssh login rather than per shell, it is
# the mechanism the image already uses for this kind of thing, and a slow or broken robotctl there
# cannot wedge an interactive shell.
#
# Exits 0 on every path, always. A banner that fails must never be the reason a login is noisy or
# slow — that is how people start disabling motd.
install_login_banner() {
    if [ ! -d /etc/update-motd.d ]; then
        # No motd machinery on this image. Not worth building one for a banner.
        return 0
    fi

    cat > /etc/update-motd.d/40-robot <<'BANNER'
#!/bin/sh
# What this robot is running, and whether it is working. Installed by scripts/install.sh.
command -v robotctl >/dev/null 2>&1 || exit 0

live="$(readlink /opt/robot/daemon/current 2>/dev/null | sed 's|releases/||')"
[ -n "$live" ] || exit 0

# First line only: `robotctl health` leads with the whole-robot verdict, and the detail below it
# belongs to someone who has decided to look.
verdict="$(robotctl health 2>/dev/null | head -1 | sed 's/^robot *//')"
[ -n "$verdict" ] || verdict="not answering"

printf '\nrobot   %s — %s\n' "$live" "$verdict"

# The rollback that prompted this banner. Grepped rather than parsed: there is no jq on this image,
# and the only question is whether the last attempt ended that way.
if robotctl update status 2>/dev/null | grep -q rolled_back; then
    printf '        the last update was ROLLED BACK — robotctl update status\n'
fi
exit 0
BANNER
    chmod 755 /etc/update-motd.d/40-robot
    say "wrote /etc/update-motd.d/40-robot, so a login says what is running"
}

# The robot's name beside the hostname in the prompt: `microduck@radxa-zero3 (coincoin):~$`.
#
# Every board flashed from one image says `microduck@radxa-zero3`, so three ssh windows to
# three ducks are three identical prompts — and a command typed into the wrong one is exactly
# the failure the per-board *name* exists to prevent (configd/src/identity.rs). The prompt is
# where an operator looks before typing; put the name there.
#
# A profile.d snippet rather than editing anyone's .bashrc: it needs no per-user provisioning
# and an update replaces it cleanly. The wrinkle is ordering — ~/.bashrc sets PS1 *after*
# /etc/profile.d runs, so the snippet cannot edit PS1 directly; it hooks PROMPT_COMMAND and
# injects the name at the first prompt instead, when PS1 has settled.
install_name_prompt() {
    if [ ! -d /etc/profile.d ]; then
        # Not the image we know. A prompt nicety is not worth creating login machinery for.
        return 0
    fi

    cat > /etc/profile.d/robot-name-prompt.sh <<'PROMPT'
# The robot's name beside the hostname in the prompt. Installed by scripts/install.sh.
#
# Prompt surgery below is bash syntax, and /etc/profile.d is also read by plain sh.
[ -n "$BASH_VERSION" ] || return 0

# The same name the robot advertises over BLE: configd's store where someone has renamed it,
# else `duck-xxxx` derived exactly as configd derives it — the first four hex characters of
# the SoC serial's SHA-256 (configd/src/identity.rs says why it is not the Bluetooth address).
# Read from the file rather than asked of configd, so the prompt works while daemons are down
# — which is much of what anyone is ssh'd in to deal with.
_robot_name="$(sed -n 's/.*"name": *"\(.*\)".*/\1/p' /var/lib/robot/config/config.json 2>/dev/null)"
if [ -z "$_robot_name" ] && [ -r /proc/device-tree/serial-number ]; then
    _robot_serial="$(tr '\0' '\n' < /proc/device-tree/serial-number 2>/dev/null | head -n 1)"
    [ -n "$_robot_serial" ] && _robot_name="duck-$(printf '%s' "$_robot_serial" | sha256sum | cut -c1-4)"
fi
unset _robot_serial

# A name reaches prompt expansion, where $(...) and backticks *execute*. configd only strips
# control characters, so strip the rest here.
_robot_name="$(printf '%s' "$_robot_name" | tr -d '\\$`')"

if [ -n "$_robot_name" ]; then
    # Global substitution, not first-match: the xterm-title prefix Debian's .bashrc builds
    # contains its own \h before the visible one, and naming the window too is a feature —
    # it is the other place an operator tells sessions apart.
    _robot_name_inject() {
        case "$PS1" in *"($_robot_name)"*) return ;; esac
        PS1="${PS1//\\h/\\h ($_robot_name)}"
    }
    PROMPT_COMMAND="_robot_name_inject${PROMPT_COMMAND:+;$PROMPT_COMMAND}"
fi
PROMPT
    chmod 644 /etc/profile.d/robot-name-prompt.sh
    say "wrote /etc/profile.d/robot-name-prompt.sh, so the prompt names the duck"
}

# Tab-completion for `robotctl`, so an operator on a board they are debugging over a flaky
# ssh link types `update apply` rather than remembering it.
#
# A *loader* rather than a generated snapshot: it asks the live binary for its completions
# at shell start, so after an update adds a subcommand the completions describe the release
# now installed, with no reinstall step and nothing for a rollback to leave behind. The cost
# is one process per interactive bash — the same shape `bash-completion` itself uses for
# dynamic completers.
#
# Written by hand rather than shipped in the artifact for the same reason: the artifact
# would then have to carry a file whose only content is this indirection.
install_completions() {
    if [ ! -d /etc/bash_completion.d ]; then
        # No bash-completion on this image. Not worth installing a directory nothing reads.
        return 0
    fi
    cat > /etc/bash_completion.d/robotctl <<'EOF'
# Generated by the robot daemon installer. Asks robotctl for its own completions, so this
# file never needs regenerating when an update changes the command tree.
#
# `eval` rather than `source <(robotctl …)`: process substitution is not reliable in bash
# 3.2, and stderr is discarded so a rolled-back release that predates `robotctl
# completions` leaves the shell with no completions rather than a usage error on every
# login.
if command -v robotctl >/dev/null 2>&1; then
    eval "$(robotctl completions bash 2>/dev/null)"
fi
EOF
    chmod 644 /etc/bash_completion.d/robotctl
}

install_completions
install_login_banner
install_name_prompt
