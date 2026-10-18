# `duckctl` — every command

Talk to a robot from a laptop, with no network and no ssh. It is the phone app's stand-in, and the
way to reach a robot that has never seen a wifi network.

Bluetooth LE is how it gets there today, and the name deliberately does not say so: `mediad` gives
a robot a second transport that reaches a different set of methods, so the tool is named for what
it talks to rather than for the radio it currently uses. It was `duck-btctl` while BLE was the only
answer.

**Never on a robot.** Nothing in a release depends on it — `robotctl` is the tool that ships, and
[`cheatsheet.md`](cheatsheet.md) has its commands, most of which have a `duckctl` equivalent
below.

## Getting it

Run it from a clone of this repo:

```bash
cargo run -q -p duckctl -- --name <robot-name> info
```

Or install it once, at the cost of a snapshot that no longer follows the branch:

```bash
cargo install --path duckctl
```

```bash
duckctl --name <robot-name> info
```

Every command below is written in the installed form. Prefix it with
`cargo run -q -p duckctl --` to run it from the clone instead.

This tool used to install itself as `btctl`. If `which btctl` still finds one, it is a build from
whenever you installed it and it will never change again:

```bash
cargo uninstall btd --bin btctl
```

## Finding a robot

```bash
duckctl scan
```

```
1 robot(s) advertising the duck service:
  aa:bb:cc:dd:ee:ff duck-c51b — 192.168.1.42, 1 service(s)  ← DUCK_ROBOT

7 other device(s) in range, not listed. …
```

Robots only, with everything else in radio range counted rather than listed. `--verbose` expands
that list, and it is worth reading when the robot you want is not in the first one.

Each robot broadcasts its IPv4 address, so this is also where the address to ssh to comes from. No
connection is made and no PIN is needed. `no address` on the line means the robot is not on a
network; a line with no address at all means a release from before robots broadcast one, and
`duckctl wifi status` still reports it.

The SSID is not in the listing — it does not fit in an advertisement. `duckctl wifi status` has
it, along with the signal and both addresses.

For the address on its own:

```bash
ssh radxa@$(duckctl ip)
```

`ip` prints the address and nothing else, so it substitutes. It reads the advertisement, so it
connects to nothing, needs no PIN, and takes about a second — and the answer is not stale: `btd`
re-reads the address every five seconds and re-advertises when it moves.

A robot bonded to this machine often stops advertising the service to it, and then `ip` connects and
asks `net.status` instead. That is slower and needs the PIN, and it always answers. `--verbose` says
which of the two happened.

A robot with no network address is told what to do about it rather than reported as empty, because
the fix is over the radio and has to be — `net.connect` is refused over WebRTC by design.

A robot that has never been renamed calls itself `duck-` plus four characters derived from its
serial, so `duck-c51b`. Either half of a robot reported under two names at once — macOS shows
`radxa-zero3 [duck-c51b]` — works as `--name`.

A robot that scanned as `duck-c51b` and then, after one connection, only as `radxa-zero3` is on a
release that gave its name to the advertisement but not to the adapter, and the client cached the
adapter's. Update the robot. That does not clear what the client already cached, so clear it too:
`bluetoothctl remove <mac>` on Linux, or forget the robot in macOS Bluetooth settings.

With no name at all — no `--name`, no `DUCK_ROBOT` — the first robot found wins. With a name, two
robots answering to it is an error rather than a choice:

```
2 robots answer to "radxa-zero3": radxa-zero3, radxa-zero3
```

That happens on a board whose bootloader leaves its serial blank, because it is then named after
its hostname and every board flashed from one image has the same one. Rename one from the robot
itself and use the new name:

```bash
robotctl system set-name ducky
```

## The console

The robot serves a page that streams its camera and drives it:

```bash
duckctl open
```

Finds the robot, then opens `http://<address>:8080/` in a browser. `--print` gives the URL instead,
for a machine with no browser or for a script; `--port` for a robot started with a non-default
`mediad --web-port`.

Nothing to install and nothing to serve — `mediad` embeds the page, so a robot running that daemon
is a robot with a console.

What is on it: the camera with the link's bitrate, frame rate, loss and round trip beside it; two
pads and the keys `W`/`A`/`S`/`D` and `Q`/`E` to drive, at a gamepad's 0.3 m/s and 1.5 rad/s; a drag
on the picture to look at a point; enable, init, relax, stop and shutdown; the skills and the voice
bank as menus; and the state stream at 2 Hz beside `robot.health`, which is where a hot servo, a flat
pack or a loop running slow gets named.

`stop` zeroes the intents the page is sending. **It is not an emergency stop** — nothing in this
system cuts servo power from a browser — and the button is a plain one for that reason.

The raw JSON box, the log, and the two calls a WebRTC peer is refused are in the drawer at the
bottom. They prove the route table is being consulted rather than drive the robot.

Two ports are involved and only this one is typed: the page reaches the signalling server on 8443
itself, using the host it was served from. If the page loads and then says its signalling port did
not answer, the robot is up and something between you and 8443 is not — a firewall, most often.

The camera and the drive controls are on WebRTC and nothing else, so a robot with no network address
has no console. Join it to one over the radio first — `duckctl wifi connect` below needs no network
of its own.

## Always the same robot

Put the name in the environment instead of on every command line:

```bash
export DUCK_ROBOT=duck-c51b
```

Put that line in `~/.zshrc` to keep it. Every command below then works without `--name`:

```bash
duckctl info
```

`DUCK_PIN` does the same for `--pin`, which a robot with a PIN of its own needs on every command:

```bash
export DUCK_PIN=418299
```

For one command against a different robot, `--name` still wins:

```bash
duckctl --name duck-ffff info
```

To ignore the default for one command — a bench with somebody else's robot on it — set it to
nothing:

```bash
DUCK_ROBOT= duckctl scan
```

`scan` marks the robot `DUCK_ROBOT` names and lists it first, and every command that goes looking
for it says so before it starts scanning.

## Identity

```bash
duckctl --name <robot-name> info
```

Name, serial and uptime.

```bash
duckctl --name <robot-name> name <new-name>
```

Up to 24 characters. It takes effect within a few seconds and needs no restart, but the Mac keeps
serving the name it learned earlier, so `scan` and macOS Bluetooth settings both lag behind. Every
later command uses the new name.

A rename does not follow `DUCK_ROBOT`. The tool says so afterwards; the variable has to be changed
by hand, or every later command looks for a name that no longer answers.

```bash
duckctl --name <robot-name> reboot
```

## Wifi

```bash
duckctl --name <robot-name> wifi status
```

SSID, signal and addresses.

```bash
duckctl --name <robot-name> wifi scan
```

Takes a few seconds — the robot sweeps the radio rather than returning the previous scan.

```bash
duckctl --name <robot-name> wifi connect <ssid> --psk <passphrase>
```

Omit `--psk` for an open network. Joining disconnects the robot from the network it is on, so an ssh
session over wifi drops; that is the command working. It can take up to 45 seconds to answer.

```bash
duckctl --name <robot-name> wifi forget <ssid>
```

## Is it alright

```bash
duckctl --name <robot-name> health
```

Whether the control loop is healthy.

```bash
duckctl --name <robot-name> status
```

The version handshake and the update status.

## Which release is it running

```bash
duckctl --name <robot-name> version
```

The API version, the release, and the git revision it was built from. A `revision` of `null` means
the release was built on somebody's laptop rather than by CI.

## Updates

Same words as `robotctl update`, so a command learned on the robot works here. Every one of them
takes `--component <name>` and defaults to `daemon`, which is the only component a robot has today.

```bash
duckctl --name <robot-name> update check
```

```bash
duckctl --name <robot-name> update status
```

```bash
duckctl --name <robot-name> update versions
```

```bash
duckctl --name <robot-name> update log --limit 20
```

Installing takes minutes and prints progress lines as it goes:

```bash
duckctl --name <robot-name> update apply
```

```
· daemon: preflight
· daemon: downloading 12%
· daemon: downloading 47%
· daemon: verifying
· daemon: swapping
· daemon: health_gate
{
  "outcome": "applied",
  "from": "0.5.1",
  "to": "0.6.0"
}
note: the robot restarts its daemons now, and `btd` about five seconds after this reply — so this
connection drops. That is the update working. Reconnect and run `duckctl update status`:
`last_attempt` carries the outcome of what just ran.
```

The connection dropping after an apply is the update working, not a failure. Reconnect and read
`update status`.

A branch build, an exact version, or the staging candidate:

```bash
duckctl --name <robot-name> update apply --ref my-branch
```

```bash
duckctl --name <robot-name> update apply --version 0.5.1
```

```bash
duckctl --name <robot-name> update apply --staging
```

`--dry-run` verifies everything and stops before the swap. `--ref` and `--version` are alternatives;
asking for both is refused.

Going back — the previous release, or one named from `update versions`:

```bash
duckctl --name <robot-name> update rollback
```

```bash
duckctl --name <robot-name> update select 0.5.1
```

Both are gated like an apply, so one that does not come up is reverted. Neither discards anything.

Progress for an update somebody else started, or one triggered by the robot itself:

```bash
duckctl --name <robot-name> update watch
```

It prints where the update in flight has got to and then everything that follows. It never receives
a reply, so it ends with Ctrl-C.

## Anything else — `call`

```bash
duckctl --name <robot-name> call <method> '<json-params>'
```

Params default to `{}`. These are reachable over Bluetooth but have no wrapper of their own, and are
written without the `duckctl --name <robot-name>` in front of them:

| | |
|---|---|
| `call system.services` | Which daemons are up, and the release each runs. |
| `call pad.status` | Is a gamepad bonded, and is it connected? |
| `call pad.pair '{"timeout_seconds":30}'` | Bond a pad held in pairing mode. |
| `call pad.forget '{"mac":"<address>"}'` | Drop a bond. |

`call` waits 60 seconds for an answer. The update commands above wait on *silence* instead — three
minutes with nothing arriving at all — which is why they are the way to run an update rather than
`call update.apply`.

## Global options

- `--name <robot-name>` — which robot. Without it, `DUCK_ROBOT`; without that, the first one found
  wins. Worth giving always: it skips a slow fallback tier that tries every already-connected
  peripheral on the Mac, earbuds included.
- `--pin <six-digits>` — defaults to `DUCK_PIN`, then to `000000`. `robotctl system pin` on the
  robot shows the real one.
- `--verbose` — print every line sent and received, and have `scan` list every device rather than
  only the robots. The first thing to add when something hangs.

## What it prints

Replies go to stdout as pretty JSON, and everything else — progress, diagnosis, what the radio
saw — to stderr. So `duckctl ... info > reply.json` keeps the two apart, and a JSON-RPC error
from the robot still exits non-zero. Progress lines start with `·` and are one line each, so
`update apply > outcome.json` leaves them on screen and keeps the outcome in the file.

One command is one connection: it finds the robot, pairs if it has to, proves the PIN, asks, and
disconnects.

Every command gives up after a period of silence rather than after a fixed total, so a slow update
is never cut off and a robot that stops answering is reported in seconds. A link that *drops* is
reported at once rather than waited out, and after an apply it says so: the restart is what took the
connection down. An update in flight survives either: the robot pulls, so it carries on with nobody
watching, and `update status` afterwards says how it went.

## What is refused

Motor control (`robot.move`, `robot.head`, `robot.enable`, `robot.stop`, `robot.init`,
`robot.relax`), high-rate telemetry (`robot.subscribe`), the two update commands a person has to
mean (`update.pin`, `update.resetToGolden`) and the pairing PIN (`system.pairingPin`,
`system.setPairingPin`) are refused by `btd` itself and never reach a daemon. They come back as
error code 14, "not available over Bluetooth".

That is a security boundary rather than a missing feature, and each refusal has its reason next
to it in `btd/src/route.rs` — [`app-path-design.md`](../design/app-path-design.md) §3.1 is the
design.
Those commands are `robotctl` on the robot.

## When it cannot find the robot

```bash
duckctl --verbose scan
```

An empty list — not one pair of earbuds — points at the Mac rather than the robot: Bluetooth off,
or the terminal never granted the Bluetooth permission.

A list the robot is missing from points at the robot. It advertises its name in a scan response that
can be missed on its own, so a device reported with no name and no services is a plausible robot;
`--name` connects to one anyway. If macOS shows the robot as paired but a connection or the first
read hangs, the bond is half-finished:

```bash
sudo pkill bluetoothd
```

Forgetting the robot in macOS Bluetooth settings does the same thing. On the robot itself,
`journalctl -u btd -b` says whether the GATT application registered at all.
