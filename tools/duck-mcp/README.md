# duck-mcp

The duck's body as an MCP server (stdio), for any mind: a voice agent, anima, Claude Code.
Runs on the robot next to robotd and only *asks* robotd (its JSON-RPC socket); robotd keeps every
safety rule it has, and this layer adds the ones a language model needs: no motor power, policy or
walking tools (a person does those), head gestures only while the stand policy holds the robot up,
refusals that say why (too hot, battery low, not standing).

Tools (v0, 2026-10-12): `status`, `look(yaw_deg, pitch_deg)`, `nod`, `shake_head`, `tilt_head`,
`play_sound(tag)`.

## Install on the robot

The OPi reaches the PyPI index but downloads from files.pythonhosted.org crawl (~10 B/s), so fetch
the aarch64 wheels on a PC and install offline. mcp 1.10.1 needs pydantic < 2.12 (it imports a
private helper newer pydantic dropped).

```bash
pip download --dest wheels --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
    --platform manylinux_2_28_aarch64 --platform any --python-version 3.13 --implementation cp \
    --only-binary=:all: "mcp==1.10.1" "pydantic>=2.11,<2.12"
scp -r wheels server.py orangepi@<robot-ip>:duck-mcp/
ssh orangepi@<robot-ip> 'cd ~/duck-mcp && python3 -m venv .venv && \
    .venv/bin/pip install --no-index --find-links wheels "mcp==1.10.1" "pydantic>=2.11,<2.12"'
```

## Use

A client starts it over ssh (`.mcp.json` at the repo root does this for Claude Code):

```bash
ssh orangepi@<robot-ip> "~/duck-mcp/.venv/bin/python -W ignore ~/duck-mcp/server.py"
```

## Next

- walking tools (short, slow, ToF-gated), `say(text)` once TTS plays on the robot, `snapshot()` once
  the camera driver (vin) loads at boot;
- `robot.look` runs the alpha robot's head IK (`HeadFk::alpha`), not rev-a's.
