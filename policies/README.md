# Policies

The ONNX policies `robotd` runs. All of them are `obs[1,61] -> actions[1,14]`; `robotd`
checks that at load rather than discovering it mid-stride.

## This is a temporary home

**These belong on the Hugging Face Hub**, delivered as a `model` updater component that
versions independently of the daemon — a gait retrain should not need a daemon release, and a
daemon fix should not re-download 6 MB of unchanged weights. `deploy/updater.toml` already
describes that component and deliberately leaves it unconfigured until the repos exist.

They are vendored here because they were not on the Hub yet and the daemon cannot move without
them. Committing them makes a release self-contained, which is the property that makes the
update path testable end to end: one `robotctl update apply` turns a standing robot into a
walking one.

Removing this directory later is the whole migration: point the `[policy]` paths in
`deploy/robotd.toml` at wherever the model component installs, and drop the `--include`
lines from `.github/workflows/dev.yml`, `.github/workflows/_build-release.yml` and
`scripts/dev-push.sh` — all three, and `xtask`'s `every_policy_in_the_repo_is_packaged`
test is what keeps the three lists honest until then.

## Provenance

Copied from `apirrone/microduck_runtime` at commit `5f3b314` (`roulade.onnx` at `7e4ab6d`,
where it first appeared), dereferencing the symlinks that repository uses to give stable
names to specific training runs:

| here | there | role |
| --- | --- | --- |
| `alpha_walking.onnx` | `BEST_alpha_walking_rough.onnx` | walking / velstand |
| `alpha_stand.onnx` | `BEST_alpha_stand_body_control.onnx` | standing + body-pose |
| `alpha_sitstand.onnx` | `BEST_alpha_sitstand.onnx` | sit ↔ stand (posture flag) |
| `alpha_ground_pick.onnx` | `alpha_ground_pick.onnx` | ground pick (phase command) |
| `ball_kick_left.onnx` | `ball_kick_left.onnx` | left-leg kick |
| `ball_kick_right.onnx` | `ball_kick_right.onnx` | right-leg kick |
| `roller.onnx` | `BEST_roller.onnx` | roller-mode locomotion |
| `roller_crouch.onnx` | `BEST_roller_crounch.onnx` | roller-mode crouch (ground-pick slot) |
| `roulade.onnx` | `roulade.onnx` | forward roll (Mjlab-Roulade-MicroDuck) |

(`roller_crouch` also fixes the upstream file name's typo.)

The names here are the *roles* — what `deploy/robotd.toml` asks for — not the training runs.
That indirection is deliberate and worth keeping: swapping which run is "the walking policy"
should not mean editing config on every robot.

## Why the 61-D family only

The prototype also ships a 51-D family (`3 gyro + 3 gravity + 42 joints + 3 command`, the
legacy `[vx, vy, vtheta]` command). 61 is the same sensors with the unified 13-value command
(`[vel(3), head(4), body(6)]`) this daemon builds — the only observation it builds. A 51-D
file fails at load with one precise line naming both widths:

```
policy unavailable: .../walking.onnx: observation width is 51, expected 61
```

That shape check earned its place: it turned a wrong-policy mistake into a diagnosis instead
of a robot moving in ways nobody could explain.

## Trying your own

No release needed — `deploy/robotd.toml` takes absolute paths:

```toml
[policy]
walk  = "/home/radxa/my_walk.onnx"
stand = "/home/radxa/my_stand.onnx"
```

Then `sudo systemctl restart robotd`. A policy that fails to load is reported through
`robot.health` as `policy unavailable: <reason>` while the loop keeps ticking and holding its
pose, so a bad file is visible without putting the robot on the floor.
