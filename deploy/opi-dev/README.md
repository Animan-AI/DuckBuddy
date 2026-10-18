# rev-a dev robot (OPi Zero 3W): robotd + padd as user services

```bash
mkdir -p ~/.config/systemd/user && cp robotd-dev.service padd-dev.service ~/.config/systemd/user/
cd ~/robot && ln -sfn <config>.toml active.toml        # which config robotd runs
systemctl --user daemon-reload && systemctl --user enable --now robotd-dev.service
sudo loginctl enable-linger orangepi                    # once: start at boot with nobody logged in
```

- Change config: `ln -sfn other.toml ~/robot/active.toml && systemctl --user restart robotd-dev` (padd follows).
- Scripted trials (`trial.sh`) want robotd but **not** padd: padd sends a zero twist every tick with the
  sticks centred, which overrides the trial's `robot.move` — `systemctl --user stop padd-dev` first.
- `start.sh` refuses while the service's robotd runs: `systemctl --user stop robotd-dev` to go manual.
- Logs: `~/robot/recs/robotd-service.log`, `~/robot/recs/padd-service.log`.
- uhid (BLE gamepads) loads at boot from `/etc/modules-load.d/uhid.conf`; the pad reconnects by itself.
- **Start on the pad powers a limp robot up and stands it** — hold the robot first.
