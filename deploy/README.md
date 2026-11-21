# Running piEnviro (Pi Zero 2 W)

Everything that runs on piEnviro, apart from the sensor code itself. Log in with `ssh abi@piEnviro`;
the working copy is `~/piEnviro`.

```
enviroMultiClient.py (container "piEnviro", host network) → Mosquitto 127.0.0.1:1883 on the Pi
   ├─ bridge "homelab" ⇄ 192.168.8.111:1883 (EMQX on bigfootpi): sensor/environment out (WeatherApp), sensor/rack in (PiAbi)
   └─ mqtt2status ── tailnet ──> ts-relay:8430 → status.vupputur.cc "Indoor"
```

| File | Installed as | What |
|---|---|---|
| `../docker-compose.yml` | used in `~/piEnviro` | the sensor container: hardware devices, `network_mode: host`, `MQTT_BROKER=127.0.0.1`, script mounted |
| `pienviro.service` | `/etc/systemd/system/` | runs `docker compose up` in `~/piEnviro` at boot |
| `mosquitto/pienviro.conf` | `/etc/mosquitto/conf.d/` | local broker on 127.0.0.1 only, forwards `sensor/environment` to the homelab broker |
| `mqtt2status`, `mqtt2status.service` | `/usr/local/bin/`, `/etc/systemd/system/` | pushes the latest reading every 30 s to status.vupputur.cc |
| `systemd/journald-persistent.conf` | `/etc/systemd/journald.conf.d/` | journal kept across reboots (50 MB cap), so a freeze leaves its last messages: `journalctl -b -1 -k` |
| `systemd/watchdog.conf` | `/etc/systemd/system.conf.d/` | systemd pets the hardware watchdog; a hang of 15 s resets the Pi |
| `networkmanager/wifi-powersave-off.conf` | `/etc/NetworkManager/conf.d/` | Wi-Fi power save off (check: `/sbin/iwconfig wlan0` shows `Power Management:off`) |

`/boot/firmware/config.txt` is not in the repo. Under `[all]` it keeps `arm_freq=600` (less heat and
power); `over_voltage=-6` and `gpu_freq=100` (copied from a Pi 4 guide) were removed on 2026-11-21
while chasing freezes, with a backup in `/boot/firmware/config.txt.bak-2026-11-21`. Don't add them back.

## How changes flow

Edit on the PC (`C:\Git\piEnviro`), commit and push. Then, on the Pi:

```bash
cd ~/piEnviro && git pull            # fetches over HTTPS (the repo is public); the Pi never pushes
sudo systemctl restart pienviro      # after changing docker-compose.yml or examples/enviroMultiClient.py
```

The script is mounted into the container, so a restart is enough. Only `Dockerfile` or library
changes need `docker compose build`, which is slow on a Zero 2 W.

After changing a file in `deploy/`, reinstall it on the Pi:

```bash
cd ~/piEnviro/deploy
sudo install -m 644 pienviro.service mqtt2status.service /etc/systemd/system/
sudo install -m 755 mqtt2status /usr/local/bin/
sudo install -m 644 mosquitto/pienviro.conf /etc/mosquitto/conf.d/
sudo install -D -m 644 systemd/journald-persistent.conf -t /etc/systemd/journald.conf.d/
sudo install -D -m 644 systemd/watchdog.conf -t /etc/systemd/system.conf.d/
sudo install -D -m 644 networkmanager/wifi-powersave-off.conf -t /etc/NetworkManager/conf.d/
sudo systemctl reload NetworkManager   # applies on the next (re)connect, e.g. after a reboot
sudo systemctl restart systemd-journald && sudo journalctl --flush
sudo systemctl daemon-reexec          # picks up the watchdog setting
sudo systemctl daemon-reload && sudo systemctl restart mosquitto mqtt2status pienviro
```

## First-time setup (new SD card)

```bash
sudo apt-get install -yq mosquitto python3-paho-mqtt
# Debian's mosquitto.conf has no listener; never add "listener 1883" / "allow_anonymous true" there
# (that opens the broker to the LAN). Our conf.d file adds the localhost-only listener.
# Then the reinstall block above, and: sudo systemctl enable mosquitto mqtt2status pienviro
```

`mqtt2status` also needs `/etc/mqtt2status.env` (root, 600) with `INGEST_TOKEN=`. The value is
`SENSOR_INGEST_TOKEN` from ts-relay's `secrets/edge.env`. Copy it from the PC without printing it:

```bash
ssh abishek@ts-relay 'sudo grep ^SENSOR_INGEST_TOKEN= /opt/ts-relay/secrets/edge.env' | sed 's/^SENSOR_INGEST_TOKEN=/INGEST_TOKEN=/' \
  | ssh abi@piEnviro 'sudo sh -c "umask 077; cat > /etc/mqtt2status.env"'
```

The ts-relay side (the tailnet-only, write-only ingest and the status page) lives in the ts-relay
repo: `stacks/routes/edge/Caddyfile` and the "Sensor ingest" notes in its AGENTS.md.

The pushed metrics are `pienviro_temperature_celsius`, `_raw_temperature_celsius`,
`_cpu_temperature_celsius`, `_humidity_percent`, `_pressure_hpa`, `_pm_ug_m3{size}` and `_gas_kohm{gas}`,
plus `pienviro_uptime_seconds` (the Pi's uptime, to spot reboots/crashes) and `rack_temperature_celsius` /
`rack_humidity_percent` from the rack sensor (sensor/rack).
Light and noise are deliberately not pushed: they reveal when someone is home, and ts-relay is
untrusted. They still go to the local broker and on to the homelab broker.
