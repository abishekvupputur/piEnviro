# Running piEnviro (Pi Zero 2 W)

Everything that runs on piEnviro, apart from the sensor code itself. Log in with `ssh abi@piEnviro`;
the working copy is `~/piEnviro`.

```
enviroMultiClient.py (container "piEnviro", host network) → Mosquitto 127.0.0.1:1883 on the Pi
   ├─ bridge "homelab" → 192.168.8.111:1883 (WeatherApp), when that answers
   └─ mqtt2status ── tailnet ──> ts-relay:8430 → status.vupputur.cc "Indoor"
```

| File | Installed as | What |
|---|---|---|
| `../docker-compose.yml` | used in `~/piEnviro` | the sensor container: hardware devices, `network_mode: host`, `MQTT_BROKER=127.0.0.1`, script mounted |
| `pienviro.service` | `/etc/systemd/system/` | runs `docker compose up` in `~/piEnviro` at boot |
| `mosquitto/pienviro.conf` | `/etc/mosquitto/conf.d/` | local broker on 127.0.0.1 only, forwards `sensor/environment` to the homelab broker |
| `mqtt2status`, `mqtt2status.service` | `/usr/local/bin/`, `/etc/systemd/system/` | pushes the latest reading every 30 s to status.vupputur.cc |

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
`_cpu_temperature_celsius`, `_humidity_percent`, `_pressure_hpa`, `_pm_ug_m3{size}` and `_gas_kohm{gas}`.
Light and noise are deliberately not pushed: they reveal when someone is home, and ts-relay is
untrusted. They still go to the local broker and on to the homelab broker.
