# Meshtastic MQTT Bridge

Python service that bridges Home Assistant MQTT to the Meshtastic mesh network.

## Overview

```
Home Assistant ──MQTT──► meshtastic_bridge.py ──LoRa──► Tracker device
                  ◄──────────────────────────────────── (status updates)
```

The bridge:
- Subscribes to `meshtastic/admin/geofence_config/<tracker_id>` for config messages
- Forwards polygon geofence configs to tracker devices via the Meshtastic mesh
- Receives status messages from trackers and publishes them to `meshtastic/tracker/<id>/status`
- Handles offline trackers with automatic retry (up to 5 attempts, 30 s apart)
- Respects Meshtastic mesh rate limits (5 s between sends)

## Message Format

### Inbound (HA → Bridge → Tracker)

Published to `meshtastic/admin/geofence_config/<tracker_id>`:

```json
{
  "type": "geofence_config",
  "tracker_id": "1234",
  "person_name": "Alice",
  "geofence_name": "home_yard",
  "polygon": [
    [40.7128, -74.0060],
    [40.7135, -74.0060],
    [40.7135, -74.0055],
    [40.7128, -74.0055]
  ],
  "relay_gpio": 10,
  "enable_beeper": true,
  "version": 1
}
```

### Outbound (Tracker → Bridge → HA)

Published to `meshtastic/tracker/<tracker_id>/status`:

```json
{
  "type": "status",
  "tracker_id": "1234",
  "inside": true,
  "lat": 40.7130,
  "lon": -74.0058,
  "battery": 85
}
```

## Requirements

- Python 3.8+
- paho-mqtt
- meshtastic Python API
- pypubsub
- PyYAML

## Installation

```bash
# Clone the repository
git clone https://github.com/awayman/meshtastic-geofence-tracker.git
cd meshtastic-geofence-tracker/bridge

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Copy and edit configuration
cp config.example.yaml config.yaml
$EDITOR config.yaml
```

## Running

```bash
python meshtastic_bridge.py --config config.yaml
```

Optional environment variables:
- `BRIDGE_CONFIG` — path to config file (default: `config.yaml`)
- `LOG_LEVEL` — `DEBUG`, `INFO`, `WARNING`, or `ERROR` (default: `INFO`)

## Systemd Service (Linux)

```bash
# Create dedicated user
sudo useradd -r -s /bin/false meshtastic
sudo usermod -aG dialout meshtastic   # serial port access

# Install service files
sudo mkdir -p /opt/meshtastic-bridge
sudo cp -r . /opt/meshtastic-bridge/
sudo python3 -m venv /opt/meshtastic-bridge/venv
sudo /opt/meshtastic-bridge/venv/bin/pip install -r /opt/meshtastic-bridge/requirements.txt
sudo cp /opt/meshtastic-bridge/config.example.yaml /opt/meshtastic-bridge/config.yaml
# Edit /opt/meshtastic-bridge/config.yaml

sudo cp systemd/meshtastic-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable meshtastic-bridge
sudo systemctl start meshtastic-bridge

# View logs
journalctl -u meshtastic-bridge -f
```

## Docker

```bash
# Copy and edit config
cp config.example.yaml config.yaml
$EDITOR config.yaml

# Build and run
docker compose up -d

# View logs
docker compose logs -f
```

For a USB-connected Meshtastic device, uncomment the `devices` section in `docker-compose.yml`.

## Configuration Reference

| Key | Description | Default |
|-----|-------------|---------|
| `mqtt.host` | MQTT broker hostname | required |
| `mqtt.port` | MQTT broker port | `1883` |
| `mqtt.username` | MQTT username | `""` |
| `mqtt.password` | MQTT password | `""` |
| `mqtt.tls.enabled` | Enable TLS | `false` |
| `meshtastic.connection_type` | `serial` or `tcp` | required |
| `meshtastic.port` | Serial port path | required for serial |
| `meshtastic.host` | TCP hostname | required for tcp |
