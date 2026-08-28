FROM python:3.11-slim

LABEL org.opencontainers.image.title="Meshtastic MQTT Bridge"
LABEL org.opencontainers.image.description="Bridges Home Assistant MQTT to the Meshtastic mesh network"
LABEL org.opencontainers.image.source="https://github.com/awayman/meshtastic-geofence-tracker"

WORKDIR /app

# Install system dependencies for serial port access
RUN apt-get update && apt-get install -y --no-install-recommends \
    udev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY meshtastic_bridge.py .

# Config is mounted at runtime; see docker-compose.yml
ENV BRIDGE_CONFIG=/config/config.yaml
ENV LOG_LEVEL=INFO

# Serial device access requires --device /dev/ttyUSB0 at runtime
# or privileged mode in docker-compose.yml

ENTRYPOINT ["python", "meshtastic_bridge.py"]
