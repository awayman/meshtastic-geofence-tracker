#!/usr/bin/env python3
"""
Meshtastic MQTT Bridge
Bridges Home Assistant MQTT to the Meshtastic mesh network.

- Listens on MQTT for geofence configuration messages from Home Assistant
- Forwards polygon geofence configs to Meshtastic tracker devices via LoRa mesh
- Receives status messages from trackers
- Publishes tracker status back to MQTT for Home Assistant monitoring
- Handles offline trackers gracefully (queue/retry logic)
"""

import json
import logging
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import paho.mqtt.client as mqtt
import yaml

try:
    import meshtastic
    import meshtastic.serial_interface
    import meshtastic.tcp_interface
    from pubsub import pub
except ImportError as e:
    print(f"ERROR: Missing dependency: {e}")
    print("Install with: pip install -r requirements.txt")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger("meshtastic_bridge")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MQTT_CONFIG_TOPIC_PREFIX = "meshtastic/admin/geofence_config"
MQTT_STATUS_TOPIC_PREFIX = "meshtastic/tracker"

# Meshtastic message size limit in bytes
MESHTASTIC_MAX_MESSAGE_BYTES = 240

# How long (seconds) between delivery retries for queued messages
RETRY_INTERVAL_SECONDS = 30

# Maximum number of retry attempts before dropping a message
MAX_RETRY_ATTEMPTS = 5

# How long (seconds) to wait between sends to respect mesh rate limits
MESH_SEND_INTERVAL_SECONDS = 5


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class QueuedMessage:
    """A message waiting to be delivered to a tracker."""

    tracker_id: str
    payload: bytes
    attempts: int = 0
    enqueued_at: float = field(default_factory=time.time)


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_config(path: str) -> dict:
    """Load YAML configuration file."""
    if not os.path.exists(path):
        logger.error("Config file not found: %s", path)
        sys.exit(1)
    with open(path, "r") as fh:
        cfg = yaml.safe_load(fh)
    logger.info("Configuration loaded from %s", path)
    return cfg


def validate_config(cfg: dict) -> None:
    """Raise ValueError if required keys are missing."""
    required = {
        "mqtt": ["host", "port"],
        "meshtastic": ["connection_type"],
    }
    for section, keys in required.items():
        if section not in cfg:
            raise ValueError(f"Missing config section: {section}")
        for key in keys:
            if key not in cfg[section]:
                raise ValueError(f"Missing config key: {section}.{key}")

    conn_type = cfg["meshtastic"]["connection_type"]
    if conn_type == "serial" and "port" not in cfg["meshtastic"]:
        raise ValueError("meshtastic.port required when connection_type=serial")
    if conn_type == "tcp" and "host" not in cfg["meshtastic"]:
        raise ValueError("meshtastic.host required when connection_type=tcp")


# ---------------------------------------------------------------------------
# Geofence config validation
# ---------------------------------------------------------------------------

def validate_geofence_message(data: dict) -> Optional[str]:
    """Return an error string if the message is invalid, else None."""
    required_fields = ["type", "tracker_id", "polygon"]
    for f in required_fields:
        if f not in data:
            return f"Missing required field: {f}"
    if data.get("type") != "geofence_config":
        return f"Unexpected message type: {data.get('type')}"
    polygon = data.get("polygon")
    if not isinstance(polygon, list) or len(polygon) < 3:
        return "polygon must be a list of at least 3 coordinate pairs"
    for point in polygon:
        if not (isinstance(point, (list, tuple)) and len(point) == 2):
            return f"Invalid polygon point: {point}"
    return None


# ---------------------------------------------------------------------------
# Meshtastic interface wrapper
# ---------------------------------------------------------------------------

class MeshtasticInterface:
    """Wraps the Meshtastic Python API connection."""

    def __init__(self, cfg: dict):
        self._cfg = cfg
        self._iface = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        conn_type = self._cfg["connection_type"]
        logger.info("Connecting to Meshtastic device via %s…", conn_type)
        if conn_type == "serial":
            port = self._cfg.get("port")
            self._iface = meshtastic.serial_interface.SerialInterface(devPath=port)
        elif conn_type == "tcp":
            host = self._cfg["host"]
            port = self._cfg.get("port", 4403)
            self._iface = meshtastic.tcp_interface.TCPInterface(hostname=host, portNumber=port)
        else:
            raise ValueError(f"Unknown meshtastic connection_type: {conn_type}")
        logger.info("Connected to Meshtastic device")

    def disconnect(self) -> None:
        if self._iface:
            try:
                self._iface.close()
            except Exception:
                pass
            self._iface = None
            logger.info("Disconnected from Meshtastic device")

    @property
    def connected(self) -> bool:
        return self._iface is not None

    def send_text(self, text: str, destination_id: int) -> bool:
        """Send a text message to a destination node. Returns True on success."""
        with self._lock:
            if not self._iface:
                logger.warning("Cannot send: not connected to Meshtastic device")
                return False
            try:
                self._iface.sendText(
                    text=text,
                    destinationId=destination_id,
                    wantAck=True,
                )
                logger.debug(
                    "Sent %d bytes to node 0x%x", len(text.encode()), destination_id
                )
                return True
            except Exception as exc:
                logger.error("Failed to send to node 0x%x: %s", destination_id, exc)
                return False

    def subscribe_receive(self, callback) -> None:
        """Subscribe to incoming Meshtastic messages."""
        pub.subscribe(callback, "meshtastic.receive.text")
        logger.debug("Subscribed to meshtastic.receive.text")


# ---------------------------------------------------------------------------
# Bridge service
# ---------------------------------------------------------------------------

class MeshtasticMQTTBridge:
    """Main bridge service connecting MQTT and Meshtastic."""

    def __init__(self, cfg: dict):
        self._cfg = cfg
        self._mqtt_cfg = cfg["mqtt"]
        self._mesh_cfg = cfg["meshtastic"]

        self._mqtt_client = mqtt.Client(
            client_id=self._mqtt_cfg.get("client_id", "meshtastic_bridge"),
            protocol=mqtt.MQTTv311,
        )
        self._mesh = MeshtasticInterface(self._mesh_cfg)

        self._message_queue: queue.Queue = queue.Queue()
        self._pending: list[QueuedMessage] = []
        self._pending_lock = threading.Lock()

        self._running = False
        self._last_send_time = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._running = True

        # Connect MQTT
        self._setup_mqtt()
        self._connect_mqtt()

        # Connect Meshtastic
        self._mesh.connect()
        self._mesh.subscribe_receive(self._on_mesh_receive)

        # Start worker threads
        threading.Thread(target=self._queue_worker, daemon=True, name="queue-worker").start()
        threading.Thread(target=self._retry_worker, daemon=True, name="retry-worker").start()

        logger.info("Bridge service started")

    def stop(self) -> None:
        self._running = False
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()
        self._mesh.disconnect()
        logger.info("Bridge service stopped")

    # ------------------------------------------------------------------
    # MQTT setup
    # ------------------------------------------------------------------

    def _setup_mqtt(self) -> None:
        username = self._mqtt_cfg.get("username")
        password = self._mqtt_cfg.get("password")
        if username:
            self._mqtt_client.username_pw_set(username, password)

        tls = self._mqtt_cfg.get("tls", {})
        if tls.get("enabled"):
            self._mqtt_client.tls_set(
                ca_certs=tls.get("ca_certs"),
                certfile=tls.get("certfile"),
                keyfile=tls.get("keyfile"),
            )

        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message

    def _connect_mqtt(self) -> None:
        host = self._mqtt_cfg["host"]
        port = self._mqtt_cfg.get("port", 1883)
        keepalive = self._mqtt_cfg.get("keepalive", 60)
        logger.info("Connecting to MQTT broker %s:%d…", host, port)
        self._mqtt_client.connect(host, port, keepalive)
        self._mqtt_client.loop_start()

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            logger.info("Connected to MQTT broker")
            topic = f"{MQTT_CONFIG_TOPIC_PREFIX}/+"
            client.subscribe(topic, qos=1)
            logger.info("Subscribed to %s", topic)
        else:
            logger.error("MQTT connection failed with code %d", rc)

    def _on_mqtt_disconnect(self, client, userdata, rc) -> None:
        if rc != 0:
            logger.warning("Unexpected MQTT disconnect (rc=%d); will auto-reconnect", rc)
        else:
            logger.info("MQTT disconnected cleanly")

    def _on_mqtt_message(self, client, userdata, msg) -> None:
        topic = msg.topic
        logger.info("MQTT message received on %s (%d bytes)", topic, len(msg.payload))

        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            logger.error("Failed to parse MQTT payload on %s: %s", topic, exc)
            return

        error = validate_geofence_message(data)
        if error:
            logger.error("Invalid geofence message on %s: %s", topic, error)
            return

        tracker_id = data["tracker_id"]
        logger.info(
            "Received geofence config for tracker %s (geofence: %s, points: %d)",
            tracker_id,
            data.get("geofence_name", "unnamed"),
            len(data.get("polygon", [])),
        )

        # Encode and check size
        payload_str = json.dumps(data, separators=(",", ":"))
        payload_bytes = payload_str.encode("utf-8")

        if len(payload_bytes) > MESHTASTIC_MAX_MESSAGE_BYTES:
            logger.warning(
                "Payload for tracker %s is %d bytes (limit %d); message may be truncated",
                tracker_id,
                len(payload_bytes),
                MESHTASTIC_MAX_MESSAGE_BYTES,
            )

        self._enqueue_message(tracker_id, payload_bytes)

    # ------------------------------------------------------------------
    # Meshtastic receive callback
    # ------------------------------------------------------------------

    def _on_mesh_receive(self, packet, interface) -> None:
        try:
            decoded = packet.get("decoded", {})
            text = decoded.get("text", "")
            from_node = packet.get("from", 0)

            logger.info("Received mesh message from node 0x%x: %s", from_node, text[:80])

            try:
                data = json.loads(text)
            except json.JSONDecodeError:
                # Non-JSON mesh message; log and ignore
                logger.debug("Non-JSON mesh message from 0x%x: %s", from_node, text)
                return

            msg_type = data.get("type")
            tracker_id = data.get("tracker_id", hex(from_node))

            if msg_type == "status":
                self._publish_tracker_status(tracker_id, data)
            elif msg_type == "ack":
                logger.info("Delivery ACK from tracker %s", tracker_id)
                self._remove_pending(tracker_id)
            else:
                logger.debug("Unhandled mesh message type '%s' from 0x%x", msg_type, from_node)

        except Exception as exc:
            logger.error("Error processing mesh receive: %s", exc, exc_info=True)

    # ------------------------------------------------------------------
    # MQTT publish helpers
    # ------------------------------------------------------------------

    def _publish_tracker_status(self, tracker_id: str, data: dict) -> None:
        topic = f"{MQTT_STATUS_TOPIC_PREFIX}/{tracker_id}/status"
        payload = json.dumps(data)
        result = self._mqtt_client.publish(topic, payload, qos=1, retain=True)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            logger.info("Published status for tracker %s to %s", tracker_id, topic)
        else:
            logger.error(
                "Failed to publish status for tracker %s (rc=%d)", tracker_id, result.rc
            )

    # ------------------------------------------------------------------
    # Message queue / retry logic
    # ------------------------------------------------------------------

    def _enqueue_message(self, tracker_id: str, payload: bytes) -> None:
        msg = QueuedMessage(tracker_id=tracker_id, payload=payload)
        self._message_queue.put(msg)
        logger.debug("Enqueued message for tracker %s", tracker_id)

    def _queue_worker(self) -> None:
        """Drain the inbound queue and attempt delivery."""
        while self._running:
            try:
                msg = self._message_queue.get(timeout=1)
            except queue.Empty:
                continue

            self._attempt_delivery(msg)

    def _attempt_delivery(self, msg: QueuedMessage) -> None:
        """Try to deliver a message; on failure add to pending retry list."""
        # Enforce mesh rate limit
        elapsed = time.time() - self._last_send_time
        if elapsed < MESH_SEND_INTERVAL_SECONDS:
            time.sleep(MESH_SEND_INTERVAL_SECONDS - elapsed)

        if not self._mesh.connected:
            logger.warning(
                "Mesh not connected; queuing message for tracker %s for retry",
                msg.tracker_id,
            )
            self._add_to_pending(msg)
            return

        try:
            dest_id = int(msg.tracker_id, 16) if msg.tracker_id.startswith("0x") else int(msg.tracker_id)
        except (ValueError, TypeError):
            logger.error(
                "Cannot resolve tracker_id '%s' to a node ID; dropping message",
                msg.tracker_id,
            )
            return

        msg.attempts += 1
        success = self._mesh.send_text(msg.payload.decode("utf-8"), dest_id)
        self._last_send_time = time.time()

        if success:
            logger.info(
                "Delivered message to tracker %s (attempt %d)",
                msg.tracker_id,
                msg.attempts,
            )
        else:
            logger.warning(
                "Delivery failed for tracker %s (attempt %d/%d)",
                msg.tracker_id,
                msg.attempts,
                MAX_RETRY_ATTEMPTS,
            )
            if msg.attempts < MAX_RETRY_ATTEMPTS:
                self._add_to_pending(msg)
            else:
                logger.error(
                    "Dropping message for tracker %s after %d failed attempts",
                    msg.tracker_id,
                    msg.attempts,
                )

    def _add_to_pending(self, msg: QueuedMessage) -> None:
        with self._pending_lock:
            self._pending.append(msg)
        logger.debug("Added message for tracker %s to retry list", msg.tracker_id)

    def _remove_pending(self, tracker_id: str) -> None:
        with self._pending_lock:
            before = len(self._pending)
            self._pending = [m for m in self._pending if m.tracker_id != tracker_id]
            removed = before - len(self._pending)
        if removed:
            logger.info("Removed %d pending message(s) for tracker %s", removed, tracker_id)

    def _retry_worker(self) -> None:
        """Periodically retry pending messages."""
        while self._running:
            time.sleep(RETRY_INTERVAL_SECONDS)
            with self._pending_lock:
                due = list(self._pending)
                self._pending = []

            if due:
                logger.info("Retrying %d pending message(s)…", len(due))
            for msg in due:
                self._attempt_delivery(msg)

    # ------------------------------------------------------------------
    # Run loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        self.start()
        try:
            while self._running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Meshtastic MQTT Bridge")
    parser.add_argument(
        "--config",
        default=os.environ.get("BRIDGE_CONFIG", "config.yaml"),
        help="Path to YAML configuration file (default: config.yaml or $BRIDGE_CONFIG)",
    )
    parser.add_argument(
        "--log-level",
        default=os.environ.get("LOG_LEVEL", "INFO"),
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    args = parser.parse_args()

    logging.getLogger().setLevel(args.log_level)
    logger.setLevel(args.log_level)

    cfg = load_config(args.config)
    try:
        validate_config(cfg)
    except ValueError as exc:
        logger.error("Configuration error: %s", exc)
        sys.exit(1)

    bridge = MeshtasticMQTTBridge(cfg)

    def _handle_signal(signum, frame):
        logger.info("Received signal %d; shutting down…", signum)
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    bridge.run()


if __name__ == "__main__":
    main()
