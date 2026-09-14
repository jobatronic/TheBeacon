#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
 ARGUS BRIDGE — Orange Pi Zero (Armbian) <-> Arduino Nano <-> MQTT
═══════════════════════════════════════════════════════════════

Role:
    Keep a persistent serial link to the Argus Arduino controller,
    send it a heartbeat so it never falls into local failsafe,
    publish everything it reports to MQTT (with Home Assistant
    auto-discovery), and let a few Home Assistant buttons trigger
    RESET commands on the critical relays.

Design philosophy (matches the Arduino firmware):
    - Boring and predictable over clever.
    - Never silently die — reconnect and keep logging instead.
    - Make failure visible (log it) rather than swallowing it.
    - The serial link to the Arduino is the important thing. MQTT
      is decoration on top of it — a broker outage or a bad
      publish must NEVER be allowed to interrupt the heartbeat
      loop. Every MQTT call in this file is wrapped so it can
      fail without taking anything else down with it.

Requires:
    python3-serial       sudo apt install -y python3-serial
    python3-paho-mqtt     sudo apt install -y python3-paho-mqtt

Before running, fill in the MQTT_* values below to match your
Mosquitto broker (the add-on running on your Home Assistant Pi 4)
and create a dedicated login for this script there rather than
using anonymous access.

Run manually first to test:

        python3 argus_bridge.py

Then install as a systemd service (see the accompanying
argus-bridge.service file) so it survives reboots, SSH logouts,
and crashes.
"""

import sys
import json
import time
import logging
import threading
import queue

import serial

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print(
        "Missing dependency: paho-mqtt. "
        "Install with: sudo apt install python3-paho-mqtt",
        file=sys.stderr,
    )
    raise


# ═══════════════════════════════════════════════════════════════
#  CONFIGURATION — SERIAL
# ═══════════════════════════════════════════════════════════════

SERIAL_PORT = "/dev/ttyS1"
BAUD_RATE = 115200

# Comfortably faster than the Arduino's 5000ms HEARTBEAT_TIMEOUT.
HEARTBEAT_INTERVAL_SECONDS = 1.0

# How long to wait between attempts to (re)open the serial port.
RECONNECT_DELAY_SECONDS = 2.0

# Serial read timeout. Short enough that the heartbeat and any
# queued outgoing commands still get serviced promptly even if
# the Arduino isn't sending anything.
READ_TIMEOUT_SECONDS = 0.5


# ═══════════════════════════════════════════════════════════════
#  CONFIGURATION — MQTT / HOME ASSISTANT
# ═══════════════════════════════════════════════════════════════
#
#  Fill these in for your setup. MQTT_BROKER_HOST is your Home
#  Assistant Pi 4's LAN IP (the Mosquitto add-on listens there on
#  port 1883 by default). Create a dedicated username/password for
#  this script under the Mosquitto add-on's configuration rather
#  than relying on anonymous access.
#

MQTT_BROKER_HOST = "CHANGE_ME"        # e.g. "192.168.1.50"
MQTT_BROKER_PORT = 1883
MQTT_USERNAME = "CHANGE_ME"
MQTT_PASSWORD = "CHANGE_ME"

MQTT_CLIENT_ID = "argus_bridge"

# How long to wait between attempts to reach the broker at startup.
MQTT_RECONNECT_DELAY_SECONDS = 5.0

# Topic layout:
#   argus/state/<name>   — values this script publishes
#   argus/cmd/<name>      — commands Home Assistant can send us
#   argus/status           — "online" / "offline" (availability)
TOPIC_PREFIX = "argus"
AVAILABILITY_TOPIC = f"{TOPIC_PREFIX}/status"

# Home Assistant's MQTT discovery prefix. "homeassistant" is the
# default HA listens on — no reason to change this unless you've
# changed it on the HA side too.
DISCOVERY_PREFIX = "homeassistant"

# Groups all these entities into one "device" card in Home
# Assistant instead of a flat list of unrelated-looking sensors.
DEVICE_ID = "argus_rack"

HA_DEVICE_INFO = {
    "identifiers": [DEVICE_ID],
    "name": "Argus Rack Controller",
    "manufacturer": "DIY",
    "model": "Orange Pi Zero + Arduino Nano",
}

# How long a RESET button press holds the relay off for, in
# seconds. Matches the firmware's RESET <n> <secs> command.
RESET_DURATION_SECONDS = 30

# Relay numbers for the critical (NC) devices, matching
# argus_controller.ino's relayNC[] table.
ROUTER_RELAY_NUMBER = 3            # OpenWRT Pi
DVR_RELAY_NUMBER = 4               # Camera DVR
HOME_AUTOMATION_RELAY_NUMBER = 5   # ESP8266/32 hub Pi


# ═══════════════════════════════════════════════════════════════
#  LOGGING
# ═══════════════════════════════════════════════════════════════
#
#  Plain stdout logging. Under systemd this lands in the journal
#  automatically (`journalctl -u argus-bridge -f`) with no extra
#  configuration needed.
#

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
)

log = logging.getLogger("argus")


# ═══════════════════════════════════════════════════════════════
#  SHARED STATE
# ═══════════════════════════════════════════════════════════════
#
#  Only the main loop ever writes to the serial port. Anything
#  that wants to send a command (stdin, or an MQTT button press)
#  puts it on this queue instead of touching the serial object
#  directly.
#

send_queue: "queue.Queue[str]" = queue.Queue()


# ═══════════════════════════════════════════════════════════════
#  MQTT TOPIC HELPERS
# ═══════════════════════════════════════════════════════════════

def state_topic(suffix: str) -> str:
    return f"{TOPIC_PREFIX}/state/{suffix}"


def command_topic(suffix: str) -> str:
    return f"{TOPIC_PREFIX}/cmd/{suffix}"


# ═══════════════════════════════════════════════════════════════
#  MQTT CLIENT SETUP
# ═══════════════════════════════════════════════════════════════
#
#  paho-mqtt 2.0 changed its callback signatures and requires
#  opting in to the old ("VERSION1") style explicitly. This works
#  whether apt gave you paho-mqtt 1.x or 2.x, without needing two
#  copies of every callback.
#

try:
    mqtt_client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION1,
        client_id=MQTT_CLIENT_ID,
    )
except AttributeError:
    # paho-mqtt < 2.0 has no CallbackAPIVersion at all — the plain
    # constructor already gives us this same callback style.
    mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)


def on_mqtt_connect(client, userdata, flags, rc):

    if rc != 0:
        log.error("MQTT connect failed (rc=%s)", rc)
        return

    log.info("MQTT connected to %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT)

    # Tell Home Assistant we're alive, then (re)subscribe to our
    # command topics — subscriptions don't survive a reconnect on
    # their own.
    client.publish(AVAILABILITY_TOPIC, "online", retain=True)

    client.subscribe(command_topic("reset_r3"))
    client.subscribe(command_topic("reset_r4"))
    client.subscribe(command_topic("reset_r5"))

    publish_all_discovery()


def on_mqtt_message(client, userdata, msg):

    topic = msg.topic

    log.info("MQTT command received on %s", topic)

    if topic == command_topic("reset_r3"):
        send_queue.put(f"RESET {ROUTER_RELAY_NUMBER} {RESET_DURATION_SECONDS}")

    elif topic == command_topic("reset_r4"):
        send_queue.put(f"RESET {DVR_RELAY_NUMBER} {RESET_DURATION_SECONDS}")

    elif topic == command_topic("reset_r5"):
        send_queue.put(
            f"RESET {HOME_AUTOMATION_RELAY_NUMBER} {RESET_DURATION_SECONDS}"
        )


mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# Last Will and Testament: if this script dies without a clean
# exit, the broker tells Home Assistant we've gone offline instead
# of every entity silently freezing at its last value forever.
mqtt_client.will_set(AVAILABILITY_TOPIC, "offline", retain=True)

mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message


# ═══════════════════════════════════════════════════════════════
#  MQTT PUBLISHING — never allowed to raise
# ═══════════════════════════════════════════════════════════════
#
#  Anything that touches the MQTT client goes through here. If the
#  broker is unreachable, this logs and moves on — it must never
#  interrupt the serial loop above it.
#

def publish_state(suffix: str, value) -> None:

    try:
        mqtt_client.publish(state_topic(suffix), str(value), retain=True)
    except Exception:
        log.warning("MQTT publish failed for %s (continuing)", suffix)


def publish_discovery(component: str, object_id: str, config: dict) -> None:

    topic = f"{DISCOVERY_PREFIX}/{component}/{DEVICE_ID}/{object_id}/config"

    try:
        mqtt_client.publish(topic, json.dumps(config), retain=True)
    except Exception:
        log.warning("MQTT discovery publish failed for %s", object_id)


def discovery_base(object_id: str, name: str) -> dict:

    return {
        "name": name,
        "unique_id": f"{DEVICE_ID}_{object_id}",
        "device": HA_DEVICE_INFO,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "online",
        "payload_not_available": "offline",
    }


def publish_all_discovery() -> None:
    """
    Tells Home Assistant what entities exist. Published retained,
    so HA rebuilds the whole device even if it was restarted after
    this script last ran. Safe to call every time we (re)connect.
    """

    log.info("Publishing Home Assistant discovery config")

    publish_discovery("sensor", "exhaust_temp", {
        **discovery_base("exhaust_temp", "Exhaust Temperature"),
        "state_topic": state_topic("temp_c"),
        "unit_of_measurement": "°C",
        "device_class": "temperature",
        "state_class": "measurement",
    })

    publish_discovery("sensor", "fan_speed", {
        **discovery_base("fan_speed", "Fan Speed"),
        "state_topic": state_topic("fan_pct"),
        "unit_of_measurement": "%",
        "state_class": "measurement",
        "icon": "mdi:fan",
    })

    publish_discovery("sensor", "last_event", {
        **discovery_base("last_event", "Last Event"),
        "state_topic": state_topic("last_event"),
        "icon": "mdi:message-text-outline",
    })

    publish_discovery("binary_sensor", "door", {
        **discovery_base("door", "Cabinet Door"),
        "state_topic": state_topic("door"),
        "device_class": "door",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("binary_sensor", "fire", {
        **discovery_base("fire", "Fire Alarm"),
        "state_topic": state_topic("fire"),
        "device_class": "heat",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("binary_sensor", "filter_clogged", {
        **discovery_base("filter_clogged", "Filter Clogged"),
        "state_topic": state_topic("filter_clogged"),
        "device_class": "problem",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("binary_sensor", "temp_sensor_fault", {
        **discovery_base("temp_sensor_fault", "Temp Sensor Fault"),
        "state_topic": state_topic("temp_fault"),
        "device_class": "problem",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("binary_sensor", "watchdog_fired", {
        **discovery_base("watchdog_fired", "Heartbeat Failsafe Active"),
        "state_topic": state_topic("watchdog_fired"),
        "device_class": "problem",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("binary_sensor", "water", {
        **discovery_base("water", "Water Detected"),
        "state_topic": state_topic("water"),
        "device_class": "moisture",
        "payload_on": "ON",
        "payload_off": "OFF",
    })

    publish_discovery("button", "reset_router", {
        **discovery_base("reset_router", "Reset Router (R3)"),
        "command_topic": command_topic("reset_r3"),
        "payload_press": "PRESS",
    })

    publish_discovery("button", "reset_dvr", {
        **discovery_base("reset_dvr", "Reset DVR (R4)"),
        "command_topic": command_topic("reset_r4"),
        "payload_press": "PRESS",
    })

    publish_discovery("button", "reset_home_automation", {
        **discovery_base(
            "reset_home_automation", "Reset Home Automation Pi (R5)"
        ),
        "command_topic": command_topic("reset_r5"),
        "payload_press": "PRESS",
    })


# ═══════════════════════════════════════════════════════════════
#  EVENT HANDLING
# ═══════════════════════════════════════════════════════════════
#
#  Every line the Arduino sends comes through here exactly once:
#  logged, mirrored to MQTT as "last event", and — for anything
#  that represents an ongoing state — published to its own topic
#  so Home Assistant has a live tile for it.
#

def handle_line(line: str) -> None:

    log.info("ARDUINO: %s", line)

    publish_state("last_event", line)

    if line == "READY":
        log.warning(
            "Arduino reported READY "
            "(fresh boot, power cycle, or watchdog reset)"
        )
        return

    if not line.startswith("EVT "):
        # Unrecognized / non-event line. Nothing further to do.
        return

    parts = line.split()
    rest = parts[1:]  # everything after "EVT"

    if not rest:
        return


    # ─── Door ──────────────────────────────────────────────────
    if rest[0] == "DOOR" and len(rest) >= 2:

        log.info("Door is now %s", rest[1])

        publish_state("door", "ON" if rest[1] == "OPEN" else "OFF")


    # ─── Fire ──────────────────────────────────────────────────
    elif rest[0] == "FIRE" and len(rest) >= 2 and rest[1] == "CLEARED":

        log.info("Fire alarm cleared")

        publish_state("fire", "OFF")

    elif rest[0] == "FIRE":

        temp = rest[1] if len(rest) >= 2 else "?"

        log.critical("FIRE THRESHOLD EXCEEDED: %s C", temp)

        publish_state("fire", "ON")

        # TODO: this is the one alarm you actually want wired to
        # something real (push notification, siren relay, etc.)
        # before relying on this system for anything.


    # ─── Temperature rising too fast ──────────────────────────
    elif rest[0] == "TEMP" and len(rest) >= 2 and rest[1] == "RISING":

        rate = rest[2] if len(rest) >= 3 else "?"

        log.warning("Temperature rising quickly: %s C/min", rate)


    # ─── Exhaust sensor fault/recovery ────────────────────────
    elif rest[0] == "TEMP" and len(rest) >= 2 and rest[1] == "SENSOR":

        status = rest[2] if len(rest) >= 3 else "?"

        if status == "FAULT":
            log.error(
                "Exhaust temperature sensor fault "
                "(disconnected, shorted, or unplugged)"
            )
            publish_state("temp_fault", "ON")
        else:
            log.info("Exhaust temperature sensor OK")
            publish_state("temp_fault", "OFF")


    # ─── Periodic report: EVT TEMP <val|FAULT> FAN <n> ... ────
    # Only sent if ENABLE_PERIODIC_REPORT is true in the firmware.
    # This is the main feed for the temperature/fan graphs.
    elif rest[0] == "TEMP":

        if len(rest) >= 2 and rest[1] != "FAULT":
            try:
                publish_state("temp_c", float(rest[1]))
            except ValueError:
                pass

        if len(rest) >= 4 and rest[2] == "FAN":
            try:
                fan_raw = int(rest[3])
                publish_state("fan_pct", round(fan_raw / 255 * 100))
            except ValueError:
                pass

        log.info("Periodic report: %s", " ".join(rest))


    # ─── Filter clogged ────────────────────────────────────────
    elif rest[0] == "FILTER" and len(rest) >= 2 and rest[1] == "CLOGGED":

        log.warning("Fan running hard without cooling — check the filter")

        publish_state("filter_clogged", "ON")

    elif rest[0] == "FILTER" and len(rest) >= 2 and rest[1] == "CLEARED":

        log.info("Filter clog condition cleared")

        publish_state("filter_clogged", "OFF")


    # ─── Water ─────────────────────────────────────────────────
    elif rest[0] == "WATER" and len(rest) >= 2 and rest[1] == "DETECTED":

        log.critical("Water detected in the cabinet")

        publish_state("water", "ON")

    elif rest[0] == "WATER" and len(rest) >= 2 and rest[1] == "CLEARED":

        log.info("Water condition cleared")

        publish_state("water", "OFF")


    # ─── Relay reset progress ──────────────────────────────────
    elif rest[0] == "RESET":

        log.info("Relay reset event: %s", " ".join(rest))


    # ─── Pi heartbeat failsafe ─────────────────────────────────
    elif rest[0] == "WATCHDOG" and len(rest) >= 2:

        if rest[1] == "FIRED":
            log.error(
                "Arduino lost our heartbeat and entered local "
                "failsafe — check why this script stopped, or "
                "was too slow to, send HB"
            )
            publish_state("watchdog_fired", "ON")
        else:
            log.info("Arduino heartbeat failsafe cleared")
            publish_state("watchdog_fired", "OFF")


    # ─── Arduino's own hardware watchdog fired ────────────────
    elif rest[0] == "MCU" and len(rest) >= 2 and rest[1] == "WATCHDOG":

        log.error(
            "Arduino's own hardware watchdog reset it — "
            "firmware got stuck and recovered on its own"
        )


    else:

        log.info("Unhandled event: %s", " ".join(rest))


# ═══════════════════════════════════════════════════════════════
#  SENDING COMMANDS
# ═══════════════════════════════════════════════════════════════

def send_command(ser: "serial.Serial", command: str) -> None:

    line = command.strip()

    if not line:
        return

    log.info("SENDING: %s", line)

    ser.write((line + "\n").encode("ascii", errors="replace"))
    ser.flush()


# ═══════════════════════════════════════════════════════════════
#  STDIN COMMAND READER
# ═══════════════════════════════════════════════════════════════
#
#  Lets you type commands directly when running this in a
#  foreground SSH session for testing:
#
#      R1 ON
#      RESET 5 30
#      FAN AUTO
#
#  When run as a systemd service with no attached terminal, stdin
#  simply never produces anything — harmless.
#

def stdin_reader_thread() -> None:

    try:
        for raw_line in sys.stdin:

            line = raw_line.strip()

            if line:
                send_queue.put(line)

    except Exception:
        # stdin going away (e.g. no terminal attached) is not an
        # error condition for this program.
        pass


# ═══════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════
#
#  Reconnects on any serial error instead of exiting, so a
#  power-cycled Arduino or a bumped cable doesn't require someone
#  to notice and manually restart this script. MQTT runs in its
#  own background thread (loop_start) and is completely decoupled
#  from this — a broker outage never blocks the Arduino heartbeat.
#

def connect_mqtt_with_retry() -> None:

    while True:
        try:
            mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)
            mqtt_client.loop_start()
            return
        except Exception as exc:
            log.error(
                "MQTT broker unreachable (%s) — retrying in %.0fs",
                exc,
                MQTT_RECONNECT_DELAY_SECONDS,
            )
            time.sleep(MQTT_RECONNECT_DELAY_SECONDS)


def run() -> None:

    connect_mqtt_with_retry()

    threading.Thread(target=stdin_reader_thread, daemon=True).start()

    while True:

        try:
            with serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=READ_TIMEOUT_SECONDS,
            ) as ser:

                log.info(
                    "Connected to %s @ %d baud",
                    SERIAL_PORT,
                    BAUD_RATE,
                )

                last_heartbeat = 0.0

                while True:

                    now = time.monotonic()

                    if now - last_heartbeat >= HEARTBEAT_INTERVAL_SECONDS:
                        send_command(ser, "HB")
                        last_heartbeat = now

                    # Drain any queued outgoing commands (stdin or
                    # an MQTT button press).
                    while not send_queue.empty():
                        send_command(ser, send_queue.get_nowait())

                    raw = ser.readline()

                    if not raw:
                        continue

                    try:
                        line = raw.decode("ascii", errors="replace").strip()
                    except Exception:
                        continue

                    if line:
                        handle_line(line)

        except serial.SerialException as exc:

            log.error(
                "Serial error (%s) — retrying in %.0fs",
                exc,
                RECONNECT_DELAY_SECONDS,
            )
            time.sleep(RECONNECT_DELAY_SECONDS)

        except Exception:

            log.exception(
                "Unexpected error — retrying in %.0fs",
                RECONNECT_DELAY_SECONDS,
            )
            time.sleep(RECONNECT_DELAY_SECONDS)


if __name__ == "__main__":

    try:
        run()

    except KeyboardInterrupt:
        log.info("Shutting down (Ctrl+C)")

        try:
            mqtt_client.publish(AVAILABILITY_TOPIC, "offline", retain=True)
            mqtt_client.loop_stop()
        except Exception:
            pass
