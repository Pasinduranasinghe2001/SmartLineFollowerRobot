#!/usr/bin/env python3
"""
=============================================================================
 mqtt_tuner.py  -  Interactive MQTT parameter tuner for EC6090 Robot
=============================================================================
Requires:  pip install paho-mqtt

Usage:
    python tools/mqtt_tuner.py
    python tools/mqtt_tuner.py --broker 192.168.1.100
    python tools/mqtt_tuner.py --broker 192.168.1.100 --port 1883

Once connected:
    Type any SET command:   KP 18.5
    Save to EEPROM:         save
    Load from EEPROM:       load
    Request status dump:    status
    Watch ACK messages:     shown automatically
    Exit:                   quit  /  Ctrl-C

All parameter keys (case-insensitive):
    PID:         KP  KD  FILTER  SPEEDDROP  BASE  FAST  SLOW  TURN  SHARP
                 RECOVER  SEARCH  REVERSE  REVERSE_BIAS  FORWARD_REC
                 MINSPEED  TIMEOUT_L  TIMEOUT_R  FORWARD_TIME  LTRIM  RTRIM
    Obstacle:    APPSPD  AVDSPD  PCKSPD  REVTIME  FWDTIME  T90TIME
                 SLWDIST  COLDIST  PCKDIST  REDTHR  GRNTHR
    Servo:       SVHOME  SVPICK
    Physic 3:    CURVTHR  CURVSPD  CURVLOOPS
    Physic 4:    AVDSIDE
=============================================================================
"""

import argparse
import json
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] paho-mqtt not installed. Run:  pip install paho-mqtt")
    sys.exit(1)

# ── Topics (must match config.h) ─────────────────────────────────────────
TOPIC_SET    = "robot/params/set"
TOPIC_SAVE   = "robot/params/save"
TOPIC_LOAD   = "robot/params/load"
TOPIC_STATUS = "robot/params/status"
TOPIC_ACK    = "robot/params/ack"

# ── Colour helpers ────────────────────────────────────────────────────────
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
RED    = "\033[31m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

# ── Valid keys grouped for help display ──────────────────────────────────
KEY_GROUPS = {
    "PID / Speed": [
        ("KP",           "float", "PID proportional gain"),
        ("KD",           "float", "PID derivative gain"),
        ("FILTER",       "float", "Position low-pass alpha (0=raw)"),
        ("SPEEDDROP",    "float", "PWM drop per unit |error|"),
        ("BASE",         "int",   "Normal forward speed (PWM)"),
        ("FAST",         "int",   "Fast outer wheel speed"),
        ("SLOW",         "int",   "Slow inner wheel speed"),
        ("TURN",         "int",   "Recovery pivot speed"),
        ("SHARP",        "int",   "Sharp 90deg turn speed"),
        ("RECOVER",      "int",   "Post-obstacle recovery speed"),
        ("SEARCH",       "int",   "Lost-line search pivot speed"),
        ("REVERSE",      "int",   "Reverse speed"),
        ("REVERSE_BIAS", "int",   "Reverse asymmetry delta"),
        ("FORWARD_REC",  "int",   "Forward creep speed in recovery"),
        ("MINSPEED",     "int",   "Minimum PID dynBase"),
        ("LTRIM",        "int",   "Left motor trim (+PWM)"),
        ("RTRIM",        "int",   "Right motor trim (+PWM)"),
        ("TIMEOUT_L",    "ms",    "Max ms for left pivot recovery"),
        ("TIMEOUT_R",    "ms",    "Max ms for right pivot / reverse"),
        ("FORWARD_TIME", "ms",    "Forward creep time in recovery"),
    ],
    "Obstacle / Avoidance": [
        ("APPSPD",   "int",   "Approach speed to obstacle"),
        ("AVDSPD",   "int",   "Avoidance maneuver speed"),
        ("PCKSPD",   "int",   "Green pick approach speed"),
        ("REVTIME",  "ms",    "Reverse time before side-step"),
        ("FWDTIME",  "ms",    "Forward time past obstacle"),
        ("T90TIME",  "ms",    "Time for each 90deg pivot"),
        ("SLWDIST",  "cm",    "Start-slow distance"),
        ("COLDIST",  "cm",    "Colour check distance"),
        ("PCKDIST",  "cm",    "Green pick distance"),
        ("REDTHR",   "int",   "Red colour threshold"),
        ("GRNTHR",   "int",   "Green colour threshold"),
    ],
    "Servo": [
        ("SVHOME", "deg", "Servo closed angle (110 = grip)"),
        ("SVPICK", "deg", "Servo open angle  (  1 = release)"),
    ],
    "Physic 3 - Curvature-Adaptive Speed": [
        ("CURVTHR",   "float", "|filteredPos| threshold to enter curve mode"),
        ("CURVSPD",   "int",   "Speed cap while in curve mode"),
        ("CURVLOOPS", "int",   "Consecutive loops above thresh to activate"),
    ],
    "Physic 4 - Obstacle Side Memory": [
        ("AVDSIDE", "0/1", "0=always avoid left  1=auto-select from lastSeenSide"),
    ],
}

VALID_KEYS = {k for group in KEY_GROUPS.values() for k, _, _ in group}


def print_help():
    print()
    print(f"{BOLD}{'─'*65}{RESET}")
    print(f"{BOLD}  EC6090 Robot MQTT Tuner  -  Parameter Reference{RESET}")
    print(f"{BOLD}{'─'*65}{RESET}")
    for group, params in KEY_GROUPS.items():
        print(f"\n  {CYAN}{BOLD}{group}{RESET}")
        for key, unit, desc in params:
            print(f"    {YELLOW}{key:<14}{RESET} ({unit:<5})  {desc}")
    print()
    print(f"  {BOLD}Commands:{RESET}")
    print(f"    {GREEN}<KEY> <value>{RESET}   e.g.  KP 18.5   or   CURVTHR 0.55")
    print(f"    {GREEN}save{RESET}            Save current params to EEPROM")
    print(f"    {GREEN}load{RESET}            Load params from EEPROM")
    print(f"    {GREEN}status{RESET}          Dump current params (printed on robot serial)")
    print(f"    {GREEN}help{RESET}            Show this reference")
    print(f"    {GREEN}quit{RESET}            Exit tuner")
    print()


class RobotTuner:
    def __init__(self, broker: str, port: int):
        self.broker   = broker
        self.port     = port
        self.connected = False
        self._ack_lock = threading.Lock()

        self.client = mqtt.Client(client_id="mqtt_tuner_py")
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message    = self._on_message

    # ── MQTT callbacks ────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            self.client.subscribe(TOPIC_ACK)
            print(f"\n{GREEN}[MQTT] Connected to {self.broker}:{self.port}{RESET}")
            print(f"{CYAN}[MQTT] Subscribed to {TOPIC_ACK}{RESET}")
            print(f"Type {YELLOW}help{RESET} for parameter reference, "
                  f"{YELLOW}quit{RESET} to exit.\n")
        else:
            print(f"{RED}[MQTT] Connection failed (rc={rc}){RESET}")

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print(f"\n{RED}[MQTT] Disconnected (rc={rc}){RESET}")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            ok   = data.get("ok", None)
            if ok is True:
                key = data.get("key", data.get("action", "?"))
                val = data.get("value", "")
                val_str = f" = {val}" if val else ""
                print(f"  {GREEN}[ACK] {key}{val_str} -> OK{RESET}")
            elif ok is False:
                reason = data.get("reason", "unknown error")
                key    = data.get("key", "?")
                print(f"  {RED}[ACK] {key} -> FAILED: {reason}{RESET}")
            else:
                # Event messages (connected, etc.)
                print(f"  {CYAN}[MSG] {msg.payload.decode()}{RESET}")
        except Exception:
            print(f"  {CYAN}[MSG] {msg.payload.decode()}{RESET}")

    # ── Publish helpers ───────────────────────────────────────────────────
    def send_set(self, key: str, value: str):
        key = key.upper().strip()
        if key not in VALID_KEYS:
            # Warn but still send - robot will reply with unknown key
            print(f"  {YELLOW}[WARN] '{key}' not in known keys. Sending anyway...{RESET}")
        payload = json.dumps({"key": key, "value": value})
        self.client.publish(TOPIC_SET, payload)
        print(f"  {CYAN}[TX] SET {key} = {value}{RESET}")

    def send_save(self):
        self.client.publish(TOPIC_SAVE, "1")
        print(f"  {CYAN}[TX] SAVE{RESET}")

    def send_load(self):
        self.client.publish(TOPIC_LOAD, "1")
        print(f"  {CYAN}[TX] LOAD{RESET}")

    def send_status(self):
        self.client.publish(TOPIC_STATUS, "1")
        print(f"  {CYAN}[TX] STATUS (check robot serial monitor){RESET}")

    # ── Main loop ─────────────────────────────────────────────────────────
    def run(self):
        print(f"{BOLD}EC6090 Robot MQTT Tuner{RESET}")
        print(f"Connecting to broker {YELLOW}{self.broker}:{self.port}{RESET} ...")

        try:
            self.client.connect(self.broker, self.port, keepalive=60)
        except Exception as e:
            print(f"{RED}[ERROR] Cannot reach broker: {e}{RESET}")
            print("Check that your broker is running and MQTT_BROKER in config.h matches.")
            sys.exit(1)

        self.client.loop_start()   # background thread for MQTT receive

        # Wait for connection
        deadline = time.time() + 5.0
        while not self.connected and time.time() < deadline:
            time.sleep(0.1)
        if not self.connected:
            print(f"{RED}[ERROR] Timed out waiting for broker connection.{RESET}")
            self.client.loop_stop()
            sys.exit(1)

        # ── Interactive command loop ──────────────────────────────────────
        while True:
            try:
                raw = input(f"{BOLD}tuner>{RESET} ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                break

            if not raw:
                continue

            cmd = raw.lower()

            if cmd in ("quit", "exit", "q"):
                break
            elif cmd in ("help", "h", "?"):
                print_help()
            elif cmd == "save":
                self.send_save()
            elif cmd == "load":
                self.send_load()
            elif cmd == "status":
                self.send_status()
            else:
                parts = raw.split()
                if len(parts) == 2:
                    self.send_set(parts[0], parts[1])
                else:
                    print(f"  {RED}[ERROR] Format: <KEY> <value>  e.g.  KP 18.5{RESET}")
                    print(f"  Type {YELLOW}help{RESET} for full parameter list.")

        self.client.loop_stop()
        self.client.disconnect()
        print("Disconnected. Bye.")


# ── Entry point ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="EC6090 Robot MQTT parameter tuner"
    )
    parser.add_argument(
        "--broker", default="192.168.1.100",
        help="MQTT broker IP (default: 192.168.1.100)"
    )
    parser.add_argument(
        "--port", type=int, default=1883,
        help="MQTT broker port (default: 1883)"
    )
    args = parser.parse_args()

    tuner = RobotTuner(broker=args.broker, port=args.port)
    tuner.run()
