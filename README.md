# SmartLineFollowerRobot

> **EC6090 Robotics & Automation | ESP32-based autonomous robot**  
> 5-sensor PID line following + ultrasonic obstacle detection + TCS3200 colour-based pick-and-place

[![PlatformIO](https://img.shields.io/badge/built%20with-PlatformIO-orange)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/board-ESP32%20DevKit-blue)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
[![Framework](https://img.shields.io/badge/framework-Arduino-teal)](https://docs.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

## Overview

SmartLineFollowerRobot is a modular, fully-tunable autonomous robot firmware written in C++ for the **ESP32 DevKit (38-pin)** using the **PlatformIO + Arduino** framework.

The robot follows a yellow/white line on a dark floor, detects obstacles ahead with an HC-SR04 ultrasonic sensor, reads the colour of a cube placed on the track with a TCS3200 sensor, and then either **avoids a red cube** (U-shape bypass manoeuvre) or **picks up and carries a green cube** to the end zone before releasing it with a single-servo drop-gate gripper.

---

## Features

- **Analog PID line following** using a 5-channel IR sensor array with weighted position averaging
- **Low-pass filter** on PID position to suppress ADC noise
- **Line-width adaptive gain** — reduces PID correction at intersections to prevent oscillation
- **4-stage lost-line recovery** — reverse with bias, creep forward, pivot left/right, repeat
- **Ultrasonic obstacle detection** with cached readings (50 ms interval) and distance-based slowing
- **TCS3200 colour detection** — 3-sample average with configurable red/green thresholds
- **Red cube avoidance** — timed U-shape bypass (reverse → pivot left → forward → pivot right → forward → find line)
- **Green cube pick-and-place** — close gate, scoop cube, carry to end zone, open gate and drop
- **EEPROM persistence** — all calibration data and tuning parameters survive power cycles
- **Live Serial tuning** — change any parameter at runtime with `SET KEY VALUE` without re-uploading
- **Auto-calibration** — interactive 2-phase floor/line calibration adapts to any surface

---

## Hardware

### Bill of Materials

| Component | Model | Qty |
|---|---|---|
| Microcontroller | ESP32 DevKit (38-pin) | 1 |
| Motor driver | L298N dual H-bridge | 1 |
| Drive motors | DC geared motor (TT or N20) | 2 |
| Front wheel | Castor / omni ball | 1 |
| IR sensor array | BFD-1000 / TCRT5000 5-ch analog | 1 |
| Ultrasonic sensor | HC-SR04 | 1 |
| Colour sensor | TCS3200 | 1 |
| Servo (gripper gate) | SG90 / MG90S | 1 |
| Voltage divider | 1 kOhm + 2 kOhm resistors | 1 set |
| Power | 7.4 V LiPo or 6× AA | 1 |

### Pin Mapping

```
ESP32 GPIO   Signal          Notes
-----------  --------------  -------------------------------------------
GPIO 34      IR S1 (left)    Input-only GPIO -- no OUTPUT allowed
GPIO 35      IR S2
GPIO 32      IR S3
GPIO 33      IR S4
GPIO 25      IR S5 (right)
GPIO 14      ENA (PWM)       LEDC channel 0, right motor
GPIO 27      IN1             L298N channel A direction
GPIO 26      IN2
GPIO 12      ENB (PWM)       LEDC channel 1, left motor
GPIO 13      IN3             L298N channel B direction
GPIO 23      IN4
GPIO 22      Servo signal    ESP32Servo, LEDC timers 2 & 3
GPIO  5      TRIG            HC-SR04 trigger
GPIO 18      ECHO            HC-SR04 echo -- USE VOLTAGE DIVIDER (5V->3.3V)
GPIO  4      TCS S0          Frequency scale
GPIO  2      TCS S1          Strapping pin -- set LOW after boot
GPIO 15      TCS S2          Filter select
GPIO 21      TCS S3          Filter select
GPIO 19      TCS OUT         Square wave output (3.3V safe)
```

> **Warning:** HC-SR04 ECHO outputs **5 V**. Connect a **1 kOhm + 2 kOhm** resistor voltage divider before GPIO 18 to avoid damaging the ESP32.

---

## Project Structure

```
SmartLineFollowerRobot/
├── platformio.ini          # PlatformIO build config + library deps
├── include/
│   ├── config.h            # All pin numbers, LEDC channels, EEPROM constants
│   ├── params.h            # Tunable Params struct + extern P declaration
│   ├── motors.h            # L298N motor driver API
│   ├── sensors.h           # 5-ch IR sensor API + SeenSide enum
│   ├── ultrasonic.h        # HC-SR04 distance sensor API
│   ├── color.h             # TCS3200 colour sensor API + ColorResult enum
│   ├── servo_gate.h        # Single-servo drop-gate API
│   └── robot.h             # RobotState enum + task function API
└── src/
    ├── main.cpp            # Top-level state machine + end-zone drop logic
    ├── params.cpp          # Default values, EEPROM load/save, SET handler
    ├── motors.cpp          # LEDC PWM primitives + movement functions
    ├── sensors.cpp         # ADC read, normalise, calibrate, PID position
    ├── ultrasonic.cpp      # Timed trigger/echo + 50 ms cached read
    ├── color.cpp           # TCS3200 channel read + colour classify
    ├── servo_gate.cpp      # Servo init, open, close, slow sweep
    └── robot.cpp           # PID follow, 4-stage recovery, avoid, pick
```

---

## Robot State Machine

```
BOOT
 |
 v
[ST_LINE_FOLLOW] <-----------------+------------------+
 | dist < obstacleSlowDist (17cm)  |                  |
 v                                 |                  |
[ST_OBSTACLE_SLOW]                 |                  |
 | dist <= colorCheckDist (9cm)    |                  |
 v                                 |                  |
[ST_COLOR_DETECT]                  |                  |
 |           |                     |                  |
 | RED        | GREEN              |                  |
 v            v                    |                  |
[ST_RED_AVOID] [ST_GREEN_PICK]     |                  |
 |              | cubeOnBoard=true |                  |
 +---->>--------+------------------+                  |
                                                      |
        (all sensors dark + clear ahead)              |
                    |                                 |
                    v                                 |
               [END ZONE DROP]                        |
               servo_open -> delay -> servo_close     |
               Robot halts --------------------------->
```

---

## Getting Started

### Prerequisites

- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) (VS Code extension recommended)
- USB-to-Serial driver for your ESP32 board

### 1. Clone & Open

```bash
git clone https://github.com/Pasinduranasinghe2001/SmartLineFollowerRobot.git
cd SmartLineFollowerRobot
git checkout dev
code .   # opens in VS Code with PlatformIO
```

### 2. Build & Upload

```
PlatformIO sidebar -> Project Tasks -> esp32dev -> Build
PlatformIO sidebar -> Project Tasks -> esp32dev -> Upload
```

Or via CLI:

```bash
platformio run --environment esp32dev --target upload
```

### 3. Calibrate Sensors

Open Serial Monitor at **115200 baud**, then type:

```
CALIBRATE
```

Follow the two prompts:
1. Place all 5 IR sensors over the **dark floor** → press Enter
2. Place all 5 IR sensors over the **yellow/white line** → press Enter

Then save to EEPROM:

```
SAVE
```

---

## Serial Commands

All commands are case-insensitive and sent at 115200 baud.

| Command | Description |
|---|---|
| `CALIBRATE` | Run interactive 2-phase IR calibration |
| `STATUS` | Print all current parameter values |
| `SAVE` | Write calibration + params to EEPROM |
| `LOAD` | Load calibration + params from EEPROM |
| `SET KEY VALUE` | Change any parameter live (see table below) |

### Tunable Parameters

#### PID / Line Follow

| Key | Default | Description |
|---|---|---|
| `BASE` | 110 | Normal forward PWM speed |
| `KP` | 16.0 | Proportional gain |
| `KD` | 10.0 | Derivative gain |
| `FILTER` | 0.65 | Low-pass alpha (0=raw, 1=frozen) |
| `WIDTHKP` | 6.0 | Width compensation gain |
| `MINSPEED` | 40 | Minimum PID output speed |
| `LTRIM` | 8 | Left motor PWM offset (balance) |
| `RTRIM` | 0 | Right motor PWM offset |

#### Recovery

| Key | Default | Description |
|---|---|---|
| `REVERSE` | 90 | Reverse speed during recovery |
| `REVERSE_BIAS` | 18 | PWM asymmetry when reversing with bias |
| `SEARCH` | 100 | Pivot search speed |
| `TIMEOUT_L` | 1800 | Max ms for left pivot |
| `TIMEOUT_R` | 4000 | Max ms for right pivot / reverse |
| `FORWARD_TIME` | 160 | ms to creep forward in recovery |

#### Obstacle / Colour / Servo

| Key | Default | Description |
|---|---|---|
| `APPSPD` | 70 | Approach speed near obstacle |
| `AVDSPD` | 100 | Speed during avoidance manoeuvre |
| `PCKSPD` | 50 | Speed during green-pick approach |
| `SLWDIST` | 17.0 | cm: start slowing (obstacle detected) |
| `COLDIST` | 9.0 | cm: stop and read colour sensor |
| `PCKDIST` | 5.5 | cm: close enough to scoop green cube |
| `T90TIME` | 650 | ms per 90 deg pivot in avoidance **tune first** |
| `REVTIME` | 700 | ms to reverse before side-step |
| `FWDTIME` | 1400 | ms forward past obstacle side |
| `REDTHR` | 120 | pulseIn threshold for red (lower = more red) |
| `GRNTHR` | 100 | pulseIn threshold for green |
| `SVHOME` | 109 | Servo home angle (gate CLOSED) |
| `SVPICK` | 183 | Servo pick angle (gate OPEN) |

**Example:**

```
SET KP 18.5
SET T90TIME 580
SET SVHOME 105
SAVE
```

---

## Tuning Guide

### Step 1 - Calibrate sensors
Always run `CALIBRATE` on the actual track surface under competition lighting. Thresholds are stored per-sensor, so the robot adapts to surface reflectance automatically.

### Step 2 - Tune T90TIME
Place the robot on a flat area, send `SET T90TIME 650`, run a test avoidance (`ST_RED_AVOID` triggered), and adjust by +/- 50 ms until both 90 deg pivots are accurate.

### Step 3 - Tune KP / KD
Start with `KP=10`, `KD=0`. Increase KP until the robot oscillates on a straight line, then back off 20%. Add KD (start at KP/2) to damp oscillation.

### Step 4 - Tune servo angles
Send `SET SVHOME 109` and `SET SVPICK 183`, manually test with the cube in the pocket, then `SAVE`.

---

## Architecture Notes

### Why modular `.h` / `.cpp` pairs?
Each subsystem (motors, sensors, ultrasonic, colour, servo, robot logic) is completely isolated. You can swap the motor driver from L298N to DRV8833 by only editing `motors.cpp` and `config.h`.

### Why LEDC instead of analogWrite?
The ESP32 does not have Arduino-style `analogWrite()`. The LEDC peripheral provides hardware PWM on any GPIO. Motors use channels 0 & 1 (timers 0 & 1). The servo uses timers 2 & 3 to avoid conflicts.

### Why is ECHO on a voltage divider?
The HC-SR04 ECHO pin outputs 5 V. ESP32 GPIO inputs are only 3.3 V tolerant. A 1 kOhm + 2 kOhm resistor divider reduces 5 V to 3.3 V safely.

### End-zone drop logic
After picking a green cube, `cubeOnBoard = true`. In every loop iteration, if all 5 IR sensors read dark **and** the ultrasonic sees no obstacle ahead, the robot has passed the end of the line. It stops, opens the gate for 1.2 seconds, closes it again, and halts.

---

## Dependencies

| Library | Source | Purpose |
|---|---|---|
| `ESP32Servo` v0.13+ | [madhephaestus/ESP32Servo](https://github.com/madhephaestus/ESP32Servo) | Servo control via LEDC timers |
| `EEPROM` | Built-in (ESP32 Arduino core) | Parameter persistence |
| `Arduino.h` | ESP32 Arduino core | GPIO, ADC, Serial, millis() |

Installed automatically by PlatformIO from `platformio.ini`:

```ini
lib_deps =
    madhephaestus/ESP32Servo@^0.13.0
```

---

## Contributing

1. Fork the repository
2. Create a feature branch from `dev`: `git checkout -b feature/my-improvement dev`
3. Commit with clear messages
4. Open a pull request targeting the `dev` branch

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgements

Developed as part of the **EC6090 Robotics and Automation** module,  
Faculty of Engineering, University of Ruhuna, Sri Lanka.
