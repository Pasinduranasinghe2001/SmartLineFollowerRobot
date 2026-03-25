# SmartLineFollowerRobot

> **EC6090 Robotics & Automation | ESP32-based autonomous robot**  
> 5-sensor PID line following • ultrasonic obstacle detection • TCS3200 colour-based pick-and-place

[![PlatformIO](https://img.shields.io/badge/built%20with-PlatformIO-orange)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/board-ESP32%20Dev%20Module%2038--pin-blue)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
[![Framework](https://img.shields.io/badge/framework-Arduino-teal)](https://docs.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

## Overview

SmartLineFollowerRobot is a modular, fully-tunable autonomous robot firmware written in C++ for the **ESP32 Dev Module (38-pin, WROOM-32)** using the **PlatformIO + Arduino** framework.

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
- **Debounced end-zone drop** — all-dark condition must persist ≥ 500 ms before cube is released (BUG-03 fix)
- **EEPROM persistence** — all calibration data and tuning parameters survive power cycles
- **Live Serial tuning** — change any parameter at runtime with `SET KEY VALUE` without re-uploading
- **Auto-calibration** — interactive 2-phase floor/line calibration adapts to any surface
- **Full debug logging** — `CORE_DEBUG_LEVEL=5` with timestamped, coloured Serial monitor output

---

## Hardware

### Bill of Materials

| Component | Model | Qty |
|---|---|---|
| Microcontroller | ESP32 Dev Module 38-pin (WROOM-32) | 1 |
| Motor driver | L298N dual H-bridge | 1 |
| Drive motors | DC geared motor (TT or N20) | 2 |
| Front wheel | Castor / omni ball | 1 |
| IR sensor array | BFD-1000 / TCRT5000 5-ch analog | 1 |
| Ultrasonic sensor | HC-SR04 | 1 |
| Colour sensor | TCS3200 | 1 |
| Servo (gripper gate) | SG90 / MG90S | 1 |
| Voltage divider | 1 kΩ + 2 kΩ resistors (ECHO line) | 1 set |
| Power | 7.4 V LiPo or 6× AA | 1 |

---

### Pin Mapping (Final Tested)

```
ESP32 GPIO   Signal           Notes
-----------  ---------------  -------------------------------------------------
GPIO 32      IR S1 (left)     Input-only GPIO — analogRead() only
GPIO 33      IR S2            Input-only GPIO
GPIO 34      IR S3            Input-only GPIO
GPIO 35      IR S4            Input-only GPIO
GPIO 27      IR S5 (right)    Bidirectional GPIO, used as input

GPIO  5      ENA (PWM)        LEDC channel 0, right motor (L298N ch-A)
GPIO 18      IN1              L298N ch-A direction
GPIO 19      IN2              L298N ch-A direction

GPIO 23      ENB (PWM)        LEDC channel 1, left motor (L298N ch-B)
GPIO 21      IN3              L298N ch-B direction
GPIO 22      IN4              L298N ch-B direction

GPIO 13      Servo signal     Single-servo drop-gate gripper (ESP32Servo)

GPIO 17      TRIG             HC-SR04 trigger pulse (10 µs HIGH)
GPIO 16      ECHO             HC-SR04 echo return
                              ⚠ USE 1kΩ + 2kΩ VOLTAGE DIVIDER (5V → 3.3V)

GPIO 14      TCS S0           Frequency scale select (S0=HIGH, S1=LOW → 20%)
GPIO 15      TCS S1           Frequency scale select
GPIO 26      TCS S2           Colour filter select
GPIO 25      TCS S3           Colour filter select
GPIO  2      TCS OUT          Square wave output (3.3V safe as INPUT)
                              ⚠ Boot-strapping pin — do NOT add pull-up
```

> **Warning:** HC-SR04 ECHO outputs **5 V**. Connect a **1 kΩ + 2 kΩ** resistor voltage divider before GPIO 16 to protect the ESP32 input.

> **Warning:** GPIO 2 (TCS OUT) is an ESP32 boot-strapping pin. Using it as an INPUT is safe; do NOT connect a pull-up resistor. If GPIO 2 is HIGH at boot, the chip enters download mode.

> **Note:** GPIO 32, 33, 34, 35 are input-only. They have no internal pull-up or pull-down resistors and cannot be configured as OUTPUT.

---

## Project Structure

```
SmartLineFollowerRobot/
├── platformio.ini          # PlatformIO build config — board: esp32dev (38-pin)
├── include/
│   ├── config.h            # extern pin declarations + LEDC / EEPROM #defines
│   ├── params.h            # Tunable Params struct + extern P declaration
│   ├── motors.h            # L298N motor driver API
│   ├── sensors.h           # 5-ch IR sensor API + SeenSide enum
│   ├── ultrasonic.h        # HC-SR04 distance sensor API
│   ├── color.h             # TCS3200 colour sensor API + ColorResult enum
│   ├── servo_gate.h        # Single-servo drop-gate API
│   └── robot.h             # RobotState enum + task function API
└── src/
    ├── main.cpp            # Top-level state machine + debounced end-zone drop
    ├── config.cpp          # Single authoritative pin constant definitions (BUG-01)
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
   (all sensors dark + clear ahead, DEBOUNCED 500ms)  |
                    |                                 |
                    v                                 |
               [END ZONE DROP]                        |
               servo_open -> 1.2s -> servo_close      |
               Robot halts --------------------------->
```

> **BUG-03 fix:** The end-zone drop is debounced — all 5 sensors must remain dark continuously for **500 ms** before the gate opens. This prevents false drops on tape gaps, T-junctions, or sensor bounce.

---

## Getting Started

### Prerequisites

- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) (VS Code extension recommended)
- USB-to-Serial driver for your ESP32 board (CH340 or CP2102 depending on your module)

### 1. Clone & Open

```bash
git clone https://github.com/Pasinduranasinghe2001/SmartLineFollowerRobot.git
cd SmartLineFollowerRobot
git checkout dev
code .   # opens in VS Code with PlatformIO
```

### 2. Set Your COM Port

Uncomment the port lines in `platformio.ini` for your OS:

```ini
; Windows
upload_port  = COM3
monitor_port = COM3

; Linux
; upload_port  = /dev/ttyUSB0

; macOS
; upload_port  = /dev/cu.usbserial-0001
```

### 3. Build & Upload

```bash
platformio run --environment esp32dev --target upload
```

Or use the PlatformIO sidebar in VS Code: **Project Tasks → esp32dev → Upload**

### 4. Calibrate Sensors

> ⚠️ **EEPROM_MAGIC was bumped to `0xAE`** in this update. Any previously saved EEPROM data will be ignored on first boot. You must run `CALIBRATE` and `SAVE` again after flashing.

Open Serial Monitor at **115200 baud**, then type:

```
CALIBRATE
```

Follow the two prompts:
1. Place all 5 IR sensors over the **dark floor** → press Enter
2. Place all 5 IR sensors over the **yellow/white line** → press Enter

Save to EEPROM:

```
SAVE
```

---

## Debug Logging

Debug logging is **enabled by default** (`CORE_DEBUG_LEVEL=5 VERBOSE`) in `platformio.ini`.  
The Serial monitor output is automatically:
- 🟥 **Coloured** (RED = error, YELLOW = warn, GREEN = info)
- ⏰ **Timestamped** (`HH:MM:SS.mmm` prefix on every line)
- 💾 **Saved to file** (`.pio/monitor.log`)
- 🔍 **Backtrace decoded** (crash addresses resolved to `file:line`)

### Debug Levels

| Level | Value | Use |
|---|---|---|
| NONE | `0` | Competition / final deployment |
| ERROR | `1` | Fatal errors only |
| WARN | `2` | Errors + warnings |
| INFO | `3` | Boot messages, state changes |
| DEBUG | `4` | Driver-level prints |
| **VERBOSE** | **`5`** | **Everything — use during development** |

Change in `platformio.ini`:
```ini
build_flags = -DCORE_DEBUG_LEVEL=5   ; development
;             -DCORE_DEBUG_LEVEL=0   ; competition
```

### Using Log Macros in Code

```cpp
static const char* TAG = "ROBOT";
ESP_LOGE(TAG, "Motor fault! spd=%d", spd);        // always shown
ESP_LOGW(TAG, "Recovery timeout %lu ms", dt);     // level >= 2
ESP_LOGI(TAG, "State -> OBSTACLE_SLOW");           // level >= 3
ESP_LOGD(TAG, "PID err=%.2f corr=%.2f", e, c);   // level >= 4
ESP_LOGV(TAG, "raw=%d str=%d", r, s);             // level >= 5
```

---

## Serial Commands

All commands are case-insensitive, sent at 115200 baud.

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
| `T90TIME` | 650 | ms per 90° pivot in avoidance — **tune first** |
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

### Step 1 — Calibrate sensors
Always run `CALIBRATE` on the actual track surface under competition lighting. Thresholds are stored per-sensor so the robot adapts to surface reflectance automatically.

### Step 2 — Tune T90TIME
Place the robot on a flat area and trigger a test avoidance. Adjust `T90TIME` by ±50 ms until both 90° pivots are accurate.

### Step 3 — Tune KP / KD
Start with `KP=10`, `KD=0`. Increase KP until the robot oscillates on a straight line, then back off 20%. Add KD (start at KP/2) to damp the oscillation.

### Step 4 — Tune servo angles
Send `SET SVHOME 109` and `SET SVPICK 183`, manually test with the cube in the pocket, then `SAVE`.

---

## Architecture Notes

### One definition per pin (BUG-01 fix)
All pin constants are declared `extern const int` in `include/config.h` and **defined exactly once** in `src/config.cpp`. The old `static const` in a header approach gave every `.cpp` file its own private copy of `IR_PIN[5]`, wasting flash and risking pointer aliasing across translation units.

### Debounced end-zone drop (BUG-03 fix)
After picking a green cube, `cubeOnBoard = true`. In every loop iteration, if all 5 IR sensors are dark **and** the ultrasonic sees no obstacle ahead, a 500 ms debounce timer starts. Only when the condition persists uninterrupted for the full 500 ms does the robot stop and open the gate. Any sensor detecting the line resets the timer immediately, preventing false drops on tape gaps or T-junctions.

### Why LEDC instead of analogWrite?
The ESP32 does not have Arduino-style `analogWrite()`. The LEDC peripheral provides hardware PWM on any GPIO. Motors use channels 0 & 1 (timers 0 & 1). The servo uses LEDC timers 2 & 3 to avoid conflicts.

### Why is ECHO on a voltage divider?
The HC-SR04 ECHO pin outputs 5 V. ESP32 GPIO inputs are only 3.3 V tolerant. A 1 kΩ + 2 kΩ resistor divider reduces 5 V to 3.3 V safely.

### GPIO 2 as TCS OUT
GPIO 2 is an ESP32 boot-strapping pin. When used as an **input** (reading the TCS3200 square wave output) it is perfectly safe. It only causes issues if driven HIGH externally during reset, which the TCS3200 output does not do at boot time. Do **not** add a pull-up resistor to this pin.

---

## Bug Fix Changelog

| ID | File(s) | Fix Summary |
|---|---|---|
| BUG-01 | `config.h`, `config.cpp` | Replace `static const` with `extern const` + single definition file to prevent ODR violations |
| BUG-02 | `main.cpp`, `robot.cpp` | Remove duplicate `servo_close()` from `ST_GREEN_PICK`; single authoritative call inside `robot_executeGreenPick()` |
| BUG-03 | `main.cpp` | Add 500 ms debounce timer to end-zone drop; prevents false release on tape gaps or sensor bounce |
| BUG-10 | `main.cpp` | Move `params_loadEEPROM()` before `servo_init()` so saved servo angles are applied on first write |
| PIN-01 | `config.cpp`, `platformio.ini` | Apply final tested 38-pin pin map; revert board to `esp32dev`; bump `EEPROM_MAGIC` to `0xAE` |

---

## Dependencies

| Library | Source | Purpose |
|---|---|---|
| `ESP32Servo` v0.13+ | [madhephaestus/ESP32Servo](https://github.com/madhephaestus/ESP32Servo) | Servo control via LEDC timers |
| `EEPROM` | Built-in (ESP32 Arduino core) | Parameter persistence |
| `Arduino.h` | ESP32 Arduino core | GPIO, ADC, Serial, millis() |

Installed automatically by PlatformIO:

```ini
lib_deps =
    madhephaestus/ESP32Servo @ ^0.13.0
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
