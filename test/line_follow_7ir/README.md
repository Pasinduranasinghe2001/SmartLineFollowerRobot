# 7-IR Line Follower Test

Standalone test sketch for validating the two newly added outer IR sensors
(S6=GPIO36, S7=GPIO39) before merging into the main project.

## How to run

### 1. Add env to `platformio.ini`

```ini
[env:test_7ir]
platform  = espressif32
board     = esp32dev
framework = arduino
build_src_filter = -<*> +<../test/line_follow_7ir/>
monitor_speed = 115200
lib_deps  =
    madhephaestus/ESP32Servo @ ^0.13.0
```

### 2. Select the env and upload

In PlatformIO IDE bottom toolbar click the env selector (shows `env:esp32dev`)
and switch to `env:test_7ir`. Then click Upload.

### 3. Open Serial Monitor at 115200

```
COMMANDS
  CALIBRATE   2-phase floor/line calibration for all 7 sensors
  STATUS      print all params + live sensor readings + visual bitmap
  SAVE        write calibration + params to EEPROM
  LOAD        restore from EEPROM
  HELP        list all SET keys

SET KEYS
  BASE, MIN, SEARCH, REVERSE, FWDREC, TIMEOUT
  KP, KD, FILTER, SPEEDDROP, LTRIM, RTRIM
```

## New GPIO pins

| Sensor | GPIO | Note |
|--------|------|------|
| S6 | GPIO 36 | VP pin, input-only, ADC1_CH0 |
| S7 | GPIO 39 | VN pin, input-only, ADC1_CH3 |

Both are ADC1 channels so they work correctly with `analogRead()` without
affecting Wi-Fi (ADC2 restriction does not apply here).

## PID tuning note

The weight span is now **−6 … +6** (was −4 … +4 with 5 sensors).  
The wider span means PID corrections are proportionally larger when the
line is near the outer sensors. Start with:

```
SET KP 12
SET KD 4
SET SPEEDDROP 4
```

and increase KP gradually until the robot follows smoothly without oscillating.

## What the verbose output looks like

```
[T] .X.X...  pos=-1.50  recov=0
[T] ..XX...  pos=-0.80  recov=0
[T] ...X...  pos=+0.00  recov=0
[T] ...XX..  pos=+0.90  recov=0
```

X = sensor sees line, `.` = background.  
Outermost X positions confirm S6/S7 are working.
