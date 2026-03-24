// =========================================================================
//  config.h  –  All pin assignments, LEDC channels, EEPROM constants
//  EC6090 Mini-Project  |  ESP32 DevKitC v4  (30-PIN WROOM-32)
//
//  30-PIN BOARD PIN AVAILABILITY AUDIT
//  ─────────────────────────────────────────────────────────────────────
//  The 30-pin DevKit exposes these GPIO numbers on its two rows of 15:
//
//  Left row  (top→bottom): 3V3  GND  GPIO15  GPIO2   GPIO0   GPIO4
//                          GPIO16  GPIO17  GPIO5   GPIO18  GPIO19
//                          GPIO21  RX0(3)  TX0(1)  GND    5V
//
//  Right row (top→bottom): 3V3  GND  GPIO13  GPIO12  GPIO14  GPIO27
//                          GPIO26  GPIO25  GPIO33  GPIO32  GPIO35
//                          GPIO34  GPIO39  GPIO36  GND    VIN
//
//  ✅ All pins used in this project are present on the 30-pin board:
//     IR  : 34, 35, 32, 33, 25  – right row, input-only ADC pins  ✅
//     ENA : 14                  – right row                        ✅
//     IN1 : 27                  – right row                        ✅
//     IN2 : 26                  – right row                        ✅
//     ENB : 12                  – right row                        ✅
//     IN3 : 13                  – right row                        ✅
//     IN4 : 23                  – NOT on 30-pin header!            ⚠ SEE BELOW
//     SERVO: 22                 – NOT on 30-pin header!            ⚠ SEE BELOW
//     TRIG: 5                   – left row                         ✅
//     ECHO: 18                  – left row                         ✅
//     CS_S0: 4                  – left row                         ✅
//     CS_S1: 2                  – left row (strapping pin)         ✅
//     CS_S2: 15                 – left row                         ✅
//     CS_S3: 21                 – left row                         ✅
//     CS_OUT:19                 – left row                         ✅
//
//  ⚠  GPIO 22 and GPIO 23 are NOT broken out on the 30-pin board header.
//     They are internally present on the chip but physically inaccessible
//     on the 30-pin PCB edge connector.
//
//     RECOMMENDED REMAPS for 30-pin board:
//       PIN_IN4  : GPIO23 → GPIO17   (right of GPIO16 on left row)
//       PIN_SERVO: GPIO22 → GPIO16   (available on left row)
//
//     GPIO 16 & 17 are free on WROOM-32 (not used by internal flash).
//     Change the values in src/config.cpp and re-flash.
//
//  ⚠  GPIO 34, 35, 36, 39 are INPUT-ONLY (no internal pull-up/down,
//     no OUTPUT capability).  Used here only as analogRead() IR inputs.
//
//  ⚠  GPIO 2 (CS_S1) is a boot-strapping pin.  It must be LOW or floating
//     during reset/flash.  The TCS3200 drives it LOW at 20% scale which is
//     correct, but do NOT connect a pull-up to this pin.
// =========================================================================
#pragma once
#include <Arduino.h>

// ─── IR Sensors  (input-only GPIOs – right row of 30-pin board) ──────────
//  S1 = far-left   …   S5 = far-right
extern const int IR_PIN[5];          // { 34, 35, 32, 33, 25 }  ✅ all present

// ─── Right Motor  (L298N channel A) ──────────────────────────────────────
extern const int PIN_ENA;            // 14  – PWM via LEDC       ✅
extern const int PIN_IN1;            // 27                        ✅
extern const int PIN_IN2;            // 26                        ✅

// ─── Left Motor  (L298N channel B) ───────────────────────────────────────
extern const int PIN_ENB;            // 12  – PWM via LEDC       ✅
extern const int PIN_IN3;            // 13                        ✅
extern const int PIN_IN4;            // ⚠ was 23 (not on 30-pin header)
                                     //   REMAPPED → 17  (left row, free GPIO)

// ─── Servo  (single servo drop-gate / gripper) ───────────────────────────
extern const int PIN_SERVO;          // ⚠ was 22 (not on 30-pin header)
                                     //   REMAPPED → 16  (left row, free GPIO)

// ─── HC-SR04 Ultrasonic ──────────────────────────────────────────────────
//  ⚠  ECHO outputs 5 V – use 1 kΩ / 2 kΩ voltage divider → 3.3 V
extern const int PIN_TRIG;           //  5  ✅
extern const int PIN_ECHO;           // 18  ✅

// ─── TCS3200 Color Sensor ────────────────────────────────────────────────
//  Frequency scale: S0=HIGH, S1=LOW  →  20 %
//  ⚠  GPIO 2 (PIN_CS_S1) is a boot-strapping pin; keep LOW during flash
extern const int PIN_CS_S0;          //  4  ✅
extern const int PIN_CS_S1;          //  2  ✅ (strapping – see note above)
extern const int PIN_CS_S2;          // 15  ✅
extern const int PIN_CS_S3;          // 21  ✅
extern const int PIN_CS_OUT;         // 19  ✅

// ─── LEDC (ESP32 hardware PWM) ────────────────────────────────────────────
#define LEDC_FREQ           5000     // 5 kHz carrier
#define LEDC_RESOLUTION        8     // 8-bit → duty 0–255
#define LEDC_CH_ENA            0     // channel 0 → right motor ENA
#define LEDC_CH_ENB            1     // channel 1 → left  motor ENB
//  ESP32Servo uses LEDC timers 2 & 3 (allocated in servo_gate.cpp)

// ─── EEPROM ───────────────────────────────────────────────────────────────
#define EEPROM_SIZE          512
#define EEPROM_MAGIC        0xAD     // bumped 0xAC→0xAD: pin remap invalidates old saves
#define EEPROM_ADDR_MAGIC      0
#define EEPROM_ADDR_CAL        1     // 5 sensors × 3 ints × 4 bytes = 60 B
#define EEPROM_ADDR_PARAMS   100

// ─── Physical motor inversion ─────────────────────────────────────────────
#define PHYS_RIGHT_INVERT  false
#define PHYS_LEFT_INVERT   false
