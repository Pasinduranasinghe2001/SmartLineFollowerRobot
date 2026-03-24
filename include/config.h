// =========================================================================
//  config.h  –  All pin assignments, LEDC channels, EEPROM constants
//  EC6090 Mini-Project  |  ESP32 DevKit
//
//  BUG-01 FIX:
//  All integer pin constants are declared extern here and DEFINED ONCE
//  in src/config.cpp.  Using `static const` in a header causes every
//  translation unit (.cpp) that includes this file to get its own private
//  copy of the variable — this is an ODR (One Definition Rule) violation
//  that bloats flash and can cause subtle aliasing bugs, especially with
//  arrays like IR_PIN[5].
//
//  #define constants (LEDC, EEPROM, inversion flags) are fine in headers
//  because the preprocessor handles them before the compiler and they
//  never produce multiple object symbols.
// =========================================================================
#pragma once
#include <Arduino.h>

// ─── IR Sensors  (input-only GPIOs on ESP32) ─────────────────────────────
//  S1 = far-left   …   S5 = far-right
extern const int IR_PIN[5];          // { 34, 35, 32, 33, 25 }

// ─── Right Motor  (L298N channel A) ──────────────────────────────────────
extern const int PIN_ENA;            // 14  – PWM via LEDC
extern const int PIN_IN1;            // 27
extern const int PIN_IN2;            // 26

// ─── Left Motor  (L298N channel B) ───────────────────────────────────────
extern const int PIN_ENB;            // 12  – PWM via LEDC
extern const int PIN_IN3;            // 13
extern const int PIN_IN4;            // 23

// ─── Servo  (single servo drop-gate / gripper) ───────────────────────────
extern const int PIN_SERVO;          // 22

// ─── HC-SR04 Ultrasonic ──────────────────────────────────────────────────
//  ⚠  ECHO outputs 5 V – use a 1 kΩ / 2 kΩ resistor voltage divider → 3.3 V
extern const int PIN_TRIG;           // 5
extern const int PIN_ECHO;           // 18

// ─── TCS3200 Color Sensor ────────────────────────────────────────────────
//  Frequency scale: S0=HIGH, S1=LOW  →  20 %
//  ⚠  GPIO 2 (PIN_CS_S1) is a boot strapping pin; driven LOW after setup()
extern const int PIN_CS_S0;          //  4
extern const int PIN_CS_S1;          //  2  ← strapping pin
extern const int PIN_CS_S2;          // 15
extern const int PIN_CS_S3;          // 21
extern const int PIN_CS_OUT;         // 19

// ─── LEDC (ESP32 hardware PWM) ────────────────────────────────────────────
//  #define is fine here — preprocessor tokens, not linker symbols
#define LEDC_FREQ           5000     // 5 kHz carrier frequency
#define LEDC_RESOLUTION        8     // 8-bit resolution → duty 0–255
#define LEDC_CH_ENA            0     // LEDC channel 0 → right motor (ENA)
#define LEDC_CH_ENB            1     // LEDC channel 1 → left  motor (ENB)
//  Note: ESP32Servo uses LEDC timers 2 & 3 (allocated in servo_gate.cpp)

// ─── EEPROM ───────────────────────────────────────────────────────────────
#define EEPROM_SIZE          512
#define EEPROM_MAGIC        0xAC
#define EEPROM_ADDR_MAGIC      0
#define EEPROM_ADDR_CAL        1     // 5 sensors × 3 ints × 4 bytes = 60 B
#define EEPROM_ADDR_PARAMS   100

// ─── Physical motor inversion ─────────────────────────────────────────────
//  Set to true if a wheel spins backwards relative to expected direction
#define PHYS_RIGHT_INVERT  false
#define PHYS_LEFT_INVERT   false
