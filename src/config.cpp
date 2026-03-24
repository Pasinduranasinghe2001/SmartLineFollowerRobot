// =========================================================================
//  config.cpp  –  Single authoritative definitions for all pin constants
//  EC6090 Mini-Project  |  ESP32 DevKit
//
//  BUG-01 FIX:
//  This file provides ONE definition for every symbol declared `extern`
//  in include/config.h.  Before this fix, all constants were declared
//  `static const` inside config.h, which caused every .cpp translation
//  unit that included config.h to silently get its own private copy.
//  For scalar ints the compiler can optimise them away, but for the
//  IR_PIN[5] array each translation unit held a separate copy in flash,
//  wasting memory and risking subtle aliasing when a pointer to the array
//  was passed across compilation units.
//
//  Rule of thumb:
//    Header  →  `extern const`  (declaration – no storage allocated)
//    This file  →  `const`  (definition – storage allocated exactly once)
// =========================================================================
#include "config.h"

// ─── IR Sensors ───────────────────────────────────────────────────────────────
//  S1(left) … S5(right) — GPIO 34, 35, 32, 33, 25 are input-only on ESP32
const int IR_PIN[5] = { 34, 35, 32, 33, 25 };

// ─── Right Motor  (L298N channel A) ──────────────────────────────────────
const int PIN_ENA = 14;   // PWM enable – LEDC channel 0
const int PIN_IN1 = 27;   // direction bit A
const int PIN_IN2 = 26;   // direction bit B

// ─── Left Motor  (L298N channel B) ───────────────────────────────────────
const int PIN_ENB = 12;   // PWM enable – LEDC channel 1
const int PIN_IN3 = 13;   // direction bit A
const int PIN_IN4 = 23;   // direction bit B

// ─── Servo ────────────────────────────────────────────────────────────────────
const int PIN_SERVO = 22; // SG90 / MG90S signal pin

// ─── HC-SR04 Ultrasonic ──────────────────────────────────────────────────
const int PIN_TRIG =  5;  // trigger output (10 µs pulse)
const int PIN_ECHO = 18;  // echo input  ⚠ 5V → use voltage divider → 3.3V

// ─── TCS3200 Color Sensor ────────────────────────────────────────────────
const int PIN_CS_S0  =  4;  // frequency scale select 0
const int PIN_CS_S1  =  2;  // frequency scale select 1  ⚠ strapping pin
const int PIN_CS_S2  = 15;  // photodiode filter select 0
const int PIN_CS_S3  = 21;  // photodiode filter select 1
const int PIN_CS_OUT = 19;  // square wave frequency output (3.3V safe)
