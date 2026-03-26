// =========================================================================
//  config.cpp  –  Single authoritative definitions for all pin constants
//  EC6090 Mini-Project  |  ESP32 Dev Module (38-pin, WROOM-32)
//
//  FINAL TESTED PIN MAP (confirmed working on physical hardware)
//  Do NOT change these without re-testing the full wiring.
// =========================================================================
#include "config.h"

// ── IR Sensors ────────────────────────────────────────────────────────────
//  GPIO 32,33,34,35 are input-only ADC pins; GPIO 27 is bidirectional.
//  All five are safe for analogRead() only — do not set as OUTPUT.
const int IR_PIN[7] = { 32, 33, 34, 35, 27, 36, 39 };   // S1(left) … S5(right)

// ── Right Motor  (L298N channel A) ────────────────────────────────────────
const int PIN_ENA =  5;    // PWM enable – LEDC channel 0
const int PIN_IN1 = 18;    // direction A
const int PIN_IN2 = 19;    // direction B

// ── Left Motor  (L298N channel B) ─────────────────────────────────────────
const int PIN_ENB = 23;    // PWM enable – LEDC channel 1
const int PIN_IN3 = 21;    // direction A
const int PIN_IN4 = 22;    // direction B

// ── Servo  (single-servo drop-gate gripper) ───────────────────────────────
const int PIN_SERVO = 13;

// ── HC-SR04 Ultrasonic ────────────────────────────────────────────────────
//  ⚠  ECHO outputs 5 V – protect GPIO16 with a 1 kΩ / 2 kΩ voltage divider
const int PIN_TRIG = 17;
const int PIN_ECHO = 16;

// ── TCS3200 Color Sensor ──────────────────────────────────────────────────
//  Frequency scale configured as 20%: S0=HIGH, S1=LOW
//  ⚠  GPIO 2 (PIN_CS_S1) is a boot-strapping pin – keep LOW during flash
const int PIN_CS_S0  = 14;
const int PIN_CS_S1  = 15;   //  ← was GPIO2; moved to free strapping pin
const int PIN_CS_S2  = 26;
const int PIN_CS_S3  = 25;
const int PIN_CS_OUT =  2;   //  ← TCS3200 OUT; GPIO2 as input is fine
