// =========================================================================
//  config.cpp  –  Single authoritative definitions for all pin constants
//  EC6090 Mini-Project  |  ESP32 DevKitC v4  (30-PIN WROOM-32)
//
//  ⚠  PIN REMAP for 30-pin board (GPIO 22 & 23 not on header):
//       PIN_IN4  : was GPIO 23  →  now GPIO 17
//       PIN_SERVO: was GPIO 22  →  now GPIO 16
//
//  To revert to 38-pin assignments change 17→23 and 16→22 here only.
// =========================================================================
#include "config.h"

// ── IR Sensors ────────────────────────────────────────────────────────────
//  GPIO 34,35,32,33,25 = input-only ADC pins, all present on 30-pin board
const int IR_PIN[5] = { 34, 35, 32, 33, 25 };

// ── Right Motor  (L298N channel A) ────────────────────────────────────────
const int PIN_ENA = 14;   // PWM enable – LEDC channel 0
const int PIN_IN1 = 27;
const int PIN_IN2 = 26;

// ── Left Motor  (L298N channel B) ─────────────────────────────────────────
const int PIN_ENB = 12;   // PWM enable – LEDC channel 1
const int PIN_IN3 = 13;
const int PIN_IN4 = 17;   // ⚠ REMAPPED from 23 → 17 (23 not on 30-pin header)

// ── Servo ──────────────────────────────────────────────────────────────────
const int PIN_SERVO = 16; // ⚠ REMAPPED from 22 → 16 (22 not on 30-pin header)

// ── HC-SR04 Ultrasonic ────────────────────────────────────────────────────
const int PIN_TRIG =  5;
const int PIN_ECHO = 18;  // ⚠ 5V output – use voltage divider → 3.3V

// ── TCS3200 Color Sensor ──────────────────────────────────────────────────
const int PIN_CS_S0  =  4;
const int PIN_CS_S1  =  2;  // boot-strapping pin – keep LOW during flash
const int PIN_CS_S2  = 15;
const int PIN_CS_S3  = 21;
const int PIN_CS_OUT = 19;
