// =========================================================================
//  config.h  –  All pin assignments, LEDC channels, EEPROM constants
//  EC6090 Mini-Project  |  ESP32 DevKit
// =========================================================================
#pragma once
#include <Arduino.h>

// ─── IR Sensors  (input-only GPIOs on ESP32) ─────────────────────────────
//  S1 = far-left   …   S5 = far-right
static const int IR_PIN[5];

// ─── Right Motor  (L298N channel A) ──────────────────────────────────────
static const int PIN_ENA = 14;   // PWM via LEDC
static const int PIN_IN1 = 27;
static const int PIN_IN2 = 26;

// ─── Left Motor  (L298N channel B) ───────────────────────────────────────
static const int PIN_ENB = 12;   // PWM via LEDC
static const int PIN_IN3 = 13;
static const int PIN_IN4 = 23;

// ─── Servo  (single servo drop-gate / gripper) ───────────────────────────
static const int PIN_SERVO = 22;

// ─── HC-SR04 Ultrasonic ──────────────────────────────────────────────────
//  ⚠  ECHO outputs 5 V – use a 1kΩ / 2kΩ resistor voltage divider to 3.3 V
static const int PIN_TRIG = 5;
static const int PIN_ECHO = 18;

// ─── TCS3200 Color Sensor ────────────────────────────────────────────────
//  Frequency scale: S0=HIGH, S1=LOW  →  20 %
//  ⚠  GPIO 2 (CS_S1) is a strapping pin; set it LOW after boot
static const int PIN_CS_S0  =  4;
static const int PIN_CS_S1  =  2;
static const int PIN_CS_S2  = 15;
static const int PIN_CS_S3  = 21;
static const int PIN_CS_OUT = 19;

// ─── LEDC (ESP32 PWM) ─────────────────────────────────────────────────────
#define LEDC_FREQ        5000   // 5 kHz
#define LEDC_RESOLUTION     8  // 8-bit → 0-255
#define LEDC_CH_ENA         0  // channel for right motor
#define LEDC_CH_ENB         1  // channel for left motor

// ─── EEPROM ───────────────────────────────────────────────────────────────
#define EEPROM_SIZE         512
#define EEPROM_MAGIC       0xAC
#define EEPROM_ADDR_MAGIC    0
#define EEPROM_ADDR_CAL      1   // 5 × 3 × sizeof(int) = 60 bytes
#define EEPROM_ADDR_PARAMS 100

// ─── Physical motor inversion (flip if wheel spins backwards) ────────────
#define PHYS_RIGHT_INVERT false
#define PHYS_LEFT_INVERT  false
