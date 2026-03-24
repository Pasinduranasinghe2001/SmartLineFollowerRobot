// =========================================================================
//  motors.cpp  –  L298N motor driver via ESP32 LEDC PWM
//
//  Physical layout:
//    Right motor → L298N channel A (ENA, IN1, IN2)
//    Left  motor → L298N channel B (ENB, IN3, IN4)
//
//  All public functions apply leftTrim / rightTrim from Params
//  and honour PHYS_RIGHT_INVERT / PHYS_LEFT_INVERT from config.h
// =========================================================================
#include <Arduino.h>
#include "motors.h"
#include "config.h"
#include "params.h"

// ── LEDC write wrapper (keeps 0-255 style interface) ─────────────────────
static inline void _pwmRight(int s) {
    ledcWrite(LEDC_CH_ENA, (uint32_t)constrain(s, 0, 255));
}
static inline void _pwmLeft(int s) {
    ledcWrite(LEDC_CH_ENB, (uint32_t)constrain(s, 0, 255));
}

// ── Primitives ───────────────────────────────────────────────────────────
void motors_setRight(int spd, bool forward) {
    spd = constrain(spd + P.rightTrim, 0, 255);
    bool fwd = PHYS_RIGHT_INVERT ? !forward : forward;
    digitalWrite(PIN_IN1, fwd ? LOW  : HIGH);
    digitalWrite(PIN_IN2, fwd ? HIGH : LOW);
    _pwmRight(spd);
}

void motors_setLeft(int spd, bool forward) {
    spd = constrain(spd + P.leftTrim, 0, 255);
    bool fwd = PHYS_LEFT_INVERT ? !forward : forward;
    digitalWrite(PIN_IN3, fwd ? LOW  : HIGH);
    digitalWrite(PIN_IN4, fwd ? HIGH : LOW);
    _pwmLeft(spd);
}

// ── Init ─────────────────────────────────────────────────────────────────
void motors_init() {
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

    ledcSetup(LEDC_CH_ENA, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(PIN_ENA, LEDC_CH_ENA);

    ledcSetup(LEDC_CH_ENB, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(PIN_ENB, LEDC_CH_ENB);

    motors_stop();
}

// ── High-level movement API ───────────────────────────────────────────────
void motors_stop() {
    _pwmRight(0);
    _pwmLeft(0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

void motors_driveStraight(int spd) {
    motors_setRight(spd, true);
    motors_setLeft(spd, true);
}

void motors_driveBackward(int spd) {
    motors_setRight(spd, false);
    motors_setLeft(spd, false);
}

void motors_driveBackwardBiasLeft(int fast, int slow) {
    // Left wheel faster backward → steer left while reversing
    motors_setRight(slow, false);
    motors_setLeft(fast,  false);
}

void motors_driveBackwardBiasRight(int fast, int slow) {
    // Right wheel faster backward → steer right while reversing
    motors_setRight(fast, false);
    motors_setLeft(slow,  false);
}

void motors_steerLeft(int fast, int slow) {
    // Right wheel faster → robot curves left
    motors_setRight(fast, true);
    motors_setLeft(slow,  true);
}

void motors_steerRight(int fast, int slow) {
    // Left wheel faster → robot curves right
    motors_setRight(slow, true);
    motors_setLeft(fast,  true);
}

void motors_pivotLeft(int spd) {
    // Right forward, left backward → pivot left in place
    motors_setRight(spd, true);
    motors_setLeft(spd,  false);
}

void motors_pivotRight(int spd) {
    // Left forward, right backward → pivot right in place
    motors_setRight(spd, false);
    motors_setLeft(spd,  true);
}
