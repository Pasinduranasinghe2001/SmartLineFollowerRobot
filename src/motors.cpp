// =========================================================================
//  motors.cpp  –  L298N motor driver via ESP32 LEDC PWM
//
//  Physical layout:
//    Right motor → L298N channel A (ENA, IN1, IN2)
//    Left  motor → L298N channel B (ENB, IN3, IN4)
//
//  All public functions apply leftTrim / rightTrim from Params
//  and honour PHYS_RIGHT_INVERT / PHYS_LEFT_INVERT from config.h
//
//  BUG-3 FIX (direction-change shoot-through)
//  ─────────────────────────────────────────────────────────────────────
//  PROBLEM (before fix):
//    motors_setRight/Left() wrote the direction pins (IN1/IN2, IN3/IN4)
//    WHILE the LEDC PWM was still running at the previous duty cycle.
//    During the nanoseconds between the two digitalWrite() calls the
//    L298N briefly saw one input HIGH and one already changed — a partial
//    direction state with live current flowing.  This caused:
//      • Audible buzz / whine from motor coils and driver internals
//      • Current spike through L298N output transistors
//      • Risk of driver latch-up on repeated direction reversals
//
//  FIX:
//    Zero the PWM duty (_pwmRight/Left(0)) FIRST, then set the direction
//    pins, then restore the target duty.  The L298N enable line is dead
//    during the direction transition so no shoot-through is possible.
//    Order: kill PWM → set IN pins → apply PWM.
//
//  BUG-4 FIX (motors_stop() leaving ENA/ENB floating)
//  ─────────────────────────────────────────────────────────────────────
//  PROBLEM (before fix):
//    ledcWrite(ch, 0) is supposed to hold the LEDC output LOW (0 % duty).
//    On some ESP32 Arduino core versions the LEDC timer keeps running and
//    the GPIO output floats briefly between pulses rather than being driven
//    cleanly LOW.  The L298N ENA/ENB pin therefore sees occasional
//    glitch pulses, keeping the H-bridge partially active even after
//    motors_stop() returns.  Symptoms: motor twitches, unexpected motion,
//    or a continuous low-level hum after stop is called.
//
//  FIX:
//    After ledcWrite(ch, 0), call digitalWrite(PIN_ENA/ENB, LOW) to force
//    the GPIO driver LOW through the pad multiplexer.  This overrides the
//    LEDC peripheral output and guarantees the enable line is de-asserted.
//    On the next ledcWrite(ch, nonzero) call the LEDC peripheral
//    automatically reclaims the pin — no need to re-attach.
// =========================================================================
#include <Arduino.h>
#include "motors.h"
#include "config.h"
#include "params.h"

// ── LEDC write wrappers (keeps 0-255 style interface) ────────────────────
static inline void _pwmRight(int s) {
    ledcWrite(LEDC_CH_ENA, (uint32_t)constrain(s, 0, 255));
}
static inline void _pwmLeft(int s) {
    ledcWrite(LEDC_CH_ENB, (uint32_t)constrain(s, 0, 255));
}

// ── Primitives  (BUG-3 FIXED: kill PWM → set direction → apply PWM) ─────
void motors_setRight(int spd, bool forward) {
    _pwmRight(0);                                          // 1. kill PWM first
    bool fwd = PHYS_RIGHT_INVERT ? !forward : forward;
    digitalWrite(PIN_IN1, fwd ? LOW  : HIGH);              // 2. set direction
    digitalWrite(PIN_IN2, fwd ? HIGH : LOW);
    _pwmRight(constrain(spd + P.rightTrim, 0, 255));       // 3. apply speed
}

void motors_setLeft(int spd, bool forward) {
    _pwmLeft(0);                                           // 1. kill PWM first
    bool fwd = PHYS_LEFT_INVERT ? !forward : forward;
    digitalWrite(PIN_IN3, fwd ? LOW  : HIGH);              // 2. set direction
    digitalWrite(PIN_IN4, fwd ? HIGH : LOW);
    _pwmLeft(constrain(spd + P.leftTrim, 0, 255));         // 3. apply speed
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

// ── Stop  (BUG-4 FIXED: force ENA/ENB GPIO LOW after ledcWrite 0) ────────
void motors_stop() {
    // Zero LEDC duty first so no further PWM pulses are generated
    ledcWrite(LEDC_CH_ENA, 0);
    ledcWrite(LEDC_CH_ENB, 0);

    // Force enable pins LOW through GPIO pad driver.
    // Overrides any residual LEDC glitch pulses on the ENA/ENB lines.
    digitalWrite(PIN_ENA, LOW);
    digitalWrite(PIN_ENB, LOW);

    // Brake both channels: IN1=IN2=LOW, IN3=IN4=LOW → L298N fast-decay
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// ── High-level movement API ───────────────────────────────────────────────
void motors_driveStraight(int spd) {
    motors_setRight(spd, true);
    motors_setLeft(spd,  true);
}

void motors_driveBackward(int spd) {
    motors_setRight(spd, false);
    motors_setLeft(spd,  false);
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
