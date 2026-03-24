// =========================================================================
//  servo_gate.cpp  –  Single-servo drop-gate (pick & place)
//
//  One servo controls a flap at the front of the cube pocket:
//    HOME angle  →  gate CLOSED  (cube is held inside the pocket)
//    PICK angle  →  gate OPEN    (cube drops out / is released)
//
//  servoHomeAngle and servoPickAngle are tunable live via Serial:
//    SET SVHOME 109
//    SET SVPICK 183
// =========================================================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include "servo_gate.h"
#include "config.h"
#include "params.h"

static Servo _servo;

void servo_init() {
    // Allocate LEDC timers 2 & 3 for ESP32Servo
    // (timers 0 & 1 are already used by motors_init LEDC channels)
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    _servo.setPeriodHertz(50);                        // standard 50 Hz servo
    _servo.attach(PIN_SERVO, 500, 2400);              // pulse width µs range
    _servo.write(P.servoHomeAngle);                   // start closed
    Serial.printf("[SERVO] Init. Home=%d° Pick=%d°\n",
                  P.servoHomeAngle, P.servoPickAngle);
}

// Sweep gradually to avoid shocking the cube out of position
void servo_moveTo(int fromAngle, int toAngle, int stepDelayMs) {
    if (toAngle >= fromAngle) {
        for (int a = fromAngle; a <= toAngle; a++) { _servo.write(a); delay(stepDelayMs); }
    } else {
        for (int a = fromAngle; a >= toAngle; a--) { _servo.write(a); delay(stepDelayMs); }
    }
}

void servo_close() {
    Serial.printf("[SERVO] Closing (home %d°)\n", P.servoHomeAngle);
    _servo.write(P.servoHomeAngle);
}

void servo_open() {
    Serial.printf("[SERVO] Opening (pick %d°)\n", P.servoPickAngle);
    _servo.write(P.servoPickAngle);
}
