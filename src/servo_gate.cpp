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
//
//  BUG-5 FIX — servo boot jerk
//  ─────────────────────────────────────────────────────────────────────
//  PROBLEM (before fix):
//    ESP32Servo's attach() configures the LEDC timer and immediately begins
//    outputting PWM. On many ESP32Servo builds the first PWM position is
//    the servo mid-point (90°) or the minimum pulse (0°) — whichever the
//    LEDC timer defaults to.  The _servo.write(P.servoHomeAngle) call that
//    follows is applied AFTER that initial pulse has already fired, so the
//    arm physically sweeps from 90° (or 0°) to homeAngle on every reset.
//    Depending on where the gate sits mechanically this can:
//      • knock a pre-loaded cube out of the pocket at start-of-run
//      • stress the servo horn / gate linkage
//      • confuse the referee (arm visibly moves at the start whistle)
//
//  FIX — attach-write-detach-delay-reattach sequence:
//    1. write(homeAngle) BEFORE attach() — pre-loads the target angle into
//       the ESP32Servo object so the first PWM pulse is at the correct
//       position (ESP32Servo stores the angle and uses it on attach).
//    2. attach() — starts PWM; first pulse is already homeAngle.
//    3. delay(600) — give servo time to physically reach home if it was
//       not already there (600 ms is enough for a full 180° SG90 sweep).
//    4. detach() — remove PWM.  Servo holds last position mechanically.
//    5. re-attach() — ready for normal operation.  Because the servo is
//       already at homeAngle the re-attach causes zero physical movement.
//
//  Result: arm moves at most ONCE at boot (to home), then stays perfectly
//  still until servo_open() / servo_close() is explicitly called.
// =========================================================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include "servo_gate.h"
#include "config.h"
#include "params.h"

static Servo _servo;

void servo_init() {
    // Allocate LEDC timers 2 & 3 for ESP32Servo
    // (motors_init already owns timers 0 & 1 via LEDC channels 0 & 1)
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    _servo.setPeriodHertz(50);                    // standard 50 Hz servo PWM

    // ── BUG-5 FIX: safe attach sequence ────────────────────────────────
    // Step 1: write home angle BEFORE attach so first PWM pulse = home
    _servo.write(P.servoHomeAngle);

    // Step 2: attach — PWM starts; first pulse is at servoHomeAngle
    _servo.attach(PIN_SERVO, 500, 2400);          // pulse width µs range

    // Step 3: wait for arm to reach home from wherever it was physically
    delay(600);

    // Step 4 & 5: detach then re-attach — removes the brief 90° default
    //   glitch that some ESP32Servo versions produce on the very first attach.
    //   After re-attach the servo is already at home so no movement occurs.
    _servo.detach();
    _servo.attach(PIN_SERVO, 500, 2400);
    _servo.write(P.servoHomeAngle);               // confirm position after re-attach

    Serial.printf("[SERVO] Init OK. Home=%d\u00b0  Pick=%d\u00b0\n",
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
    _servo.write(P.servoHomeAngle);
    Serial.printf("[SERVO] Closed (%d\u00b0)\n", P.servoHomeAngle);
}

void servo_open() {
    _servo.write(P.servoPickAngle);
    Serial.printf("[SERVO] Opened (%d\u00b0)\n", P.servoPickAngle);
}
