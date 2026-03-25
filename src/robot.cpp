// =========================================================================
//  robot.cpp  –  PID line follow + lost-line recovery + obstacle + pick
// =========================================================================
#include <Arduino.h>
#include "robot.h"
#include "params.h"
#include "motors.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "servo_gate.h"

// ── Top-level robot state ─────────────────────────────────────────────────────────
RobotState robotState = ST_LINE_FOLLOW;

// ── PID state ───────────────────────────────────────────────────────────────────
static float pidPos      = 0.0f;
static float filteredPos = 0.0f;
static float lastError   = 0.0f;
static float lineWidth   = 0.0f;

// ── Lost-line recovery state ──────────────────────────────────────────────────────
enum RecoveryMode {
    REC_IDLE, REC_REVERSE, REC_FORWARD_CHECK, REC_TURN_LEFT, REC_TURN_RIGHT
};
static RecoveryMode  recMode    = REC_IDLE;
static unsigned long recStartMs = 0;

// ───────────────────────────────────────────────────────────────────────────
void robot_resetRecovery() {
    recMode    = REC_IDLE;
    recStartMs = 0;
}

// ───────────────────────────────────────────────────────────────────────────
//  PID LINE FOLLOWING
//
//  FIX C3 — constrain wheel outputs to [minSpeed, 255] not [0, 255]
//  ─────────────────────────────────────────────────────────────────────
//  PROBLEM (before fix):
//    constrain(dynBase ± correction, 0, 255)
//    On a sharp curve, correction can exceed dynBase so the inner wheel
//    output goes to 0 (full stall). The robot then pivots on one wheel
//    instead of curving, which:
//      • Looks like the robot slows or jerks at every corner
//      • Causes the chassis to skid sideways, overshooting the line
//      • Triggers a false lost-line event, dropping speed to forwardRecoverSpeed
//
//  FIX:
//    constrain(dynBase ± correction, P.minSpeed, 255)
//    The inner wheel is guaranteed at least P.minSpeed (40 PWM ≈ 16%)
//    so both wheels always roll. The robot curves smoothly through
//    corners at maintained forward momentum instead of pivot-jerking.
//
//  error = filteredPos  (positive = line is to the right of centre)
//  correction added to left wheel, subtracted from right wheel
//  → positive correction steers robot rightward to follow line
// ─────────────────────────────────────────────────────────────────────
void robot_followLine() {
    pidPos      = sensors_computePosition();
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * pidPos;
    lineWidth   = sensors_computeWidth();

    float error      = filteredPos;
    float derivative = error - lastError;

    // Width compensation: reduce gain on thick lines / intersections
    float wComp = 1.0f;
    if (lineWidth > 1.5f) {
        wComp = 1.0f / (1.0f + P.widthKp * 0.08f * (lineWidth - 1.5f));
        if (wComp < 0.45f) wComp = 0.45f;
    }

    float correction = (P.kp * error + P.kd * derivative) * wComp;
    lastError = error;

    // Curve speed reduction — tunable via SET SPEEDDROP
    int dynBase = P.baseSpeed - (int)(fabsf(error) * P.speedDrop);
    dynBase = constrain(dynBase, P.minSpeed, P.baseSpeed);

    // FIX C3: lower bound is P.minSpeed, not 0
    // Inner wheel guaranteed to keep rolling through corners
    int leftSpd  = constrain(dynBase + (int)correction, P.minSpeed, 255);
    int rightSpd = constrain(dynBase - (int)correction, P.minSpeed, 255);

    motors_setLeft(leftSpd,  true);
    motors_setRight(rightSpd, true);
}

// ───────────────────────────────────────────────────────────────────────────
//  LOST-LINE RECOVERY  (4-stage state machine)
// ───────────────────────────────────────────────────────────────────────────
static void _reverseStage() {
    int fastRev = constrain(P.reverseSpeed + P.reverseBiasDelta, 0, 255);
    int slowRev = constrain(P.reverseSpeed - P.reverseBiasDelta, 0, 255);

    if      (lastSeenSide == SIDE_RIGHT) motors_driveBackwardBiasRight(fastRev, slowRev);
    else if (lastSeenSide == SIDE_LEFT)  motors_driveBackwardBiasLeft(fastRev, slowRev);
    else                                 motors_driveBackward(P.reverseSpeed);

    sensors_read();
    if (sensors_anyWhite() || millis() - recStartMs > P.timeoutRight) {
        recMode    = REC_FORWARD_CHECK;
        recStartMs = millis();
    }
}

static void _forwardStage() {
    motors_driveStraight(P.forwardRecoverSpeed);
    sensors_read();

    if (sensors_isLeftTurnPattern())  { recMode = REC_TURN_LEFT;  recStartMs = millis(); return; }
    if (sensors_isRightTurnPattern()) { recMode = REC_TURN_RIGHT; recStartMs = millis(); return; }
    if (sensors_isPathFoundPattern()) { robot_resetRecovery(); return; }

    if (millis() - recStartMs > P.forwardRecoverTime) {
        if      (lastSeenSide == SIDE_LEFT)  recMode = REC_TURN_LEFT;
        else if (lastSeenSide == SIDE_RIGHT) recMode = REC_TURN_RIGHT;
        else                                 robot_resetRecovery();
        recStartMs = millis();
    }
}

static void _turnLeftStage() {
    motors_pivotLeft(P.searchSpeed);
    sensors_read();
    if (sensors_isPathFoundPattern()) { robot_resetRecovery(); return; }
    if (sensors_isLeftTurnPattern())  return;
    if (millis() - recStartMs > P.timeoutLeft) { recMode = REC_REVERSE; recStartMs = millis(); }
}

static void _turnRightStage() {
    motors_pivotRight(P.searchSpeed);
    sensors_read();
    if (sensors_isPathFoundPattern()) { robot_resetRecovery(); return; }
    if (sensors_isRightTurnPattern()) return;
    if (millis() - recStartMs > P.timeoutRight) { recMode = REC_REVERSE; recStartMs = millis(); }
}

void robot_runLostLineRecovery() {
    sensors_read();
    if (recMode == REC_IDLE) { recMode = REC_REVERSE; recStartMs = millis(); }
    switch (recMode) {
        case REC_REVERSE:       _reverseStage();    break;
        case REC_FORWARD_CHECK: _forwardStage();    break;
        case REC_TURN_LEFT:     _turnLeftStage();   break;
        case REC_TURN_RIGHT:    _turnRightStage();  break;
        default: recMode = REC_REVERSE; recStartMs = millis(); break;
    }
}

// ───────────────────────────────────────────────────────────────────────────
//  RED CUBE AVOIDANCE  (time-based, blocking)
// ───────────────────────────────────────────────────────────────────────────
void robot_executeRedAvoid() {
    Serial.println(F("[AVOID] ── RED AVOIDANCE START ──"));
    int spd = P.avoidSpeed;

    Serial.println(F("[AVOID] 1. Reverse"));
    motors_driveBackward(spd);
    delay(P.reverseAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 2. Pivot LEFT 90°"));
    { unsigned long t = millis();
      while (millis()-t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 3. Forward past obstacle"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 4. Pivot RIGHT 90°"));
    { unsigned long t = millis();
      while (millis()-t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 5. Forward to re-cross line"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 6. Pivot LEFT → find line"));
    { unsigned long t = millis();
      while (millis()-t < P.timeoutRight) {
          motors_pivotLeft(P.searchSpeed);
          sensors_read();
          if (sensors_isPathFoundPattern() || sensors_anyWhite()) break;
          delay(5);
      } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] Done → resume line follow"));
    robot_resetRecovery();
    lastSeenSide = SIDE_UNKNOWN;
}

// ───────────────────────────────────────────────────────────────────────────
//  GREEN CUBE PICK  (blocking)
// ───────────────────────────────────────────────────────────────────────────
void robot_executeGreenPick() {
    Serial.println(F("[PICK] ── GREEN PICK START ──"));

    servo_close();
    Serial.println(F("[PICK] Gate confirmed closed. Approaching cube..."));

    unsigned long deadline = millis() + 5000UL;
    while (millis() < deadline) {
        float d = ultrasonic_getDistance();
        Serial.printf("[PICK] dist=%.1f cm\n", d);
        if (d <= P.greenPickDist) break;
        motors_setLeft(P.pickApproachSpeed,  true);
        motors_setRight(P.pickApproachSpeed, true);
        delay(20);
    }

    motors_stop();
    delay(300);

    Serial.println(F("[PICK] Cube secured. Resuming line follow."));
    robot_resetRecovery();
}

bool robot_recoveryIdle() {
    return (recMode == REC_IDLE);
}
