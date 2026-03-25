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

// ── Top-level robot state ──────────────────────────────────────────────────────
RobotState robotState = ST_LINE_FOLLOW;

// ── PID state ─────────────────────────────────────────────────────────────────
static float pidPos      = 0.0f;
static float filteredPos = 0.0f;
static float lastError   = 0.0f;
static float lineWidth   = 0.0f;

// ── Lost-line recovery state ──────────────────────────────────────────────────
enum RecoveryMode {
    REC_IDLE, REC_REVERSE, REC_FORWARD_CHECK, REC_TURN_LEFT, REC_TURN_RIGHT
};
static RecoveryMode  recMode    = REC_IDLE;
static unsigned long recStartMs = 0;

// ─────────────────────────────────────────────────────────────────────────
//  RESET
// ─────────────────────────────────────────────────────────────────────────
void robot_resetRecovery() {
    recMode    = REC_IDLE;
    recStartMs = 0;
}

// ─────────────────────────────────────────────────────────────────────────
//  PID LINE FOLLOWING
//
//  error = filteredPos  (positive = line is to the right of centre)
//  correction added to left wheel, subtracted from right wheel
//  → positive correction steers robot rightward to follow line
// ─────────────────────────────────────────────────────────────────────────
void robot_followLine() {
    pidPos      = sensors_computePosition();
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * pidPos;
    lineWidth   = sensors_computeWidth();

    float error      = filteredPos;
    float derivative = error - lastError;

    // Width compensation: reduce gain on thick lines (intersections)
    float wComp = 1.0f;
    if (lineWidth > 1.5f) {
        wComp = 1.0f / (1.0f + P.widthKp * 0.08f * (lineWidth - 1.5f));
        if (wComp < 0.45f) wComp = 0.45f;
    }

    float correction = (P.kp * error + P.kd * derivative) * wComp;
    lastError = error;

    // Slow down in curves
    int dynBase = P.baseSpeed - (int)(fabsf(error) * 4.0f);
    dynBase = constrain(dynBase, P.minSpeed, P.baseSpeed);

    int leftSpd  = constrain(dynBase + (int)correction, 0, 255);
    int rightSpd = constrain(dynBase - (int)correction, 0, 255);

    motors_setLeft(leftSpd,  true);
    motors_setRight(rightSpd, true);
}

// ─────────────────────────────────────────────────────────────────────────
//  LOST-LINE RECOVERY  (4-stage state machine)
// ─────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────
//  RED CUBE AVOIDANCE  (time-based, blocking)
//
//  Maneuver:
//    1. Reverse  ~7 cm
//    2. Pivot LEFT 90°
//    3. Drive forward past obstacle (~15 cm)
//    4. Pivot RIGHT 90°  (parallel to original path)
//    5. Drive forward to re-cross line (~15 cm)
//    6. Pivot LEFT until line found
//    7. Resume PID
// ─────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────
//  GREEN CUBE PICK  (blocking)
//
//  BUG-02 FIX:
//  The ONLY servo_close() call is here, at the top of this function.
//  main.cpp no longer calls servo_close() before invoking this function.
//  Having two rapid servo_close() calls to the same angle caused a
//  brief torque spike (jerk) that could knock the cube out of the pocket
//  during the approach phase.
//
//  Gate ownership rule (single source of truth):
//    • robot_executeGreenPick()  → closes the gate (owns CLOSE)
//    • main.cpp end-zone drop    → opens then closes (owns OPEN + final CLOSE)
//    • setup()                   → closes gate once at boot (safe, servo not moving yet)
//
//  Steps:
//    1. servo_close()  – ensure gate is closed before cube enters pocket
//    2. Creep forward until ultrasonic ≤ greenPickDist
//    3. Stop – cube is now inside the pocket, held by the closed gate
//    4. Resume line following; drop at end zone handled by main.cpp
// ─────────────────────────────────────────────────────────────────────────
void robot_executeGreenPick() {
    Serial.println(F("[PICK] ── GREEN PICK START ──"));

    // ── Step 1: single authoritative gate close ────────────────────────────
    servo_close();
    Serial.println(F("[PICK] Gate closed. Approaching cube..."));

    // ── Step 2: creep forward until within pick distance ──────────────────
    unsigned long deadline = millis() + 5000UL;
    while (millis() < deadline) {
        float d = ultrasonic_getDistance();
        Serial.printf("[PICK] dist=%.1f cm\n", d);
        if (d <= P.greenPickDist) break;
        motors_setLeft(P.pickApproachSpeed,  true);
        motors_setRight(P.pickApproachSpeed, true);
        delay(20);
    }

    // ── Step 3: stop – cube is now inside the pocket ─────────────────────
    motors_stop();
    delay(300);

    Serial.println(F("[PICK] Cube secured. Resuming line follow."));
    robot_resetRecovery();
}

// ─────────────────────────────────────────────────────────────────────────
//  Expose recovery idle status to main.cpp
// ─────────────────────────────────────────────────────────────────────────
bool robot_recoveryIdle() {
    return (recMode == REC_IDLE);
}
