// =========================================================================
//  robot.cpp  -  PID line follow + lost-line recovery + obstacle + pick
//
//  Physic 3 - Curvature-Adaptive Speed:
//    robot_followLine() maintains a consecutive-loop counter.
//    When |filteredPos| > P.curveDetectThresh for >= P.curveConfirmLoops
//    loops the dynBase is capped at P.curveSlowSpeed instead of baseSpeed.
//    A separate exit hysteresis (CURVE_EXIT_LOOPS) prevents chattering.
//
//  Physic 4 - Obstacle Side Memory:
//    robot_executeRedAvoid() reads P.avoidPreferRight.
//    0 = always avoid left (original).
//    1 = auto-select: SIDE_LEFT -> pivot RIGHT, SIDE_RIGHT -> pivot LEFT.
//    All 6 avoidance steps are mirrored for the right-side case.
//
//  Previous fixes retained:
//    FIX-P1: servo open/close sequence in executeGreenPick
//    FIX-P2: ultrasonic median filter + hysteresis count
//    FIX-P3: reverse after pick + 2.5s cooldown
// =========================================================================
#include <Arduino.h>
#include "robot.h"
#include "params.h"
#include "motors.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "servo_gate.h"

// Top-level robot state
RobotState robotState = ST_LINE_FOLLOW;

// PID state
static float pidPos      = 0.0f;
static float filteredPos = 0.0f;
static float lastError   = 0.0f;

// Physic 3: curve mode state
static int  _curveAboveCount = 0;  // consecutive loops above threshold
static int  _curveBelowCount = 0;  // consecutive loops below threshold (exit)
static bool _inCurveMode     = false;
static const int CURVE_EXIT_LOOPS = 5;  // loops below thresh before exiting

// Lost-line recovery state
enum RecoveryMode {
    REC_IDLE, REC_REVERSE, REC_FORWARD_CHECK, REC_TURN_LEFT, REC_TURN_RIGHT
};
static RecoveryMode  recMode    = REC_IDLE;
static unsigned long recStartMs = 0;

// FIX-P3: cooldown after pick
static unsigned long _pickDoneMs = 0;
static const unsigned long PICK_COOLDOWN_MS = 2500UL;

bool robot_pickCooldownActive() {
    if (_pickDoneMs == 0) return false;
    return (millis() - _pickDoneMs < PICK_COOLDOWN_MS);
}

void robot_resetRecovery() {
    recMode    = REC_IDLE;
    recStartMs = 0;
}

// =========================================================================
//  PID LINE FOLLOWING  +  Physic 3: Curvature-Adaptive Speed
// =========================================================================
void robot_followLine() {
    pidPos      = sensors_computePosition();
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * pidPos;

    float error      = filteredPos;
    float derivative = error - lastError;
    float correction = P.kp * error + P.kd * derivative;
    lastError = error;

    // ── Physic 3: update curve mode ──────────────────────────────────────
    //  Entry: |error| above threshold for N consecutive loops
    //  Exit : |error| below threshold for CURVE_EXIT_LOOPS loops (hysteresis)
    if (fabsf(error) > P.curveDetectThresh) {
        _curveAboveCount++;
        _curveBelowCount = 0;
        if (_curveAboveCount >= P.curveConfirmLoops && !_inCurveMode) {
            _inCurveMode = true;
            Serial.printf("[CURVE] Entered curve mode (|err|=%.2f thresh=%.2f)\n",
                          fabsf(error), P.curveDetectThresh);
        }
    } else {
        _curveAboveCount = 0;
        if (_inCurveMode) {
            _curveBelowCount++;
            if (_curveBelowCount >= CURVE_EXIT_LOOPS) {
                _inCurveMode     = false;
                _curveBelowCount = 0;
                Serial.println(F("[CURVE] Exited curve mode"));
            }
        }
    }

    // Select effective base speed
    int effectiveBase = _inCurveMode ? P.curveSlowSpeed : P.baseSpeed;

    // Apply speedDrop on top of effective base
    int dynBase = effectiveBase - (int)(fabsf(error) * P.speedDrop);
    dynBase = constrain(dynBase, P.minSpeed, effectiveBase);

    int leftSpd  = constrain(dynBase + (int)correction, P.minSpeed, 255);
    int rightSpd = constrain(dynBase - (int)correction, P.minSpeed, 255);

    motors_setLeft(leftSpd,   true);
    motors_setRight(rightSpd, true);
}

// =========================================================================
//  LOST-LINE RECOVERY  (4-stage state machine)
// =========================================================================
static void _reverseStage() {
    int fastRev = constrain(P.reverseSpeed + P.reverseBiasDelta, 0, 255);
    int slowRev = constrain(P.reverseSpeed - P.reverseBiasDelta, 0, 255);

    if      (lastSeenSide == SIDE_RIGHT) motors_driveBackwardBiasRight(fastRev, slowRev);
    else if (lastSeenSide == SIDE_LEFT)  motors_driveBackwardBiasLeft(fastRev, slowRev);
    else                                 motors_driveBackward(P.reverseSpeed);

    sensors_read();
    if (sensors_anyOn() || millis() - recStartMs > P.timeoutRight) {
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

// =========================================================================
//  RED CUBE AVOIDANCE  +  Physic 4: Obstacle Side Memory
//
//  avoidPreferRight = 0 -> always avoid LEFT (6-step original sequence)
//  avoidPreferRight = 1 -> auto-select direction from lastSeenSide:
//      SIDE_LEFT  -> robot was curving left  -> obstacle on left  -> go RIGHT
//      SIDE_RIGHT -> robot was curving right -> obstacle on right -> go LEFT
//      SIDE_UNKNOWN -> default LEFT
//
//  All 6 steps are fully mirrored for the RIGHT-side avoidance path.
// =========================================================================
void robot_executeRedAvoid() {
    Serial.println(F("[AVOID] RED AVOIDANCE START"));
    int spd = P.avoidSpeed;

    // ── Physic 4: choose direction ───────────────────────────────────────
    bool goRight = false;
    if (P.avoidPreferRight) {
        if      (lastSeenSide == SIDE_LEFT)  goRight = true;   // obstacle on left
        else if (lastSeenSide == SIDE_RIGHT) goRight = false;  // obstacle on right
        else                                 goRight = false;  // unknown -> left
        Serial.printf("[AVOID] Side memory: lastSeenSide=%s -> avoid %s\n",
                      lastSeenSide == SIDE_LEFT  ? "LEFT"  :
                      lastSeenSide == SIDE_RIGHT ? "RIGHT" : "UNKNOWN",
                      goRight ? "RIGHT" : "LEFT");
    } else {
        Serial.println(F("[AVOID] Fixed LEFT avoidance (avoidPreferRight=0)"));
    }

    // ── Step 1: Reverse ──────────────────────────────────────────────────
    Serial.println(F("[AVOID] 1. Reverse"));
    motors_driveBackward(spd);
    delay(P.reverseAvoidTime);
    motors_stop(); delay(150);

    // ── Step 2: First 90 deg pivot ───────────────────────────────────────
    if (goRight) {
        Serial.println(F("[AVOID] 2. Pivot RIGHT 90 deg"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); }
    } else {
        Serial.println(F("[AVOID] 2. Pivot LEFT 90 deg"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); }
    }
    motors_stop(); delay(150);

    // ── Step 3: Forward past obstacle ────────────────────────────────────
    Serial.println(F("[AVOID] 3. Forward past obstacle"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    // ── Step 4: Return 90 deg pivot (opposite direction) ─────────────────
    if (goRight) {
        Serial.println(F("[AVOID] 4. Pivot LEFT 90 deg"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); }
    } else {
        Serial.println(F("[AVOID] 4. Pivot RIGHT 90 deg"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); }
    }
    motors_stop(); delay(150);

    // ── Step 5: Forward to re-cross line ─────────────────────────────────
    Serial.println(F("[AVOID] 5. Forward to re-cross line"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    // ── Step 6: Final pivot to find line ─────────────────────────────────
    if (goRight) {
        Serial.println(F("[AVOID] 6. Pivot RIGHT - find line"));
        unsigned long t = millis();
        while (millis() - t < P.timeoutRight) {
            motors_pivotRight(P.searchSpeed);
            sensors_read();
            if (sensors_isPathFoundPattern() || sensors_anyOn()) break;
            delay(5);
        }
    } else {
        Serial.println(F("[AVOID] 6. Pivot LEFT - find line"));
        unsigned long t = millis();
        while (millis() - t < P.timeoutRight) {
            motors_pivotLeft(P.searchSpeed);
            sensors_read();
            if (sensors_isPathFoundPattern() || sensors_anyOn()) break;
            delay(5);
        }
    }
    motors_stop(); delay(150);

    Serial.printf("[AVOID] Done (%s side) - resume line follow\n",
                  goRight ? "RIGHT" : "LEFT");
    robot_resetRecovery();
    lastSeenSide = SIDE_UNKNOWN;
}

// =========================================================================
//  ULTRASONIC MEDIAN HELPER
// =========================================================================
static const float US_SPIKE_LIMIT = 40.0f;
static const int   US_SAMPLES     = 5;

static float _usMedian() {
    float buf[US_SAMPLES];
    for (int i = 0; i < US_SAMPLES; i++) {
        float r = ultrasonic_getDistance();
        buf[i]  = (r > US_SPIKE_LIMIT) ? US_SPIKE_LIMIT : r;
        delay(4);
    }
    for (int i = 0; i < US_SAMPLES - 1; i++)
        for (int j = 0; j < US_SAMPLES - 1 - i; j++)
            if (buf[j] > buf[j+1]) { float t = buf[j]; buf[j] = buf[j+1]; buf[j+1] = t; }
    return buf[US_SAMPLES / 2];
}

// =========================================================================
//  GREEN CUBE PICK  (blocking)
// =========================================================================
static const int CLOSE_COUNT = 3;

void robot_executeGreenPick() {
    Serial.println(F("[PICK] GREEN PICK START"));
    servo_close();
    Serial.println(F("[PICK] Gripper closed (110 deg). Approaching cube..."));

    unsigned long deadline  = millis() + 5000UL;
    int           closeStreak = 0;

    while (millis() < deadline) {
        float d = _usMedian();
        Serial.printf("[PICK] dist=%.1f cm (median)\n", d);

        if (d <= P.greenPickDist) {
            closeStreak++;
            Serial.printf("[PICK] In range %d/%d\n", closeStreak, CLOSE_COUNT);
            if (closeStreak >= CLOSE_COUNT) break;
        } else {
            closeStreak = 0;
            motors_setLeft(P.pickApproachSpeed,  true);
            motors_setRight(P.pickApproachSpeed, true);
        }
    }

    motors_stop();
    delay(200);

    Serial.println(F("[PICK] Cube in range - opening gripper (1 deg)."));
    servo_open();
    delay(600);

    Serial.println(F("[PICK] Re-closing to hold cube for transport (110 deg)."));
    servo_close();
    delay(400);

    Serial.println(F("[PICK] Reversing to clear sensor..."));
    motors_driveBackward(P.reverseSpeed);
    delay(400);
    motors_stop();
    delay(150);

    _pickDoneMs = millis();
    Serial.println(F("[PICK] Cube secured. Resuming line follow."));
    robot_resetRecovery();
}

bool robot_recoveryIdle() {
    return (recMode == REC_IDLE);
}
