// =========================================================================
//  robot.cpp  -  PID line follow + lost-line recovery + obstacle + pick
//
//  MD0370 sensor changes:
//    - sensors_computeWidth() removed (no analog strength data)
//    - width compensation block removed from robot_followLine()
//    - sensors_anyWhite() -> sensors_anyOn() (renamed in sensors.h)
//
//  Green pick fixes:
//    FIX-P1: servo_open() now called after reaching pick distance
//    FIX-P2: ultrasonic median filter + spike rejection + hysteresis count
//    FIX-P3: reverse after pick + PICK_COOLDOWN_MS to suppress re-detect
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

// Lost-line recovery state
enum RecoveryMode {
    REC_IDLE, REC_REVERSE, REC_FORWARD_CHECK, REC_TURN_LEFT, REC_TURN_RIGHT
};
static RecoveryMode  recMode    = REC_IDLE;
static unsigned long recStartMs = 0;

// FIX-P3: cooldown after pick to suppress immediate re-detection
static unsigned long _pickDoneMs   = 0;
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
//  PID LINE FOLLOWING
// =========================================================================
void robot_followLine() {
    pidPos      = sensors_computePosition();
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * pidPos;

    float error      = filteredPos;
    float derivative = error - lastError;
    float correction = P.kp * error + P.kd * derivative;
    lastError = error;

    int dynBase = P.baseSpeed - (int)(fabsf(error) * P.speedDrop);
    dynBase = constrain(dynBase, P.minSpeed, P.baseSpeed);

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
//  RED CUBE AVOIDANCE  (time-based, blocking)
// =========================================================================
void robot_executeRedAvoid() {
    Serial.println(F("[AVOID] RED AVOIDANCE START"));
    int spd = P.avoidSpeed;

    Serial.println(F("[AVOID] 1. Reverse"));
    motors_driveBackward(spd);
    delay(P.reverseAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 2. Pivot LEFT 90 deg"));
    { unsigned long t = millis();
      while (millis()-t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 3. Forward past obstacle"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 4. Pivot RIGHT 90 deg"));
    { unsigned long t = millis();
      while (millis()-t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 5. Forward to re-cross line"));
    motors_driveStraight(spd);
    delay(P.forwardAvoidTime);
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] 6. Pivot LEFT - find line"));
    { unsigned long t = millis();
      while (millis()-t < P.timeoutRight) {
          motors_pivotLeft(P.searchSpeed);
          sensors_read();
          if (sensors_isPathFoundPattern() || sensors_anyOn()) break;
          delay(5);
      } }
    motors_stop(); delay(150);

    Serial.println(F("[AVOID] Done - resume line follow"));
    robot_resetRecovery();
    lastSeenSide = SIDE_UNKNOWN;
}

// =========================================================================
//  ULTRASONIC MEDIAN HELPER
//  Takes N readings, sorts them, returns the middle value.
//  Readings above SPIKE_LIMIT are treated as invalid and clamped to
//  SPIKE_LIMIT so they sink to the top of the sorted array and the
//  median is unaffected.
// =========================================================================
static const float US_SPIKE_LIMIT = 40.0f;   // cm - anything above = bad echo
static const int   US_SAMPLES     = 5;

static float _usMedian() {
    float buf[US_SAMPLES];
    for (int i = 0; i < US_SAMPLES; i++) {
        float r = ultrasonic_getDistance();
        buf[i]  = (r > US_SPIKE_LIMIT) ? US_SPIKE_LIMIT : r;
        delay(4);   // ~4 ms between pings avoids echo overlap
    }
    // Bubble sort (tiny array - fine on ESP32)
    for (int i = 0; i < US_SAMPLES - 1; i++)
        for (int j = 0; j < US_SAMPLES - 1 - i; j++)
            if (buf[j] > buf[j+1]) { float t = buf[j]; buf[j] = buf[j+1]; buf[j+1] = t; }
    return buf[US_SAMPLES / 2];
}

// =========================================================================
//  GREEN CUBE PICK  (blocking)
//
//  FIX-P1: servo_open() called once cube is secured (gripper release
//          was completely missing before - gate stayed closed forever)
//
//  FIX-P2: ultrasonic median filter (_usMedian) replaces raw single
//          readings. Also requires CLOSE_COUNT consecutive readings
//          inside greenPickDist before stopping, so one bad echo can't
//          trigger a premature stop.
//
//  FIX-P3: after securing the cube:
//          a) reverse away 400 ms so sensor clears the cube body
//          b) set _pickDoneMs = millis() to start cooldown window
//          c) main.cpp checks robot_pickCooldownActive() and skips
//             the obstacle branch during cooldown
//
//  Servo angles (from test_servo_diag):
//    1 deg   = gripper OPEN  (cube drops into pocket / released)
//    110 deg = gripper CLOSED (cube gripped / held in pocket)
// =========================================================================
static const int CLOSE_COUNT = 3;  // consecutive good readings required

void robot_executeGreenPick() {
    Serial.println(F("[PICK] GREEN PICK START"));

    // FIX-P1: Close gripper FIRST to be ready to capture the cube
    // 110 deg = closed/grip position (from test_servo_diag)
    servo_close();   // goes to P.servoHomeAngle = 110 deg
    Serial.println(F("[PICK] Gripper closed (110 deg). Approaching cube..."));

    unsigned long deadline = millis() + 5000UL;
    int closeStreak = 0;

    while (millis() < deadline) {
        // FIX-P2: use median of 5 readings instead of single raw reading
        float d = _usMedian();
        Serial.printf("[PICK] dist=%.1f cm (median)\n", d);

        if (d <= P.greenPickDist) {
            closeStreak++;
            Serial.printf("[PICK] In range %d/%d\n", closeStreak, CLOSE_COUNT);
            if (closeStreak >= CLOSE_COUNT) break;   // confirmed, not a spike
        } else {
            closeStreak = 0;   // reset if reading jumps back up
            motors_setLeft(P.pickApproachSpeed,  true);
            motors_setRight(P.pickApproachSpeed, true);
        }
    }

    motors_stop();
    delay(200);

    // FIX-P1: open gripper to release / drop cube into pocket
    // 1 deg = open position (from test_servo_diag)
    Serial.println(F("[PICK] Cube in range - opening gripper (1 deg)."));
    servo_open();    // goes to P.servoPickAngle = 1 deg
    delay(600);      // allow arm to fully sweep to 1 deg

    // Re-close to hold cube in pocket during transport
    Serial.println(F("[PICK] Re-closing to hold cube for transport (110 deg)."));
    servo_close();   // back to 110 deg to grip cube
    delay(400);

    // FIX-P3a: reverse to clear the cube body from the ultrasonic cone
    Serial.println(F("[PICK] Reversing to clear sensor..."));
    motors_driveBackward(P.reverseSpeed);
    delay(400);
    motors_stop();
    delay(150);

    // FIX-P3b: start cooldown clock - main.cpp ignores obstacle for 2.5 s
    _pickDoneMs = millis();

    Serial.println(F("[PICK] Cube secured. Resuming line follow."));
    robot_resetRecovery();
}

bool robot_recoveryIdle() {
    return (recMode == REC_IDLE);
}
