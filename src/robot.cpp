// =========================================================================
//  robot.cpp  -  PID line follow + lost-line recovery + obstacle + pick
//
//  Physic 3 - Curvature-Adaptive Speed
//  Physic 4 - Obstacle Side Memory
//  Debug    - PidDebugSnapshot written every followLine() call
//             robot_debugLog() prints HUMAN + CSV + writes LittleFS row
// =========================================================================
#include <Arduino.h>
#include "robot.h"
#include "params.h"
#include "motors.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "servo_gate.h"
#include "config.h"
#include "logger.h"

// Top-level robot state
RobotState robotState = ST_LINE_FOLLOW;

// PID snapshot (exported)
PidDebugSnapshot pidSnap;

// PID state
static float pidPos      = 0.0f;
static float filteredPos = 0.0f;
static float lastError   = 0.0f;

// Physic 3
static int  _curveAboveCount = 0;
static int  _curveBelowCount = 0;
static bool _inCurveMode     = false;
static const int CURVE_EXIT_LOOPS = 5;

// Recovery
enum RecoveryMode {
    REC_IDLE, REC_REVERSE, REC_FORWARD_CHECK, REC_TURN_LEFT, REC_TURN_RIGHT
};
static RecoveryMode  recMode    = REC_IDLE;
static unsigned long recStartMs = 0;

// Cooldown
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
//  PID LINE FOLLOWING  +  Physic 3  +  Debug Snapshot
// =========================================================================
void robot_followLine() {
    pidPos      = sensors_computePosition();
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * pidPos;

    float error      = filteredPos;
    float derivative = error - lastError;
    float correction = P.kp * error + P.kd * derivative;
    lastError = error;

    // Physic 3
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

    int effectiveBase = _inCurveMode ? P.curveSlowSpeed : P.baseSpeed;
    int dynBase       = effectiveBase - (int)(fabsf(error) * P.speedDrop);
    dynBase           = constrain(dynBase, P.minSpeed, effectiveBase);

    int leftSpd  = constrain(dynBase + (int)correction, P.minSpeed, 255);
    int rightSpd = constrain(dynBase - (int)correction, P.minSpeed, 255);

    motors_setLeft(leftSpd,   true);
    motors_setRight(rightSpd, true);

    // Write snapshot (always cheap)
    pidSnap.rawPos        = pidPos;
    pidSnap.filteredPos   = filteredPos;
    pidSnap.error         = error;
    pidSnap.derivative    = derivative;
    pidSnap.correction    = correction;
    pidSnap.dynBase       = dynBase;
    pidSnap.effectiveBase = effectiveBase;
    pidSnap.leftSpd       = leftSpd;
    pidSnap.rightSpd      = rightSpd;
    pidSnap.inCurveMode   = _inCurveMode;
    pidSnap.curveAbove    = _curveAboveCount;
    pidSnap.loopMs        = millis();
}

// =========================================================================
//  DEBUG LOG
//  Called from main.cpp every DBG_VERBOSE_INTERVAL ms.
//  - Prints HUMAN block + HINT block + CSV line to Serial (if DBG_VERBOSE)
//  - Appends CSV row to LittleFS /pidlog.csv (if DBG_LOG_TO_FILE)
// =========================================================================
void robot_debugLog() {
#if DBG_VERBOSE
    static bool _headerPrinted = false;
    if (!_headerPrinted) {
        Serial.println(F("\n[DBG] PID verbose log enabled"));
        Serial.println(F("[DBG] CSV columns:"));
        Serial.println(F("t_ms,rawPos,filtPos,error,derivative,correction,"
                         "dynBase,effectiveBase,leftSpd,rightSpd,curveMode,"
                         "curveAboveCount,dist_cm"));
        _headerPrinted = true;
    }

    // Sensor bitmap
    Serial.print(F("[IR]  "));
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        Serial.print(irOn[i] ? 'X' : '.');
    Serial.printf("  raw=%+.2f  filt=%+.2f  side=%s\n",
                  pidSnap.rawPos, pidSnap.filteredPos,
                  lastSeenSide == SIDE_LEFT   ? "L" :
                  lastSeenSide == SIDE_RIGHT  ? "R" :
                  lastSeenSide == SIDE_CENTER ? "C" : "?");

    // PID internals
    Serial.printf("[PID] err=%+.2f  deriv=%+.2f  corr=%+.1f\n",
                  pidSnap.error, pidSnap.derivative, pidSnap.correction);
    Serial.printf("      dyn=%d  base=%d(%s)  L=%d  R=%d  CURVE=%s(%d)\n",
                  pidSnap.dynBase, pidSnap.effectiveBase,
                  pidSnap.inCurveMode ? "SLOW" : "NORM",
                  pidSnap.leftSpd, pidSnap.rightSpd,
                  pidSnap.inCurveMode ? "ON " : "OFF",
                  pidSnap.curveAbove);
    Serial.printf("      dist=%.1fcm  t=%lums\n",
                  pidSnap.dist, pidSnap.loopMs);

    // Auto tuning hints
    if (fabsf(pidSnap.correction) > 0.8f * pidSnap.effectiveBase)
        Serial.println(F("[HINT] correction > 80% of base -> KP may be too high"));
    if (pidSnap.dynBase <= P.minSpeed && fabsf(pidSnap.error) < 1.0f)
        Serial.println(F("[HINT] dynBase clamped to minSpeed on low error -> raise minSpeed or lower speedDrop"));
    if (pidSnap.inCurveMode && pidSnap.curveAbove > 20)
        Serial.println(F("[HINT] CURVE mode held >20 loops -> curveDetectThresh may be too low"));
    if (fabsf(pidSnap.derivative) > fabsf(pidSnap.error) * 2.0f)
        Serial.println(F("[HINT] derivative >> error -> sensor noise or KD too high"));

    // CSV line to Serial
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%d,%d,%.1f\n",
                  pidSnap.loopMs, pidSnap.rawPos, pidSnap.filteredPos,
                  pidSnap.error, pidSnap.derivative, pidSnap.correction,
                  pidSnap.dynBase, pidSnap.effectiveBase,
                  pidSnap.leftSpd, pidSnap.rightSpd,
                  (int)pidSnap.inCurveMode, pidSnap.curveAbove, pidSnap.dist);
#endif

    // File log (independent of DBG_VERBOSE - works even if verbose is off)
#if DBG_LOG_TO_FILE
    logger_appendRow(pidSnap);
#endif
}

// =========================================================================
//  LOST-LINE RECOVERY
// =========================================================================
static void _reverseStage() {
    int fastRev = constrain(P.reverseSpeed + P.reverseBiasDelta, 0, 255);
    int slowRev = constrain(P.reverseSpeed - P.reverseBiasDelta, 0, 255);
    if      (lastSeenSide == SIDE_RIGHT) motors_driveBackwardBiasRight(fastRev, slowRev);
    else if (lastSeenSide == SIDE_LEFT)  motors_driveBackwardBiasLeft(fastRev, slowRev);
    else                                 motors_driveBackward(P.reverseSpeed);
    sensors_read();
    if (sensors_anyOn() || millis() - recStartMs > P.timeoutRight) {
        recMode = REC_FORWARD_CHECK; recStartMs = millis();
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
        case REC_REVERSE:       _reverseStage();   break;
        case REC_FORWARD_CHECK: _forwardStage();   break;
        case REC_TURN_LEFT:     _turnLeftStage();  break;
        case REC_TURN_RIGHT:    _turnRightStage(); break;
        default: recMode = REC_REVERSE; recStartMs = millis(); break;
    }
}

// =========================================================================
//  RED CUBE AVOIDANCE
// =========================================================================
void robot_executeRedAvoid() {
    Serial.println(F("[AVOID] RED AVOIDANCE START"));
    int spd = P.avoidSpeed;
    bool goRight = false;
    if (P.avoidPreferRight) {
        if      (lastSeenSide == SIDE_LEFT)  goRight = true;
        else if (lastSeenSide == SIDE_RIGHT) goRight = false;
        else                                 goRight = false;
        Serial.printf("[AVOID] Side memory: %s -> avoid %s\n",
                      lastSeenSide == SIDE_LEFT ? "LEFT" : lastSeenSide == SIDE_RIGHT ? "RIGHT" : "UNKNOWN",
                      goRight ? "RIGHT" : "LEFT");
    } else {
        Serial.println(F("[AVOID] Fixed LEFT avoidance"));
    }
    Serial.println(F("[AVOID] 1. Reverse"));
    motors_driveBackward(spd); delay(P.reverseAvoidTime); motors_stop(); delay(150);
    if (goRight) {
        Serial.println(F("[AVOID] 2. Pivot RIGHT 90"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); }
    } else {
        Serial.println(F("[AVOID] 2. Pivot LEFT 90"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); }
    }
    motors_stop(); delay(150);
    Serial.println(F("[AVOID] 3. Forward past obstacle"));
    motors_driveStraight(spd); delay(P.forwardAvoidTime); motors_stop(); delay(150);
    if (goRight) {
        Serial.println(F("[AVOID] 4. Pivot LEFT 90"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotLeft(spd); delay(5); }
    } else {
        Serial.println(F("[AVOID] 4. Pivot RIGHT 90"));
        unsigned long t = millis();
        while (millis() - t < P.turn90AvoidTime) { motors_pivotRight(spd); delay(5); }
    }
    motors_stop(); delay(150);
    Serial.println(F("[AVOID] 5. Forward to re-cross line"));
    motors_driveStraight(spd); delay(P.forwardAvoidTime); motors_stop(); delay(150);
    if (goRight) {
        Serial.println(F("[AVOID] 6. Pivot RIGHT - find line"));
        unsigned long t = millis();
        while (millis() - t < P.timeoutRight) {
            motors_pivotRight(P.searchSpeed); sensors_read();
            if (sensors_isPathFoundPattern() || sensors_anyOn()) break;
            delay(5);
        }
    } else {
        Serial.println(F("[AVOID] 6. Pivot LEFT - find line"));
        unsigned long t = millis();
        while (millis() - t < P.timeoutRight) {
            motors_pivotLeft(P.searchSpeed); sensors_read();
            if (sensors_isPathFoundPattern() || sensors_anyOn()) break;
            delay(5);
        }
    }
    motors_stop(); delay(150);
    Serial.printf("[AVOID] Done (%s) - resume\n", goRight ? "RIGHT" : "LEFT");
    robot_resetRecovery();
    lastSeenSide = SIDE_UNKNOWN;
}

// =========================================================================
//  ULTRASONIC MEDIAN
// =========================================================================
static const float US_SPIKE_LIMIT = 40.0f;
static const int   US_SAMPLES     = 5;

static float _usMedian() {
    float buf[US_SAMPLES];
    for (int i = 0; i < US_SAMPLES; i++) {
        float r = ultrasonic_getDistance();
        buf[i] = (r > US_SPIKE_LIMIT) ? US_SPIKE_LIMIT : r;
        delay(4);
    }
    for (int i = 0; i < US_SAMPLES-1; i++)
        for (int j = 0; j < US_SAMPLES-1-i; j++)
            if (buf[j] > buf[j+1]) { float t=buf[j]; buf[j]=buf[j+1]; buf[j+1]=t; }
    return buf[US_SAMPLES/2];
}

// =========================================================================
//  GREEN CUBE PICK
// =========================================================================
static const int CLOSE_COUNT = 3;

void robot_executeGreenPick() {
    Serial.println(F("[PICK] GREEN PICK START"));
    servo_close();
    Serial.println(F("[PICK] Gripper closed. Approaching..."));
    unsigned long deadline = millis() + 5000UL;
    int closeStreak = 0;
    while (millis() < deadline) {
        float d = _usMedian();
        Serial.printf("[PICK] dist=%.1f cm\n", d);
        if (d <= P.greenPickDist) {
            closeStreak++;
            if (closeStreak >= CLOSE_COUNT) break;
        } else {
            closeStreak = 0;
            motors_setLeft(P.pickApproachSpeed, true);
            motors_setRight(P.pickApproachSpeed, true);
        }
    }
    motors_stop(); delay(200);
    Serial.println(F("[PICK] Opening gripper."));
    servo_open(); delay(600);
    Serial.println(F("[PICK] Re-closing to hold."));
    servo_close(); delay(400);
    Serial.println(F("[PICK] Reversing to clear sensor."));
    motors_driveBackward(P.reverseSpeed); delay(400);
    motors_stop(); delay(150);
    _pickDoneMs = millis();
    Serial.println(F("[PICK] Cube secured."));
    robot_resetRecovery();
}

bool robot_recoveryIdle() { return (recMode == REC_IDLE); }
