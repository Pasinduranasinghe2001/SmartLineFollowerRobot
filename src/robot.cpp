// =========================================================================
//  robot.cpp  -  PID line follow + lost-line recovery + obstacle + pick
//
//  DUAL-ZONE SENSOR STRATEGY (feature/dual-zone-sensor-strategy)
//  ---------------------------------------------------------------
//  DETECT zone (inner S2/S3/S4): computes PD error in range ±2
//  HOLD zone   (outer S0/S6)   : triggers speed state transitions
//
//  Speed State Machine:
//    FAST  (fastSpeed=165)   : no outer sensor, |innerErr|<0.5 -> straight
//    CURVE (turnSpeed=128)   : one outer sensor active
//    SHARP (sharpSpeed=105)  : both outer sensors active
//
//  Kp=90.0, Kd=25.0 scaled for inner ±2 error range (was ±6)
//
//  Physic 3 (curveAboveCount) kept as secondary fallback.
//  Physic 4 - Obstacle Side Memory unchanged.
//  Debug    - PidDebugSnapshot written every followLine() call.
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
static float pidPos      = 0.0f;   // full 7-sensor position (for debug)
static float innerPos    = 0.0f;   // inner-zone position (for PD)
static float filteredPos = 0.0f;   // filtered inner position
static float lastError   = 0.0f;

// Physic 3 (secondary fallback)
static int  _curveAboveCount = 0;
static int  _curveBelowCount = 0;
static bool _inCurveMode     = false;
static const int CURVE_EXIT_LOOPS = 5;

// Dual-zone speed states
enum ZoneSpeedState { ZSS_FAST, ZSS_CURVE, ZSS_SHARP };
static ZoneSpeedState _zoneState = ZSS_FAST;

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
//  PID LINE FOLLOWING  -  Dual-Zone Speed State Machine
// =========================================================================
void robot_followLine() {
    // --- Read both zone positions ---
    pidPos   = sensors_computePosition();       // full 7-sensor (for debug)
    innerPos = sensors_computeInnerPosition();  // inner S2/S3/S4 (for PD)

    // Low-pass filter applied to inner position
    filteredPos = P.posFilter * filteredPos + (1.0f - P.posFilter) * innerPos;

    float error      = filteredPos;
    float derivative = error - lastError;

    // PD gains are scaled for ±2 inner error range
    // Effective Kp=90, Kd=25 (set in params via kp/kd fields)
    float correction = P.kp * error + P.kd * derivative;
    lastError = error;

    // --- DUAL-ZONE speed state machine ---
    bool outerL = sensors_outerLeft();
    bool outerR = sensors_outerRight();

    int effectiveBase;
    if (outerL && outerR) {
        // Both outer sensors active -> extremely sharp curve
        _zoneState    = ZSS_SHARP;
        effectiveBase = P.sharpSpeed;
        if (!_inCurveMode) {
            _inCurveMode = true;
            Serial.println(F("[ZONE] SHARP: both outer sensors active"));
        }
    } else if (outerL || outerR) {
        // One outer sensor active -> curve
        _zoneState    = ZSS_CURVE;
        effectiveBase = P.turnSpeed;
        if (!_inCurveMode) {
            _inCurveMode = true;
            Serial.printf("[ZONE] CURVE: outer %s sensor active\n",
                          outerL ? "LEFT" : "RIGHT");
        }
    } else {
        // No outer sensors -> straight / gentle
        if (_inCurveMode) {
            _curveBelowCount++;
            if (_curveBelowCount >= CURVE_EXIT_LOOPS) {
                _inCurveMode     = false;
                _curveBelowCount = 0;
                Serial.println(F("[ZONE] FAST: outer sensors clear -> boosting speed"));
            }
            effectiveBase = P.turnSpeed;  // brief hold before boost
        } else {
            _zoneState    = ZSS_FAST;
            effectiveBase = P.fastSpeed;
        }
    }

    // --- Physic 3 fallback (curveAboveCount on full error) ---
    // Used as secondary guard if outer sensors miss a gradual curve
    if (fabsf(pidPos) > P.curveDetectThresh) {
        _curveAboveCount++;
        if (_curveAboveCount >= P.curveConfirmLoops && effectiveBase > P.turnSpeed) {
            effectiveBase = P.turnSpeed;
            Serial.printf("[P3] Physic3 fallback: |fullErr|=%.2f > thresh=%.2f\n",
                          fabsf(pidPos), P.curveDetectThresh);
        }
    } else {
        _curveAboveCount = 0;
    }

    // --- Dynamic base: reduce speed proportionally with inner error ---
    int dynBase = effectiveBase - (int)(fabsf(error) * P.speedDrop);
    dynBase     = constrain(dynBase, P.minSpeed, effectiveBase);

    int leftSpd  = constrain(dynBase + (int)correction, P.minSpeed, 255);
    int rightSpd = constrain(dynBase - (int)correction, P.minSpeed, 255);

    motors_setLeft(leftSpd,   true);
    motors_setRight(rightSpd, true);

    // --- Write debug snapshot ---
    pidSnap.rawPos        = pidPos;          // full 7-sensor for debug visibility
    pidSnap.filteredPos   = filteredPos;     // filtered inner pos
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
// =========================================================================
void robot_debugLog() {
#if DBG_VERBOSE
    static bool _headerPrinted = false;
    if (!_headerPrinted) {
        Serial.println(F("\n[DBG] PID verbose log enabled - DUAL-ZONE mode"));
        Serial.println(F("[DBG] CSV columns:"));
        Serial.println(F("t_ms,rawPos,filtPos,error,derivative,correction,"
                         "dynBase,effectiveBase,leftSpd,rightSpd,curveMode,"
                         "curveAboveCount,dist_cm"));
        _headerPrinted = true;
    }

    // Sensor bitmap with zone markers
    Serial.print(F("[IR]  "));
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (i == 1 || i == 5) Serial.print('|');  // zone boundaries
        Serial.print(irOn[i] ? 'X' : '.');
    }
    Serial.printf("  full=%+.2f  inner=%+.2f  side=%s  hold=%s%s\n",
                  pidSnap.rawPos, pidSnap.filteredPos,
                  lastSeenSide == SIDE_LEFT   ? "L" :
                  lastSeenSide == SIDE_RIGHT  ? "R" :
                  lastSeenSide == SIDE_CENTER ? "C" : "?",
                  irOn[0] ? "L" : ".",
                  irOn[6] ? "R" : ".");

    // Zone state
    const char* zoneStr = (_zoneState == ZSS_SHARP) ? "SHARP" :
                          (_zoneState == ZSS_CURVE)  ? "CURVE" : "FAST ";
    Serial.printf("[ZONE] %s  base=%d  dyn=%d  L=%d  R=%d\n",
                  zoneStr,
                  pidSnap.effectiveBase, pidSnap.dynBase,
                  pidSnap.leftSpd, pidSnap.rightSpd);

    // PID internals
    Serial.printf("[PID] err=%+.2f  deriv=%+.2f  corr=%+.1f\n",
                  pidSnap.error, pidSnap.derivative, pidSnap.correction);
    Serial.printf("      dist=%.1fcm  t=%lums\n",
                  pidSnap.dist, pidSnap.loopMs);

    // Auto tuning hints
    if (fabsf(pidSnap.correction) > 0.8f * pidSnap.effectiveBase)
        Serial.println(F("[HINT] correction > 80% base -> Kp may be too high"));
    if (pidSnap.dynBase <= P.minSpeed && fabsf(pidSnap.error) < 0.3f)
        Serial.println(F("[HINT] dynBase at floor on low error -> raise minSpeed or lower speedDrop"));
    if (_zoneState == ZSS_FAST && fabsf(pidSnap.error) > 1.0f)
        Serial.println(F("[HINT] FAST mode but inner error>1 -> outer sensor threshold may be too high"));
    if (fabsf(pidSnap.derivative) > fabsf(pidSnap.error) * 2.0f)
        Serial.println(F("[HINT] derivative >> error -> sensor noise or Kd too high"));

    // CSV line
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%d,%d,%.1f\n",
                  pidSnap.loopMs, pidSnap.rawPos, pidSnap.filteredPos,
                  pidSnap.error, pidSnap.derivative, pidSnap.correction,
                  pidSnap.dynBase, pidSnap.effectiveBase,
                  pidSnap.leftSpd, pidSnap.rightSpd,
                  (int)pidSnap.inCurveMode, pidSnap.curveAbove, pidSnap.dist);
#endif

#if DBG_LOG_TO_FILE
    logger_appendRow(pidSnap);
#endif
}

// =========================================================================
//  LOST-LINE RECOVERY
//  Directional search guided by lastSeenSide (outer sensor memory)
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
        // Guided by outer-sensor last-seen side (more reliable than pure encoder)
        if      (lastSeenSide == SIDE_LEFT)  recMode = REC_TURN_LEFT;
        else if (lastSeenSide == SIDE_RIGHT) recMode = REC_TURN_RIGHT;
        else                                 robot_resetRecovery();
        recStartMs = millis();
    }
}

static void _turnLeftStage() {
    motors_pivotLeft(P.searchSpeed);
    sensors_read();
    // Outer sensor on correct side is the earliest line-found signal
    if (sensors_outerRight() || sensors_isPathFoundPattern()) {
        robot_resetRecovery(); return;
    }
    if (sensors_isLeftTurnPattern()) return;
    if (millis() - recStartMs > P.timeoutLeft) { recMode = REC_REVERSE; recStartMs = millis(); }
}

static void _turnRightStage() {
    motors_pivotRight(P.searchSpeed);
    sensors_read();
    // Outer sensor on correct side is the earliest line-found signal
    if (sensors_outerLeft() || sensors_isPathFoundPattern()) {
        robot_resetRecovery(); return;
    }
    if (sensors_isRightTurnPattern()) return;
    if (millis() - recStartMs > P.timeoutRight) { recMode = REC_REVERSE; recStartMs = millis(); }
}

void robot_runLostLineRecovery() {
    sensors_read();
    sensors_updateLastSeenSide();  // keep outer-sensor memory fresh during recovery
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
            if (sensors_outerLeft() || sensors_isPathFoundPattern() || sensors_anyOn()) break;
            delay(5);
        }
    } else {
        Serial.println(F("[AVOID] 6. Pivot LEFT - find line"));
        unsigned long t = millis();
        while (millis() - t < P.timeoutRight) {
            motors_pivotLeft(P.searchSpeed); sensors_read();
            if (sensors_outerRight() || sensors_isPathFoundPattern() || sensors_anyOn()) break;
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
