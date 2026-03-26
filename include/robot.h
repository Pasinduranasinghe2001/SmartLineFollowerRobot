// =========================================================================
//  robot.h  -  Top-level robot state machine API
// =========================================================================
#pragma once
#include <Arduino.h>

// ── Robot top-level states ────────────────────────────────────────────────
enum RobotState {
    ST_LINE_FOLLOW,
    ST_OBSTACLE_SLOW,
    ST_COLOR_DETECT,
    ST_RED_AVOID,
    ST_GREEN_PICK
};
extern RobotState robotState;

// ── PID debug snapshot ────────────────────────────────────────────────────
//  Written every robot_followLine() call regardless of DBG_VERBOSE.
//  robot_debugLog() reads this struct and prints it on the interval timer.
//  Having a struct means main.cpp never needs to know PID internals.
struct PidDebugSnapshot {
    float rawPos;        // sensors_computePosition() raw output
    float filteredPos;   // after posFilter blending
    float error;         // = filteredPos  (what PID acts on)
    float derivative;    // error - lastError
    float correction;    // kp*error + kd*derivative
    int   dynBase;       // base speed after speedDrop penalty
    int   effectiveBase; // baseSpeed or curveSlowSpeed
    int   leftSpd;       // final PWM sent to left motor
    int   rightSpd;      // final PWM sent to right motor
    bool  inCurveMode;   // Physic 3 active flag
    int   curveAbove;    // consecutive loops above curve threshold
    float dist;          // last ultrasonic reading (cm)
    unsigned long loopMs; // millis() at time of snapshot
};
extern PidDebugSnapshot pidSnap;

// ── API ───────────────────────────────────────────────────────────────────
void robot_followLine();
void robot_runLostLineRecovery();
void robot_executeRedAvoid();
void robot_executeGreenPick();
void robot_resetRecovery();
bool robot_recoveryIdle();
bool robot_pickCooldownActive();

// Debug log  (call every DBG_VERBOSE_INTERVAL ms from main loop)
// Prints HUMAN + CSV line to Serial when DBG_VERBOSE=1
void robot_debugLog();
