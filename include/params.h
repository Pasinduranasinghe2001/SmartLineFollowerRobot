// =========================================================================
//  params.h  -  Tunable parameter struct + extern declaration
//  All values are live-tunable via Serial  "SET KEY VALUE"
//  or via MQTT topic robot/params/set  {"key":"KP","value":"18.5"}
// =========================================================================
#pragma once
#include <Arduino.h>

struct Params {
    // ── PID / line-follow ─────────────────────────────────────────────────
    int   baseSpeed;            // normal forward speed
    int   fastSpeed;            // fast outer wheel in wide turns
    int   slowSpeed;            // slow inner wheel in wide turns
    int   turnSpeed;            // recovery pivot speed
    int   sharpSpeed;           // sharp 90 deg turn speed
    int   recoverSpeed;         // post-obstacle recovery speed
    int   searchSpeed;          // pivot search speed when line lost
    int   reverseSpeed;         // reverse speed in recovery
    int   reverseBiasDelta;     // asymmetry added when reversing with bias
    int   forwardRecoverSpeed;  // creep-forward speed in recovery fwd stage
    int   minSpeed;             // minimum PID dynBase (prevents stall)

    unsigned long timeoutLeft;          // max ms for left recovery pivot
    unsigned long timeoutRight;         // max ms for right recovery pivot/reverse
    unsigned long forwardRecoverTime;   // ms to creep forward in recovery

    float kp;          // PID proportional gain
    float kd;          // PID derivative gain
    float posFilter;   // low-pass alpha for position (0=raw, 1=frozen)
    float widthKp;     // line-width correction gain
    float speedDrop;   // PWM units subtracted per unit of |error|

    int leftTrim;      // +n = left motor gets +n PWM to compensate weaker side
    int rightTrim;

    // ── Physic 3: Curvature-Adaptive Speed ──────────────────────────────
    //  When |filteredPos| stays above curveDetectThresh for curveConfirmLoops
    //  consecutive PID loops, base speed drops to curveSlowSpeed.
    //  Speed returns to baseSpeed when |filteredPos| drops back below
    //  threshold for CURVE_EXIT_LOOPS consecutive loops (hard-coded 5).
    //
    //  Tuning guide:
    //    curveDetectThresh: start at 0.6 (outer loops), drop to 0.4 for
    //                       inner triangle. Lower = triggers on gentler curves.
    //    curveSlowSpeed:    start at 100, raise until robot stays on line
    //                       through the sharpest triangle corner.
    //    curveConfirmLoops: 3-5 loops prevents noise triggering slow-down.
    //
    //  MQTT keys:  CURVTHR  CURVSPD  CURVLOOPS
    float curveDetectThresh;   // |filteredPos| threshold to enter curve mode
    int   curveSlowSpeed;      // baseSpeed override while in curve mode
    int   curveConfirmLoops;   // consecutive loops above thresh to enter mode

    // ── Physic 4: Obstacle Side Memory ──────────────────────────────────
    //  avoidPreferRight = 0  ->  always avoid LEFT (original behaviour)
    //  avoidPreferRight = 1  ->  auto-select side from lastSeenSide:
    //      lastSeenSide == SIDE_LEFT  -> obstacle is on left  -> avoid RIGHT
    //      lastSeenSide == SIDE_RIGHT -> obstacle is on right -> avoid LEFT
    //      SIDE_UNKNOWN               -> fall back to LEFT
    //
    //  Arena context: the two RED obstacles are on opposite sides of the
    //  track (left outer loop, bottom inner path). Auto-select ensures the
    //  avoidance trajectory always goes outward, not into the track wall.
    //
    //  MQTT key:  AVDSIDE  (0 or 1)
    int   avoidPreferRight;    // 0=always left, 1=auto-select from lastSeenSide

    // ── Obstacle / Color / Servo ──────────────────────────────────────────
    int   approachSpeed;        // slow speed while closing on obstacle
    int   avoidSpeed;           // speed used during avoidance maneuver
    int   pickApproachSpeed;    // speed during final green-pick approach

    unsigned long reverseAvoidTime;   // ms to reverse before side-step
    unsigned long forwardAvoidTime;   // ms forward past obstacle side
    unsigned long turn90AvoidTime;    // ms for each 90 deg pivot

    float obstacleSlowDist;    // cm: start slowing / paying attention
    float colorCheckDist;      // cm: stop and read color sensor
    float greenPickDist;       // cm: close enough to pick green cube

    int   redThresh;    // pulseIn threshold - lower = more reflective red
    int   greenThresh;  // pulseIn threshold - lower = more reflective green

    int   servoHomeAngle;   // gate CLOSED angle (holding cube)  = 110 deg
    int   servoPickAngle;   // gate OPEN  angle (releasing cube) =   1 deg
};

extern Params P;

// API
void params_init();
bool params_loadEEPROM();
void params_saveEEPROM();
void params_printStatus();
void params_handleSerial();
