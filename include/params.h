// =========================================================================
//  params.h  –  Tunable parameter struct + extern declaration
//  All values are live-tunable via Serial  "SET KEY VALUE"
// =========================================================================
#pragma once
#include <Arduino.h>

struct Params {
    // ── PID / line-follow ────────────────────────────────────────────────────────────────
    int   baseSpeed;           // normal forward speed
    int   fastSpeed;           // fast outer wheel in wide turns
    int   slowSpeed;           // slow inner wheel in wide turns
    int   turnSpeed;           // recovery pivot speed
    int   sharpSpeed;          // sharp 90° turn speed
    int   recoverSpeed;        // post-obstacle recovery speed
    int   searchSpeed;         // pivot search speed when line lost
    int   reverseSpeed;        // reverse speed in recovery
    int   reverseBiasDelta;    // asymmetry added when reversing with bias
    int   forwardRecoverSpeed; // creep-forward speed in recovery fwd stage
    //                           FIX C5: raised 60→90; brief line-loss no longer
    //                           drops speed to 37% of baseSpeed
    int   minSpeed;            // minimum PID dynBase (prevents stall)

    unsigned long timeoutLeft;         // max ms for left recovery pivot
    unsigned long timeoutRight;        // max ms for right recovery pivot / reverse
    unsigned long forwardRecoverTime;  // ms to creep forward in recovery

    float kp;          // PID proportional gain
    float kd;          // PID derivative gain
    //                   FIX C4: reduced 10→5; derivative spikes no longer
    //                   slam inner wheel to near-stall on error rate changes
    float posFilter;   // low-pass α for position  (0=raw, 1=frozen)
    //                   FIX C2: reduced 0.65→0.50; filteredPos settles to 0
    //                   faster so dynBase penalty shrinks on straights
    float widthKp;     // line-width correction gain
    float speedDrop;   // BUG-6: PWM units subtracted per unit of |error|
    //                   formula: dynBase = baseSpeed - |error| * speedDrop
    //                   FIX: exposed as tunable param (default 4.0)
    //                   previously hardcoded 8.0 (over-penalised at baseSpeed=160)

    int leftTrim;      // +n ≡ left motor gets +n PWM to compensate weaker side
    //                   FIX C1: set to 0; value of 8 created asymmetry that
    //                   forced PID to over-correct, dragging average speed down
    int rightTrim;

    // ── Obstacle / Color / Servo ────────────────────────────────────────────────
    int   approachSpeed;       // slow speed while closing on obstacle
    int   avoidSpeed;          // speed used during avoidance maneuver
    int   pickApproachSpeed;   // speed during final green-pick approach

    unsigned long reverseAvoidTime;    // ms to reverse before side-step
    unsigned long forwardAvoidTime;    // ms forward past obstacle side
    unsigned long turn90AvoidTime;     // ms for each 90° pivot  ← tune first

    float obstacleSlowDist;    // cm: start slowing / paying attention
    float colorCheckDist;      // cm: stop and read color sensor
    float greenPickDist;       // cm: close enough to pick green cube

    int   redThresh;    // pulseIn threshold – lower = more reflective red
    int   greenThresh;  // pulseIn threshold – lower = more reflective green

    int   servoHomeAngle;  // gate CLOSED angle (holding cube)
    int   servoPickAngle;  // gate OPEN  angle (releasing cube)
};

extern Params P;

// API
void params_init();
bool params_loadEEPROM();
void params_saveEEPROM();
void params_printStatus();
void params_handleSerial();
