// =========================================================================
//  params.h  -  Tunable parameter struct + extern declaration
//  All values are live-tunable via Serial  "SET KEY VALUE"
//  or via MQTT topic robot/params/set  {"key":"KP","value":"18.5"}
// =========================================================================
#pragma once
#include <Arduino.h>

struct Params {
    // ── PID / line-follow ─────────────────────────────────────────────────
    int   baseSpeed;
    int   fastSpeed;
    int   slowSpeed;
    int   turnSpeed;
    int   sharpSpeed;
    int   recoverSpeed;
    int   searchSpeed;
    int   reverseSpeed;
    int   reverseBiasDelta;
    int   forwardRecoverSpeed;
    int   minSpeed;

    unsigned long timeoutLeft;
    unsigned long timeoutRight;
    unsigned long forwardRecoverTime;

    float kp;
    float kd;
    float posFilter;
    float widthKp;
    float speedDrop;

    int leftTrim;
    int rightTrim;

    // ── Physic 3: Curvature-Adaptive Speed ──────────────────────────────
    float curveDetectThresh;
    int   curveSlowSpeed;
    int   curveConfirmLoops;

    // ── Physic 4: Obstacle Side Memory ──────────────────────────────────
    int   avoidPreferRight;

    // ── Obstacle / Color / Servo ─────────────────────────────────────────
    int   approachSpeed;
    int   avoidSpeed;
    int   pickApproachSpeed;

    unsigned long reverseAvoidTime;
    unsigned long forwardAvoidTime;
    unsigned long turn90AvoidTime;

    float obstacleSlowDist;
    float colorCheckDist;
    float greenPickDist;

    int   redThresh;
    int   greenThresh;

    int   servoHomeAngle;
    int   servoPickAngle;

    // ── End-zone 3-phase detector ─────────────────────────────────────────
    //  Minimum ms each phase must hold before advancing:
    //    Phase 1: allOn()  must be true  for endZoneHoldMs  (1111111)
    //    Phase 2: allDark() must be true for endZoneHoldMs  (0000000)
    //    Phase 3: allOn()  must be true  for endZoneHoldMs  (1111111) -> DROP
    //
    //  Lower  = reacts faster but more prone to noise
    //  Higher = more reliable but requires wider physical markings
    //  Default 60 ms works with a ~3 cm wide white band at baseSpeed=180
    //
    //  MQTT key:  ENDZONEMS
    //  Serial:    SET ENDZONEMS 80
    unsigned long endZoneHoldMs;
};

extern Params P;

// API
void params_init();
bool params_loadEEPROM();
void params_saveEEPROM();
void params_printStatus();
void params_handleSerial();
