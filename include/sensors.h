// =========================================================================
//  sensors.h  -  7-channel MD0370 digital IR line sensor API
//
//  DUAL-ZONE STRATEGY:
//  DETECT zone (inner): S2, S3, S4  -> weighted position ±2 for PD error
//  HOLD   zone (outer): S0, S6      -> curve/hold watchdogs
//
//  MD0370 output polarity:
//    LINE_ACTIVE_LOW = true   -> module pulls OUT LOW when over line
//    LINE_ACTIVE_LOW = false  -> module pulls OUT HIGH when over line
//
//  Sensor layout (left -> right):
//    [S0]  [S1] | [S2]  [S3]  [S4] | [S5]  [S6]
//    HOLD-L ---   <-- DETECT zone -->   --- HOLD-R
//
//  Tune with: STATUS command in serial monitor
//  Bitmap check: '.' = background, 'X' = line, '|' = zone boundary
// =========================================================================
#pragma once
#include <Arduino.h>

#define IR_SENSOR_COUNT  7

// Output polarity (flip if bitmap is inverted)
#define LINE_ACTIVE_LOW  true

// Last-seen side memory (used by recovery)
enum SeenSide { SIDE_UNKNOWN, SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT };
extern SeenSide lastSeenSide;

// Sensor state (exported for robot.cpp / main.cpp)
extern int  irRaw[IR_SENSOR_COUNT];   // 0 or 1 raw digitalRead value
extern bool irOn [IR_SENSOR_COUNT];   // true = line detected (polarity-corrected)

// ── Core API ─────────────────────────────────────────────────────────────
void  sensors_init();
void  sensors_read();

// ── Boolean helpers (all 7 sensors) ──────────────────────────────────────
bool  sensors_allDark();            // all 7 see background  (0000000)
bool  sensors_allOn();              // all 7 see line        (1111111)
bool  sensors_anyOn();              // at least one sees line
int   sensors_bits();               // 7-bit bitmap: S0=MSB, S6=LSB

// ── Pattern recognition ───────────────────────────────────────────────────
bool  sensors_isLeftTurnPattern();  // line exiting left edge
bool  sensors_isRightTurnPattern(); // line exiting right edge
bool  sensors_isPathFoundPattern(); // usable centre path found

// ── Position computation ──────────────────────────────────────────────────
// Full 7-sensor weighted average  -6.0 .. +6.0  (used for debug / Physic3)
float sensors_computePosition();

// DETECT-zone weighted average using only S2/S3/S4  -> -2.0 .. +2.0
// Falls back to last known inner position when inner zone is all dark.
// Use this for PD error in robot_followLine().
float sensors_computeInnerPosition();

// ── Dual-zone HOLD helpers (outer sensors S0 / S6) ───────────────────────
bool  sensors_outerLeft();          // S0 (far-left,  weight -6) sees line
bool  sensors_outerRight();         // S6 (far-right, weight +6) sees line
bool  sensors_outerActive();        // either outer sensor active
bool  sensors_isSharpCurve();       // both S0 AND S6 active simultaneously

// ── Side memory ───────────────────────────────────────────────────────────
void  sensors_updateLastSeenSide();

// ── Debug ─────────────────────────────────────────────────────────────────
void  sensors_printDebug();         // print live bitmap + position to Serial
