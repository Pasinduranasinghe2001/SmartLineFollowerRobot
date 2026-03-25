// =========================================================================
//  sensors.h  –  7-channel MD0370 digital IR line sensor API
//
//  MD0370 output polarity:
//    LINE_ACTIVE_LOW = true   → module pulls OUT LOW when over line
//    LINE_ACTIVE_LOW = false  → module pulls OUT HIGH when over line
//
//  Tune with: STATUS command in serial monitor
//  Bitmap check: '.' = background, 'X' = line
//    Black line on white floor:  set LINE_ACTIVE_LOW to match your modules
// =========================================================================
#pragma once
#include <Arduino.h>

#define IR_SENSOR_COUNT  7

// ── Output polarity  (flip if bitmap is inverted) ────────────────────────
// true  = MD0370 OUT goes LOW  when sensor is ON the line  (most common)
// false = MD0370 OUT goes HIGH when sensor is ON the line
#define LINE_ACTIVE_LOW  true

// ── Last-seen side memory (used by recovery) ─────────────────────────────
enum SeenSide { SIDE_UNKNOWN, SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT };
extern SeenSide lastSeenSide;

// ── Sensor state (exported for robot.cpp / main.cpp) ─────────────────────
extern int  irRaw[IR_SENSOR_COUNT];   // 0 or 1  raw digitalRead value
extern bool irOn [IR_SENSOR_COUNT];   // true = line detected (polarity-corrected)

// ── API ──────────────────────────────────────────────────────────────────
void  sensors_init();
void  sensors_read();

bool  sensors_allDark();            // all 7 see background
bool  sensors_anyOn();              // at least one sees line
int   sensors_bits();               // 7-bit bitmap: S0=MSB, S6=LSB

bool  sensors_isLeftTurnPattern();  // line exiting left edge
bool  sensors_isRightTurnPattern(); // line exiting right edge
bool  sensors_isPathFoundPattern(); // usable centre path found

float sensors_computePosition();    // weighted avg  -6.0 .. +6.0
void  sensors_updateLastSeenSide();
void  sensors_printDebug();         // print live bitmap + position to Serial
