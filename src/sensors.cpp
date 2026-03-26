// =========================================================================
//  sensors.cpp  -  7-channel MD0370 digital IR line sensor
//
//  DUAL-ZONE STRATEGY:
//  DETECT zone (inner): S2, S3, S4  -> weighted position ±2 for PD error
//  HOLD   zone (outer): S0, S6      -> curve/hold watchdogs
//
//  Position output: -6.0 (far left) ... 0.0 (centre) ... +6.0 (far right)
//  WEIGHT mapping (S0..S6, left -> right):
//    S0=-6  S1=-4  S2=-2  S3=0  S4=+2  S5=+4  S6=+6
// =========================================================================
#include <Arduino.h>
#include "sensors.h"
#include "config.h"

// Exported state arrays
int  irRaw[IR_SENSOR_COUNT];
bool irOn [IR_SENSOR_COUNT];

SeenSide lastSeenSide = SIDE_UNKNOWN;

static const int WEIGHT[IR_SENSOR_COUNT] = { -6, -4, -2, 0, 2, 4, 6 };

// Inner-zone weights: only S2(-2), S3(0), S4(+2) used for PD error
static const int INNER_WEIGHT[IR_SENSOR_COUNT] = { 0, 0, -2, 0, 2, 0, 0 };
static const bool IS_INNER[IR_SENSOR_COUNT]    = { false, false, true, true, true, false, false };

static float lastPidPos      = 0.0f;
static float lastInnerPos    = 0.0f;

// -------------------------------------------------------------------------
//  Init
// -------------------------------------------------------------------------
void sensors_init() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        pinMode(IR_PIN[i], INPUT);
        irRaw[i] = 0;
        irOn[i]  = false;
    }
    Serial.printf("[sensors] MD0370 x%d  activeLevel=%s  DUAL-ZONE=ON\n",
                  IR_SENSOR_COUNT,
                  LINE_ACTIVE_LOW ? "LOW" : "HIGH");
    Serial.println(F("[sensors] HOLD zone : S0(L-far) S6(R-far)"));
    Serial.println(F("[sensors] DETECT zone: S2 S3 S4 (inner ±2)"));
}

// -------------------------------------------------------------------------
//  Read all 7 sensors
// -------------------------------------------------------------------------
void sensors_read() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int v    = digitalRead(IR_PIN[i]);
        irRaw[i] = v;
#if LINE_ACTIVE_LOW
        irOn[i] = (v == LOW);
#else
        irOn[i] = (v == HIGH);
#endif
    }
}

// -------------------------------------------------------------------------
//  DUAL-ZONE helpers
// -------------------------------------------------------------------------

// Outer HOLD zone - far-left sensor (S0)
bool sensors_outerLeft() {
    return irOn[0];   // S0 = weight -6
}

// Outer HOLD zone - far-right sensor (S6)
bool sensors_outerRight() {
    return irOn[6];   // S6 = weight +6
}

// Both outer sensors active at once -> extremely sharp curve
bool sensors_isSharpCurve() {
    return irOn[0] && irOn[6];
}

// Any outer HOLD sensor active
bool sensors_outerActive() {
    return irOn[0] || irOn[6];
}

// -------------------------------------------------------------------------
//  Boolean helpers (all 7 sensors)
// -------------------------------------------------------------------------
bool sensors_allDark() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        if (irOn[i]) return false;
    return true;
}

// All 7 sensors over line simultaneously (1111111)
bool sensors_allOn() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        if (!irOn[i]) return false;
    return true;
}

bool sensors_anyOn() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        if (irOn[i]) return true;
    return false;
}

// 7-bit bitmap: S0 = bit6 (MSB), S6 = bit0 (LSB)
int sensors_bits() {
    int b = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (irOn[i]) b |= (1 << (IR_SENSOR_COUNT - 1 - i));
    }
    return b;
}

// -------------------------------------------------------------------------
//  Pattern recognition
// -------------------------------------------------------------------------
bool sensors_isLeftTurnPattern() {
    int b = sensors_bits();
    return (b & 0b1000000) && !(b & 0b0000111);
}

bool sensors_isRightTurnPattern() {
    int b = sensors_bits();
    return (b & 0b0000001) && !(b & 0b1110000);
}

bool sensors_isPathFoundPattern() {
    int b = sensors_bits();
    return (b & 0b0011100) != 0;
}

// -------------------------------------------------------------------------
//  Full 7-sensor weighted position  -6.0 .. +6.0
//  (kept for compatibility with recovery and debug)
// -------------------------------------------------------------------------
float sensors_computePosition() {
    long  wsum  = 0;
    int   count = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (irOn[i]) { wsum += WEIGHT[i]; count++; }
    }
    if (count == 0) return lastPidPos;
    lastPidPos = (float)wsum / (float)count;
    return lastPidPos;
}

// -------------------------------------------------------------------------
//  INNER-zone weighted position  -2.0 .. +2.0
//  Uses only S2(-2), S3(0), S4(+2) for PD error calculation.
//  Falls back to full 7-sensor position if inner zone is all dark
//  (line is fully in outer hold zone -> return last known inner pos).
// -------------------------------------------------------------------------
float sensors_computeInnerPosition() {
    long  wsum  = 0;
    int   count = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (IS_INNER[i] && irOn[i]) {
            wsum  += INNER_WEIGHT[i];
            count++;
        }
    }
    if (count == 0) {
        // Line has moved to outer zone - preserve last inner position
        // so PD error stays valid (do not snap to zero)
        return lastInnerPos;
    }
    lastInnerPos = (float)wsum / (float)count;
    return lastInnerPos;
}

// -------------------------------------------------------------------------
//  Track which side the line was last seen on
// -------------------------------------------------------------------------
void sensors_updateLastSeenSide() {
    // Outer hold sensors update side memory first
    if (irOn[0]) lastSeenSide = SIDE_LEFT;
    if (irOn[6]) lastSeenSide = SIDE_RIGHT;
    // Inner sensors can refine if outer are both off
    if (!irOn[0] && !irOn[6]) {
        if (irOn[1] || irOn[2]) lastSeenSide = SIDE_LEFT;
        if (irOn[4] || irOn[5]) lastSeenSide = SIDE_RIGHT;
        if (irOn[3] && !irOn[1] && !irOn[2] && !irOn[4] && !irOn[5])
            lastSeenSide = SIDE_CENTER;
    }
}

// -------------------------------------------------------------------------
//  Debug bitmap print
// -------------------------------------------------------------------------
void sensors_printDebug() {
    Serial.print("[IR] ");
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (i == 1 || i == 5) Serial.print('|'); // zone boundary markers
        Serial.print(irOn[i] ? 'X' : '.');
    }
    Serial.printf("  full=%+.2f  inner=%+.2f  last=%s  hold=%s%s\n",
                  sensors_computePosition(),
                  lastInnerPos,
                  lastSeenSide == SIDE_LEFT   ? "LEFT"   :
                  lastSeenSide == SIDE_RIGHT  ? "RIGHT"  :
                  lastSeenSide == SIDE_CENTER ? "CENTER" : "UNKN",
                  irOn[0] ? "L" : ".",
                  irOn[6] ? "R" : ".");
}
