// =========================================================================
//  sensors.cpp  –  7-channel MD0370 digital IR line sensor
//
//  MD0370 is a comparator-based digital module. It has an onboard
//  potentiometer that sets the detection threshold. There is no
//  analog calibration needed in firmware – just tune each module's
//  pot until it cleanly switches on your track surface.
//
//  Position output: -6.0 (far left) ... 0.0 (centre) ... +6.0 (far right)
//  This feeds directly into the PID controller in robot.cpp.
//
//  WEIGHT mapping (S0..S6, left → right):
//    S0=-6  S1=-4  S2=-2  S3=0  S4=+2  S5=+4  S6=+6
//
//  BITMAP display (sensors_printDebug):
//    '.' = background  'X' = line detected
//    Example: ...XXX..  means line is slightly right of centre
// =========================================================================
#include <Arduino.h>
#include "sensors.h"
#include "config.h"

// ── Exported state arrays ─────────────────────────────────────────────────
int  irRaw[IR_SENSOR_COUNT];
bool irOn [IR_SENSOR_COUNT];

SeenSide lastSeenSide = SIDE_UNKNOWN;

// Weights: evenly spaced, -6 to +6 across 7 sensors
static const int WEIGHT[IR_SENSOR_COUNT] = { -6, -4, -2, 0, 2, 4, 6 };
static float lastPidPos = 0.0f;

// ── Init ──────────────────────────────────────────────────────────────────
void sensors_init() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        pinMode(IR_PIN[i], INPUT);
        irRaw[i] = 0;
        irOn[i]  = false;
    }
    Serial.printf("[sensors] MD0370 x%d  activeLevel=%s\n",
                  IR_SENSOR_COUNT,
                  LINE_ACTIVE_LOW ? "LOW" : "HIGH");
}

// ── Read all 7 sensors ────────────────────────────────────────────────────
void sensors_read() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int v  = digitalRead(IR_PIN[i]);
        irRaw[i] = v;
#if LINE_ACTIVE_LOW
        irOn[i] = (v == LOW);
#else
        irOn[i] = (v == HIGH);
#endif
    }
}

// ── Boolean helpers ───────────────────────────────────────────────────────
bool sensors_allDark() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        if (irOn[i]) return false;
    return true;
}

bool sensors_anyOn() {
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        if (irOn[i]) return true;
    return false;
}

// ── 7-bit bitmap: S0 = bit6 (MSB), S6 = bit0 (LSB) ──────────────────────
int sensors_bits() {
    int b = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (irOn[i]) b |= (1 << (IR_SENSOR_COUNT - 1 - i));
    }
    return b;
}

// ── Pattern recognition (7-sensor, left/right/centre) ────────────────────
//
//  LEFT turn  : only leftmost sensors lit  (0b1110000, 0b1100000, 0b1000000)
//  RIGHT turn : only rightmost sensors lit (0b0000111, 0b0000011, 0b0000001)
//  Path found : at least one centre sensor (S2,S3,S4) lit, not all dark

bool sensors_isLeftTurnPattern() {
    int b = sensors_bits();
    // S0 lit AND no right sensors (S4,S5,S6) lit
    return (b & 0b1000000) && !(b & 0b0000111);
}

bool sensors_isRightTurnPattern() {
    int b = sensors_bits();
    // S6 lit AND no left sensors (S0,S1,S2) lit
    return (b & 0b0000001) && !(b & 0b1110000);
}

bool sensors_isPathFoundPattern() {
    int b = sensors_bits();
    // Any of S2, S3, S4 is on
    return (b & 0b0011100) != 0;
}

// ── Digital weighted position  -6.0 (far-left) .. +6.0 (far-right) ──────
//
//  Uses only active (irOn) sensors. If no sensor sees the line,
//  returns last known position so PID keeps last correction.
float sensors_computePosition() {
    long wsum  = 0;
    int  count = 0;

    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (irOn[i]) {
            wsum  += WEIGHT[i];
            count++;
        }
    }

    if (count == 0) return lastPidPos;   // all dark → keep last known
    lastPidPos = (float)wsum / (float)count;
    return lastPidPos;
}

// ── Track which side the line was last seen on ────────────────────────────
void sensors_updateLastSeenSide() {
    // Outer-left pair: S0, S1
    if (irOn[0] || irOn[1]) lastSeenSide = SIDE_LEFT;
    // Outer-right pair: S5, S6
    if (irOn[5] || irOn[6]) lastSeenSide = SIDE_RIGHT;
    // Strictly centred: only S3 lit, no outer sensors
    if ( irOn[3] &&
        !irOn[0] && !irOn[1] &&
        !irOn[5] && !irOn[6]) lastSeenSide = SIDE_CENTER;
    // Hard overrides for outermost sensors
    if (irOn[0]) lastSeenSide = SIDE_LEFT;
    if (irOn[6]) lastSeenSide = SIDE_RIGHT;
}

// ── Debug bitmap print ────────────────────────────────────────────────────
void sensors_printDebug() {
    Serial.print("[IR] ");
    for (int i = 0; i < IR_SENSOR_COUNT; i++)
        Serial.print(irOn[i] ? 'X' : '.');
    Serial.printf("  pos=%+.2f  last=%s\n",
                  sensors_computePosition(),
                  lastSeenSide == SIDE_LEFT   ? "LEFT"   :
                  lastSeenSide == SIDE_RIGHT  ? "RIGHT"  :
                  lastSeenSide == SIDE_CENTER ? "CENTER" : "UNKN");
}
