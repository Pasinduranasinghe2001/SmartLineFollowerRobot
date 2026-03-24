// =========================================================================
//  sensors.cpp  –  5-channel IR line sensor (analog / BFD-1000 / TCRT5000)
//
//  ESP32 ADC: 12-bit (0–4095).
//  Calibration captures real floor and line readings, so all thresholds
//  self-adapt to whatever surface and lighting conditions are present.
//
//  strength[i] = 0–1000 normalised (0 = background, 1000 = line centre)
//  This feeds the weighted-average PID position computation.
// =========================================================================
#include <Arduino.h>
#include "sensors.h"
#include "config.h"

// ── Exported calibration arrays (also used by params EEPROM) ─────────────
int calBlack[5];
int calWhite[5];
int calThresh[5];
int irStrength[5];
int irRaw[5];
bool irOn[5];

SeenSide lastSeenSide = SIDE_UNKNOWN;

// Weighted position constants  (S1=far-left … S5=far-right)
static const int WEIGHT[5] = { -4, -2, 0, 2, 4 };
static float lastPidPos = 0.0f;

// ── Init ─────────────────────────────────────────────────────────────────
void sensors_init() {
    for (int i = 0; i < 5; i++) {
        pinMode(IR_PIN[i], INPUT);
        // Factory defaults for 12-bit ADC; replaced after calibration
        calBlack[i]  = 3600;
        calWhite[i]  =  400;
        calThresh[i] = 2000;
    }
}

// ── Read all sensors ──────────────────────────────────────────────────────
void sensors_read() {
    for (int i = 0; i < 5; i++) {
        int r = analogRead(IR_PIN[i]);    // 0–4095
        irRaw[i] = r;
        irOn[i]  = (r <= calThresh[i]);   // true = line detected

        int whiteRef = calWhite[i];
        int blackRef = calBlack[i];
        // Ensure enough contrast gap to avoid division weirdness
        if (blackRef <= whiteRef + 20) blackRef = whiteRef + 200;

        long st = map(r, blackRef, whiteRef, 0, 1000);
        irStrength[i] = (int)constrain(st, 0, 1000);
    }
}

// ── Boolean helpers ───────────────────────────────────────────────────────
bool sensors_allDark() {
    return !irOn[0] && !irOn[1] && !irOn[2] && !irOn[3] && !irOn[4];
}
bool sensors_anyWhite() {
    return irOn[0] || irOn[1] || irOn[2] || irOn[3] || irOn[4];
}

// ── Bit pattern helper (MSB = S1/left, LSB = S5/right) ───────────────────
int sensors_bits() {
    return (irOn[0] ? 16 : 0) | (irOn[1] ? 8 : 0) | (irOn[2] ? 4 : 0)
         | (irOn[3] ?  2 : 0) | (irOn[4] ? 1 : 0);
}

// ── Pattern recognition ───────────────────────────────────────────────────
// LEFT  turn : 10000 / 11000 / 11100  (line sweeps left)
bool sensors_isLeftTurnPattern() {
    int b = sensors_bits();
    return (b == 0b10000 || b == 0b11000 || b == 0b11100);
}
// RIGHT turn : 00001 / 00011 / 00111  (line sweeps right)
bool sensors_isRightTurnPattern() {
    int b = sensors_bits();
    return (b == 0b00001 || b == 0b00011 || b == 0b00111);
}
// Usable centre path → resume PID
bool sensors_isPathFoundPattern() {
    int b = sensors_bits();
    return (b == 0b00100 || b == 0b01100 || b == 0b00110 ||
            b == 0b01110 || b == 0b01000 || b == 0b00010);
}

// ── Weighted analog position  –4 (far-left) … +4 (far-right) ─────────────
float sensors_computePosition() {
    long wsum = 0, total = 0;
    for (int i = 0; i < 5; i++) {
        wsum  += (long)WEIGHT[i] * (long)irStrength[i];
        total += (long)irStrength[i];
    }
    if (total < 50) return lastPidPos;   // all dark – keep last known
    lastPidPos = (float)wsum / (float)total;
    return lastPidPos;
}

// ── Line width estimate (active sensor count + analog sum) ────────────────
float sensors_computeWidth() {
    long total = 0;
    int  active = 0;
    for (int i = 0; i < 5; i++) {
        total += irStrength[i];
        if (irStrength[i] > 250) active++;
    }
    return active + (float)total / 5000.0f;
}

// ── Track which side the line was last seen on ────────────────────────────
void sensors_updateLastSeenSide() {
    if (irOn[0] || irOn[1])  lastSeenSide = SIDE_LEFT;
    if (irOn[3] || irOn[4])  lastSeenSide = SIDE_RIGHT;
    if (irOn[2] && !irOn[0] && !irOn[1] && !irOn[3] && !irOn[4])
        lastSeenSide = SIDE_CENTER;
    // Outermost sensors override
    if (irOn[0]) lastSeenSide = SIDE_LEFT;
    if (irOn[4]) lastSeenSide = SIDE_RIGHT;
}

// ── Two-phase interactive calibration ─────────────────────────────────────
void sensors_calibrate() {
    Serial.println(F("=== CALIBRATION START ==="));
    Serial.println(F("PHASE 1: Place ALL sensors over FLOOR (background). Send any key..."));
    while (!Serial.available()) delay(50);
    while ( Serial.available()) Serial.read();

    long accB[5] = {};
    for (int r = 0; r < 60; r++) {
        for (int i = 0; i < 5; i++) accB[i] += analogRead(IR_PIN[i]);
        delay(10);
    }
    for (int i = 0; i < 5; i++) calBlack[i] = (int)(accB[i] / 60);

    Serial.println(F("PHASE 2: Place ALL sensors over the YELLOW LINE. Send any key..."));
    while (!Serial.available()) delay(50);
    while ( Serial.available()) Serial.read();

    long accW[5] = {};
    for (int r = 0; r < 60; r++) {
        for (int i = 0; i < 5; i++) accW[i] += analogRead(IR_PIN[i]);
        delay(10);
    }
    for (int i = 0; i < 5; i++) {
        calWhite[i]  = (int)(accW[i] / 60);
        calThresh[i] = (calBlack[i] + calWhite[i]) / 2;
    }

    Serial.println(F("=== CALIBRATION DONE ==="));
    for (int i = 0; i < 5; i++) {
        Serial.printf("  S%d  Floor=%d  Line=%d  Thresh=%d\n",
                      i+1, calBlack[i], calWhite[i], calThresh[i]);
    }
    Serial.println(F("Type SAVE to persist to EEPROM."));
}
