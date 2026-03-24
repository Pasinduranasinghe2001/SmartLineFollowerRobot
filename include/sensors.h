// =========================================================================
//  sensors.h  –  IR line sensor API (5-channel, BFD-1000 / TCRT5000)
// =========================================================================
#pragma once
#include <Arduino.h>

// ── Last-seen side memory (used by recovery) ─────────────────────────────
enum SeenSide { SIDE_UNKNOWN, SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT };
extern SeenSide lastSeenSide;

// ── Raw & calibration data (exposed for EEPROM in params.cpp) ────────────
extern int calBlack[5];
extern int calWhite[5];
extern int calThresh[5];
extern int irStrength[5];   // 0-1000 normalized
extern int irRaw[5];        // 0-4095 raw ADC
extern bool irOn[5];        // true = line detected

// API
void sensors_init();
void sensors_read();

bool sensors_allDark();
bool sensors_anyWhite();
int  sensors_bits();

bool sensors_isLeftTurnPattern();
bool sensors_isRightTurnPattern();
bool sensors_isPathFoundPattern();

float sensors_computePosition();    // weighted avg  –4..+4
float sensors_computeWidth();       // active sensor count estimate

void sensors_updateLastSeenSide();
void sensors_calibrate();           // interactive 2-phase calibration
