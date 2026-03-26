/*
 * MD0370 IR Sensor Array — Hardware Calibration
 * 7 sensors: IR_0 (leftmost) to IR_6 (rightmost)
 * 
 * PROCEDURE:
 *   1. Upload this sketch
 *   2. Open Serial Monitor at 115200 baud
 *   3. Follow the printed prompts:
 *      - Phase 1: Slide robot slowly over WHITE surface for 5 seconds
 *      - Phase 2: Slide robot slowly over BLACK line for 5 seconds
 *   4. Thresholds are printed — copy them into your main params
 *
 * Press the BOOT/FLASH button (or send any character) to advance each phase.
 */

#include <Arduino.h>
#include <EEPROM.h>

// ─── CONFIG ──────────────────────────────────────────────────────────────────
const int   NUM_SENSORS    = 7;
const int   IR_PINS[NUM_SENSORS] = {32, 33, 34, 35, 27, 36, 39 };  // adjust if needed
const int   CAL_DURATION_MS = 5000;   // ms per calibration phase
const int   SAMPLE_INTERVAL_MS = 20;  // sample every 20 ms
const int   EEPROM_BASE    = 50;       // EEPROM address to store thresholds
                                        // (avoid address 0 if used by params)
// ─────────────────────────────────────────────────────────────────────────────

int   minVal[NUM_SENSORS];
int   maxVal[NUM_SENSORS];
int   threshold[NUM_SENSORS];

// ─── HELPERS ─────────────────────────────────────────────────────────────────
void resetMinMax() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        minVal[i] = 1023;
        maxVal[i] = 0;
    }
}

void updateMinMax() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        int v = analogRead(IR_PINS[i]);
        if (v < minVal[i]) minVal[i] = v;
        if (v > maxVal[i]) maxVal[i] = v;
    }
}

void printRaw() {
    Serial.print("RAW: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(analogRead(IR_PINS[i]));
        if (i < NUM_SENSORS - 1) Serial.print(" | ");
    }
    Serial.println();
}

void waitForSerial(const char* msg) {
    Serial.println();
    Serial.println(msg);
    Serial.println(">>> Send any character to continue...");
    while (!Serial.available()) { delay(50); }
    while (Serial.available()) Serial.read();  // flush
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=================================================");
    Serial.println("  MD0370 x7 IR SENSOR CALIBRATION");
    Serial.println("  Sensor order: S0=LEFT ... S6=RIGHT");
    Serial.println("=================================================");

    // ── LIVE VIEW ─────────────────────────────────────────────────────────
    Serial.println("\n[LIVE] Current raw sensor readings:");
    for (int t = 0; t < 30; t++) {
        printRaw();
        delay(100);
    }

    // ── PHASE 1: WHITE SURFACE ────────────────────────────────────────────
    waitForSerial("[PHASE 1] Place robot on WHITE surface and move slowly.\n         Calibration will run for 5 seconds.");

    resetMinMax();
    Serial.println("Scanning WHITE surface...");
    unsigned long start = millis();
    while (millis() - start < CAL_DURATION_MS) {
        updateMinMax();
        // Print progress bar
        int elapsed = (millis() - start) * 20 / CAL_DURATION_MS;
        Serial.print("[");
        for (int k = 0; k < 20; k++) Serial.print(k < elapsed ? "#" : "-");
        Serial.print("] ");
        printRaw();
        delay(SAMPLE_INTERVAL_MS);
    }

    int whiteMin[NUM_SENSORS], whiteMax[NUM_SENSORS];
    memcpy(whiteMin, minVal, sizeof(minVal));
    memcpy(whiteMax, maxVal, sizeof(maxVal));

    Serial.println("\n[WHITE] Results:");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("  S"); Serial.print(i);
        Serial.print(": min="); Serial.print(whiteMin[i]);
        Serial.print("  max="); Serial.println(whiteMax[i]);
    }

    // ── PHASE 2: BLACK LINE ───────────────────────────────────────────────
    waitForSerial("[PHASE 2] Move robot over the BLACK line slowly.\n         Calibration will run for 5 seconds.");

    resetMinMax();
    Serial.println("Scanning BLACK line...");
    start = millis();
    while (millis() - start < CAL_DURATION_MS) {
        updateMinMax();
        int elapsed = (millis() - start) * 20 / CAL_DURATION_MS;
        Serial.print("[");
        for (int k = 0; k < 20; k++) Serial.print(k < elapsed ? "#" : "-");
        Serial.print("] ");
        printRaw();
        delay(SAMPLE_INTERVAL_MS);
    }

    int blackMin[NUM_SENSORS], blackMax[NUM_SENSORS];
    memcpy(blackMin, minVal, sizeof(minVal));
    memcpy(blackMax, maxVal, sizeof(maxVal));

    Serial.println("\n[BLACK] Results:");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("  S"); Serial.print(i);
        Serial.print(": min="); Serial.print(blackMin[i]);
        Serial.print("  max="); Serial.println(blackMax[i]);
    }

    // ── COMPUTE THRESHOLDS ────────────────────────────────────────────────
    Serial.println("\n=================================================");
    Serial.println("  COMPUTED THRESHOLDS  (copy into params.cpp)");
    Serial.println("=================================================");

    bool anyBad = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
        // Midpoint between white max and black min
        int whitePeak = whiteMax[i];
        int blackPeak = blackMax[i];   // MD0370: higher ADC = more reflection (white)
                                        // lower  ADC = less reflection  (black)
        threshold[i] = (whitePeak + blackPeak) / 2;

        int spread = abs(whitePeak - blackPeak);

        Serial.print("  const int IR_THRESH_");
        Serial.print(i);
        Serial.print(" = ");
        Serial.print(threshold[i]);
        Serial.print(";   // spread=");
        Serial.print(spread);
        if (spread < 80) {
            Serial.print("  *** LOW CONTRAST — CHECK SENSOR S");
            Serial.print(i);
            Serial.print(" ***");
            anyBad = true;
        }
        Serial.println();
    }

    // ── SAVE TO EEPROM ────────────────────────────────────────────────────
    Serial.println("\nSaving thresholds to EEPROM...");
    for (int i = 0; i < NUM_SENSORS; i++) {
        EEPROM.put(EEPROM_BASE + i * sizeof(int), threshold[i]);
    }
    // Write magic byte to confirm calibration was done
    EEPROM.put(EEPROM_BASE - 2, (int)0xCAFE);
    Serial.println("Saved!");

    // ── VERIFICATION ─────────────────────────────────────────────────────
    Serial.println("\n=================================================");
    Serial.println("  VERIFICATION — Live binary readout");
    Serial.println("  1 = on LINE (black)   0 = off line (white)");
    Serial.println("  Move robot over track to verify");
    Serial.println("=================================================");
    if (anyBad) {
        Serial.println("  !! WARNING: One or more sensors have low contrast.");
        Serial.println("  !! Check height (~3 mm), wiring, and potentiometer.");
    }
    delay(2000);
}

// ─── LOOP: live verification ─────────────────────────────────────────────────
void loop() {
    Serial.print("BINARY [L>R]: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        int v = analogRead(IR_PINS[i]);
        // 1 = on black line (low reflection → low ADC for MD0370)
        Serial.print(v < threshold[i] ? "1" : "0");
        Serial.print(" ");
    }
    Serial.print("  RAW: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(analogRead(IR_PINS[i]));
        Serial.print(" ");
    }
    Serial.println();
    delay(100);
}