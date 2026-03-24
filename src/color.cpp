// =========================================================================
//  color.cpp  –  TCS3200 color sensor
//
//  TCS3200 outputs a square wave whose frequency is proportional to light
//  intensity on the selected colour filter.  pulseIn() measures half-period
//  in µs:  lower value = higher frequency = more light = more of that colour.
//
//  Frequency scale pins:  S0=HIGH, S1=LOW  →  20 % output (good balance)
//
//  Red   filter: S2=LOW,  S3=LOW
//  Green filter: S2=HIGH, S3=HIGH
//  Blue  filter: S2=LOW,  S3=HIGH
// =========================================================================
#include <Arduino.h>
#include "color.h"
#include "config.h"
#include "params.h"

void color_init() {
    pinMode(PIN_CS_S0,  OUTPUT);
    pinMode(PIN_CS_S1,  OUTPUT);
    pinMode(PIN_CS_S2,  OUTPUT);
    pinMode(PIN_CS_S3,  OUTPUT);
    pinMode(PIN_CS_OUT, INPUT);

    // 20 % frequency scale
    digitalWrite(PIN_CS_S0, HIGH);
    digitalWrite(PIN_CS_S1, LOW);
}

// Read one colour channel (measures pulseIn half-period µs)
static int _readChannel(bool s2, bool s3) {
    digitalWrite(PIN_CS_S2, s2 ? HIGH : LOW);
    digitalWrite(PIN_CS_S3, s3 ? HIGH : LOW);
    delay(30);                                    // settle time
    return (int)pulseIn(PIN_CS_OUT, LOW, 100000UL);
}

void color_readRGB(int &r, int &g, int &b) {
    r = _readChannel(false, false);   // Red
    g = _readChannel(true,  true);    // Green
    b = _readChannel(false, true);    // Blue
}

// Average 3 reads for stability and decide colour
ColorResult color_detect() {
    long rSum = 0, gSum = 0, bSum = 0;
    for (int i = 0; i < 3; i++) {
        int r, g, b;
        color_readRGB(r, g, b);
        rSum += r; gSum += g; bSum += b;
        delay(10);
    }
    int r = (int)(rSum / 3);
    int g = (int)(gSum / 3);
    int b = (int)(bSum / 3);

    Serial.printf("[COLOR] R:%d  G:%d  B:%d\n", r, g, b);

    // Red cube:   R channel has lowest (strongest) value, below threshold
    if (r < P.redThresh && r <= g && r <= b) {
        Serial.println(F("[COLOR] >> RED"));
        return COLOR_RED;
    }
    // Green cube: G channel has lowest value, below threshold
    if (g < P.greenThresh && g <= r && g <= b) {
        Serial.println(F("[COLOR] >> GREEN"));
        return COLOR_GREEN;
    }

    Serial.println(F("[COLOR] >> NONE / UNKNOWN"));
    return COLOR_NONE;
}
