// =========================================================================
//  ultrasonic.cpp  –  HC-SR04, cached reading every 50 ms
//  ⚠  ECHO line: 5 V output → use 1 kΩ + 2 kΩ divider before GPIO 18
// =========================================================================
#include <Arduino.h>
#include "ultrasonic.h"
#include "config.h"

static float         _cached   = 999.0f;
static unsigned long _lastMs   = 0;
static const unsigned long CACHE_MS = 50;

void ultrasonic_init() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);
}

float ultrasonic_getDistance() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);   // 30 ms timeout ≈ 500 cm
    if (dur == 0) return 999.0f;                   // no echo = open space
    return dur * 0.034f / 2.0f;
}

float ultrasonic_getCached() {
    unsigned long now = millis();
    if (now - _lastMs >= CACHE_MS) {
        _cached = ultrasonic_getDistance();
        _lastMs = now;
    }
    return _cached;
}
