// =========================================================================
//  ultrasonic.h  –  HC-SR04 distance sensor
//  ⚠  ECHO pin outputs 5V; use voltage divider before connecting to ESP32
// =========================================================================
#pragma once
#include <Arduino.h>

void  ultrasonic_init();
float ultrasonic_getDistance();      // fresh reading (cm)
float ultrasonic_getCached();        // returns cached value, refreshes every 50 ms
