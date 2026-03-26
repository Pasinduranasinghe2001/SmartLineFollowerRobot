// =========================================================================
//  color.h  –  TCS3200 color sensor
// =========================================================================
#pragma once
#include <Arduino.h>

enum ColorResult { COLOR_NONE, COLOR_RED, COLOR_GREEN };

void        color_init();
ColorResult color_detect();          // averages 3 readings, prints R/G/B
void        color_readRGB(int &r, int &g, int &b);  // raw single read
