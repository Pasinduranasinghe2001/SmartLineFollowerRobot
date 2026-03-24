// =========================================================================
//  motors.h  –  Motor control API
// =========================================================================
#pragma once
#include <Arduino.h>

void motors_init();
void motors_stop();

// Straight / reverse
void motors_driveStraight(int spd);
void motors_driveBackward(int spd);

// Biased reverse (recovery – keep heading toward last-seen side)
void motors_driveBackwardBiasLeft(int fast, int slow);
void motors_driveBackwardBiasRight(int fast, int slow);

// Differential steer (one wheel faster, one slower – soft curve)
void motors_steerLeft(int fast, int slow);
void motors_steerRight(int fast, int slow);

// Pivot in place (one wheel fwd, other bwd)
void motors_pivotLeft(int spd);
void motors_pivotRight(int spd);

// Low-level (used by PID directly)
void motors_setLeft(int spd, bool forward);
void motors_setRight(int spd, bool forward);
