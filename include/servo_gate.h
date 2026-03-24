// =========================================================================
//  servo_gate.h  –  Single-servo drop-gate / gripper
//
//  Mechanism:
//    HOME angle  →  gate CLOSED  (cube retained in pocket)
//    PICK angle  →  gate OPEN    (cube released / drops)
//
//  The servo sweeps slowly so the gate does not shock the cube out
//  of position.  Angles are stored in Params (SVHOME / SVPICK).
// =========================================================================
#pragma once
#include <Arduino.h>

void servo_init();
void servo_close();                          // move to HOME angle (hold)
void servo_open();                           // move to PICK angle (release)
void servo_moveTo(int fromAngle, int toAngle, int stepDelayMs);
