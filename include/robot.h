// =========================================================================
//  robot.h  -  Top-level robot state machine
// =========================================================================
#pragma once

enum RobotState {
    ST_LINE_FOLLOW,     // normal PID line following
    ST_OBSTACLE_SLOW,   // obstacle detected, slowing and approaching
    ST_COLOR_DETECT,    // stopped, reading color sensor
    ST_RED_AVOID,       // executing red-cube avoidance maneuver
    ST_GREEN_PICK       // executing green-cube pick-and-place
};

extern RobotState robotState;

void  robot_followLine();
void  robot_runLostLineRecovery();
void  robot_resetRecovery();
void  robot_executeRedAvoid();
void  robot_executeGreenPick();
bool  robot_recoveryIdle();          // true if recovery state == REC_IDLE
bool  robot_pickCooldownActive();    // true for 2.5 s after green pick completes
