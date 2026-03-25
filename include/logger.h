// =========================================================================
//  logger.h  -  Offline LittleFS CSV logger
//
//  Appends one CSV row per PID debug tick to /pidlog.csv on LittleFS.
//  Designed to work WITHOUT a USB/Serial connection during the run.
//
//  Usage:
//    logger_init()         -> call in setup()  (mounts LittleFS)
//    logger_appendRow()    -> call from robot_debugLog() every interval
//    logger_dump()         -> streams entire file over Serial (post-run)
//    logger_clear()        -> deletes /pidlog.csv
//    logger_printInfo()    -> prints file size + free space to Serial
//
//  Serial commands (handled by params_handleSerial):
//    DUMPLOG   -> calls logger_dump()
//    CLEARLOG  -> calls logger_clear()
//    LOGINFO   -> calls logger_printInfo()
// =========================================================================
#pragma once
#include <Arduino.h>
#include "robot.h"   // for PidDebugSnapshot

void logger_init();
void logger_appendRow(const PidDebugSnapshot& s);
void logger_dump();
void logger_clear();
void logger_printInfo();
