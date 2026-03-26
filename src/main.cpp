// =========================================================================
//  main.cpp  -  EC6090 Mini-Project: Integrated Line-Following Robot
//
//  Task sequence:
//    1. Follow yellow line (PID + 7-channel MD0370 digital IR)
//    2. Detect obstacle  ->  slow approach
//    3. Stop at colorCheckDist, read TCS3200
//       |-- RED cube   ->  avoidance maneuver (side-memory aware)
//       `-- GREEN cube ->  pick (close gate, carry to end zone)
//    4. End zone  ->  open gate, drop green cube
//
//  End-zone: 3-phase pattern 1111111->0000000->1111111
//
//  Debug: set DBG_VERBOSE=1 in config.h to enable PID log
//         Serial Monitor shows HUMAN + CSV lines every DBG_VERBOSE_INTERVAL ms
//         CSV can be copy-pasted into Excel / Serial Plotter for graphs
//         Set DBG_LOG_TO_FILE=1 to save CSV to LittleFS /pidlog.csv
//         without a USB connection. Type DUMPLOG after run to retrieve.
// =========================================================================
#include <Arduino.h>
#include <EEPROM.h>

#include "config.h"
#include "params.h"
#include "motors.h"
#include "sensors.h"
#include "ultrasonic.h"
#include "color.h"
#include "servo_gate.h"
#include "robot.h"
#include "logger.h"
#include "mqtt_params.h"

// Cube state
static bool cubeOnBoard = false;

// Debug log timer
static unsigned long _lastDebugMs = 0;

// End-zone 3-phase detector
enum EndZonePhase {
    EZ_IDLE,
    EZ_WAIT_FIRST_ON,
    EZ_WAIT_ALL_OFF,
    EZ_WAIT_SECOND_ON
};
static EndZonePhase  _ezPhase   = EZ_IDLE;
static unsigned long _ezPhaseMs = 0;

static void _ezReset() {
    _ezPhase   = EZ_WAIT_FIRST_ON;
    _ezPhaseMs = 0;
    Serial.println(F("[ENDZONE] Detector reset -> WAIT_FIRST_ON"));
}

// ───────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] EC6090 Mini-Project - ESP32 Robot"));

    EEPROM.begin(EEPROM_SIZE);
    params_init();

    if (!params_loadEEPROM()) {
        Serial.println(F("[BOOT] No EEPROM data - using factory defaults."));
    } else {
        Serial.println(F("[BOOT] EEPROM loaded OK."));
        params_printStatus();
    }

    motors_init();
    sensors_init();
    ultrasonic_init();
    color_init();
    servo_init();
    logger_init();
    mqtt_init();

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    _ezPhase     = EZ_IDLE;
    _ezPhaseMs   = 0;
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();
    servo_close();

#if DBG_VERBOSE
    Serial.println(F("[BOOT] DBG_VERBOSE=1: PID log active."));
    Serial.printf ("[BOOT] Log interval: %d ms\n", DBG_VERBOSE_INTERVAL);
    Serial.println(F("[BOOT] CSV header will print on first debug tick."));
#endif
#if DBG_LOG_TO_FILE
    Serial.println(F("[BOOT] DBG_LOG_TO_FILE=1: writing /pidlog.csv to LittleFS."));
    Serial.println(F("[BOOT] Type CLEARLOG to wipe before run, DUMPLOG to retrieve."));
#endif

    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// ───────────────────────────────────────────────────────────────────────────
void loop() {
    mqtt_loop();
    params_handleSerial();

    // ── Stamp timestamp and ultrasonic into snapshot every tick ──────────────
    // This ensures t_ms and dist are always current even if followLine() is
    // not called this tick (e.g. robot is in recovery or obstacle slow mode).
    pidSnap.loopMs = millis();
    float dist = ultrasonic_getCached();
    pidSnap.dist = dist;

    // ─ Main robot state machine ──────────────────────────────────────────────
    switch (robotState) {

        case ST_LINE_FOLLOW: {
            if (!robot_pickCooldownActive()) {
                if (dist < P.obstacleSlowDist) {
                    Serial.printf("[STATE] Obstacle %.1f cm -> OBSTACLE_SLOW\n", dist);
                    robotState = ST_OBSTACLE_SLOW;
                    break;
                }
            }
            sensors_read();
            if (!sensors_allDark()) {
                sensors_updateLastSeenSide();
                if (robot_recoveryIdle() || sensors_isPathFoundPattern())
                    robot_followLine();
                else
                    robot_runLostLineRecovery();
            } else {
                robot_runLostLineRecovery();
            }
            break;
        }

        case ST_OBSTACLE_SLOW: {
            if (dist > P.obstacleSlowDist + 3.0f) {
                Serial.println(F("[STATE] Clear -> LINE_FOLLOW"));
                robotState = ST_LINE_FOLLOW;
                break;
            }
            if (dist <= P.colorCheckDist) {
                motors_stop();
                Serial.printf("[STATE] dist %.1f cm -> COLOR_DETECT\n", dist);
                robotState = ST_COLOR_DETECT;
                break;
            }
            sensors_read();
            int savedBase = P.baseSpeed;
            P.baseSpeed   = P.approachSpeed;
            if (!sensors_allDark()) {
                sensors_updateLastSeenSide();
                robot_followLine();
            } else {
                motors_driveStraight(P.approachSpeed);
            }
            P.baseSpeed = savedBase;
            break;
        }

        case ST_COLOR_DETECT: {
            motors_stop();
            delay(200);
            ColorResult col = color_detect();
            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED -> AVOID"));      robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN -> PICK"));     robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown -> FOLLOW")); robotState = ST_LINE_FOLLOW; }
            break;
        }

        case ST_RED_AVOID:
            robot_executeRedAvoid();
            robotState = ST_LINE_FOLLOW;
            break;

        case ST_GREEN_PICK:
            robot_executeGreenPick();
            cubeOnBoard  = true;
            _ezPhase     = EZ_IDLE;
            _ezPhaseMs   = 0;
            Serial.println(F("[STATE] Cube secured -> continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // ─ Debug log timer (fires in ALL states) ─────────────────────────────────
    // Both Serial verbose and LittleFS file logging are driven from here.
    // Separating this from the state machine means you always get a log row
    // even during recovery, obstacle approach, or idle sitting on a bench.
    if (millis() - _lastDebugMs >= DBG_VERBOSE_INTERVAL) {
        _lastDebugMs = millis();
#if DBG_VERBOSE
        robot_debugLog();   // prints HUMAN + HINT + CSV to Serial
                            // also calls logger_appendRow if DBG_LOG_TO_FILE
#elif DBG_LOG_TO_FILE
        logger_appendRow(pidSnap);  // file-only when verbose is off
#endif
    }

    // ─ End-zone 3-phase detector ──────────────────────────────────────────────
    if (cubeOnBoard) {
        if (_ezPhase == EZ_IDLE) _ezReset();

        sensors_read();
        unsigned long now = millis();

        switch (_ezPhase) {
            case EZ_WAIT_FIRST_ON:
                if (sensors_allOn()) {
                    if (_ezPhaseMs == 0) { _ezPhaseMs = now; Serial.println(F("[ENDZONE] Phase1: allOn started")); }
                    else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        Serial.printf("[ENDZONE] Phase1 confirmed (%lu ms) -> WAIT_ALL_OFF\n", now - _ezPhaseMs);
                        _ezPhase = EZ_WAIT_ALL_OFF; _ezPhaseMs = 0;
                    }
                } else {
                    if (_ezPhaseMs != 0) { Serial.println(F("[ENDZONE] Phase1 broken")); _ezPhaseMs = 0; }
                }
                break;

            case EZ_WAIT_ALL_OFF:
                if (sensors_allDark()) {
                    if (_ezPhaseMs == 0) { _ezPhaseMs = now; Serial.println(F("[ENDZONE] Phase2: allDark started")); }
                    else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        Serial.printf("[ENDZONE] Phase2 confirmed (%lu ms) -> WAIT_SECOND_ON\n", now - _ezPhaseMs);
                        _ezPhase = EZ_WAIT_SECOND_ON; _ezPhaseMs = 0;
                    }
                } else {
                    if (_ezPhaseMs != 0 || sensors_anyOn()) { Serial.println(F("[ENDZONE] Phase2 broken - full reset")); _ezReset(); }
                }
                break;

            case EZ_WAIT_SECOND_ON:
                if (sensors_allOn()) {
                    if (_ezPhaseMs == 0) { _ezPhaseMs = now; Serial.println(F("[ENDZONE] Phase3: allOn started")); }
                    else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        Serial.printf("[ENDZONE] Phase3 confirmed (%lu ms) -> DROP\n", now - _ezPhaseMs);
                        motors_stop(); delay(300);
                        Serial.println(F("[DROP] Opening gate..."));
                        servo_open(); delay(1200); servo_close(); delay(300);
                        cubeOnBoard = false; _ezPhase = EZ_IDLE; _ezPhaseMs = 0;
                        Serial.println(F("[DROP] Done. Robot halted."));
                        while (true) delay(100);
                    }
                } else {
                    if (_ezPhaseMs != 0 || !sensors_anyOn()) { Serial.println(F("[ENDZONE] Phase3 broken - full reset")); _ezReset(); }
                }
                break;

            default: break;
        }
    }

    delay(5);
}
