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
//  End-zone detection (3-phase pattern matching):
//    The physical end-zone marking is a wide white band crossing the full
//    track width, with the track ending just after it. As the robot crosses
//    this band while carrying the cube, the sensor pattern is:
//
//      Phase 1  WAIT_FIRST_ON:   1111111  (all 7 over white band)
//      Phase 2  WAIT_ALL_OFF:    0000000  (all 7 over dark gap after band)
//      Phase 3  WAIT_SECOND_ON:  1111111  (all 7 over second white band / end)
//                                          -> trigger drop
//
//    Each phase must hold for endZoneHoldMs (default 60 ms) before
//    advancing. Any failure resets back to Phase 1.
//    This pattern CANNOT be faked by a T-junction, bump, or single glitch.
//
//  Board  : ESP32 DevKit (38-pin)
//  Driver : L298N  (right=ch-A, left=ch-B)
//  Sensors: MD0370 7-ch digital IR, HC-SR04, TCS3200
//  Servo  : single MG996R drop-gate
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
#include "mqtt_params.h"

// ── Cube state ────────────────────────────────────────────────────────────
static bool cubeOnBoard = false;

// ── End-zone 3-phase detector state ─────────────────────────────────────────
//
//  IDLE            - cubeOnBoard is false; detector is off
//  WAIT_FIRST_ON   - watching for all 7 sensors to go ON  (1111111)
//  WAIT_ALL_OFF    - sensors went allOn; now watching for allDark (0000000)
//  WAIT_SECOND_ON  - sensors went dark;  now watching for allOn again (1111111)
//                    when this phase holds -> trigger drop
//
enum EndZonePhase {
    EZ_IDLE,
    EZ_WAIT_FIRST_ON,
    EZ_WAIT_ALL_OFF,
    EZ_WAIT_SECOND_ON
};

static EndZonePhase   _ezPhase     = EZ_IDLE;
static unsigned long  _ezPhaseMs   = 0;   // millis() when current phase condition first held

// Reset detector to beginning of phase 1
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
    mqtt_init();

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    _ezPhase     = EZ_IDLE;
    _ezPhaseMs   = 0;
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();
    servo_close();

    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// ───────────────────────────────────────────────────────────────────────────
void loop() {
    mqtt_loop();
    params_handleSerial();

    // ── Main robot state machine ───────────────────────────────────────────────
    switch (robotState) {

        // Normal PID line following
        case ST_LINE_FOLLOW: {
            // Skip obstacle check while pick cooldown is active
            if (!robot_pickCooldownActive()) {
                float dist = ultrasonic_getCached();
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

        // Slowing and approaching obstacle
        case ST_OBSTACLE_SLOW: {
            float dist = ultrasonic_getCached();

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

        // Stopped: read colour
        case ST_COLOR_DETECT: {
            motors_stop();
            delay(200);
            ColorResult col = color_detect();

            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED -> AVOID"));     robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN -> PICK"));    robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown -> FOLLOW")); robotState = ST_LINE_FOLLOW; }
            break;
        }

        // Bypass red cube
        case ST_RED_AVOID:
            robot_executeRedAvoid();
            robotState = ST_LINE_FOLLOW;
            break;

        // Pick green cube
        case ST_GREEN_PICK:
            robot_executeGreenPick();
            cubeOnBoard  = true;
            _ezPhase     = EZ_IDLE;   // will be activated below on next loop
            _ezPhaseMs   = 0;
            Serial.println(F("[STATE] Cube secured -> continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // ── End-zone 3-phase detector ──────────────────────────────────────────────
    //
    //  Only runs when cubeOnBoard == true.
    //  Physical marking the robot crosses:
    //
    //    |== white band ==|== dark gap ==|== white band ==| track ends
    //       1111111            0000000        1111111
    //       Phase 1            Phase 2        Phase 3 -> DROP
    //
    //  Each phase must hold for P.endZoneHoldMs ms.
    //  If a phase breaks early (noise/bump), detector resets to Phase 1.
    //
    if (cubeOnBoard) {

        // Activate detector after pick
        if (_ezPhase == EZ_IDLE) {
            _ezReset();   // -> EZ_WAIT_FIRST_ON
        }

        sensors_read();
        unsigned long now = millis();

        switch (_ezPhase) {

            // ─ Phase 1: wait for all 7 sensors to see the line (1111111) ────
            case EZ_WAIT_FIRST_ON:
                if (sensors_allOn()) {
                    if (_ezPhaseMs == 0) {
                        _ezPhaseMs = now;
                        Serial.println(F("[ENDZONE] Phase1: allOn started"));
                    } else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        Serial.printf("[ENDZONE] Phase1 confirmed (%lu ms) -> WAIT_ALL_OFF\n",
                                      now - _ezPhaseMs);
                        _ezPhase   = EZ_WAIT_ALL_OFF;
                        _ezPhaseMs = 0;
                    }
                } else {
                    // Condition broke - reset timer (but stay in phase 1)
                    if (_ezPhaseMs != 0) {
                        Serial.println(F("[ENDZONE] Phase1 broken - restart timer"));
                        _ezPhaseMs = 0;
                    }
                }
                break;

            // ─ Phase 2: wait for all 7 sensors to go dark (0000000) ──────
            case EZ_WAIT_ALL_OFF:
                if (sensors_allDark()) {
                    if (_ezPhaseMs == 0) {
                        _ezPhaseMs = now;
                        Serial.println(F("[ENDZONE] Phase2: allDark started"));
                    } else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        Serial.printf("[ENDZONE] Phase2 confirmed (%lu ms) -> WAIT_SECOND_ON\n",
                                      now - _ezPhaseMs);
                        _ezPhase   = EZ_WAIT_SECOND_ON;
                        _ezPhaseMs = 0;
                    }
                } else {
                    // Line appeared again before dark was confirmed -> full reset
                    if (_ezPhaseMs != 0 || sensors_anyOn()) {
                        Serial.println(F("[ENDZONE] Phase2 broken - full reset"));
                        _ezReset();
                    }
                }
                break;

            // ─ Phase 3: wait for all 7 to go ON again (1111111) -> DROP ──
            case EZ_WAIT_SECOND_ON:
                if (sensors_allOn()) {
                    if (_ezPhaseMs == 0) {
                        _ezPhaseMs = now;
                        Serial.println(F("[ENDZONE] Phase3: allOn started"));
                    } else if (now - _ezPhaseMs >= P.endZoneHoldMs) {
                        // ──────────── CONFIRMED END ZONE ────────────
                        Serial.printf("[ENDZONE] Phase3 confirmed (%lu ms) -> DROP\n",
                                      now - _ezPhaseMs);
                        motors_stop();
                        delay(300);

                        Serial.println(F("[DROP] Opening gate..."));
                        servo_open();     // release cube
                        delay(1200);
                        servo_close();    // return arm
                        delay(300);

                        cubeOnBoard = false;
                        _ezPhase    = EZ_IDLE;
                        _ezPhaseMs  = 0;

                        Serial.println(F("[DROP] Done. Robot halted."));
                        while (true) delay(100);   // halt until power cycle
                    }
                } else {
                    // Dark again before second allOn confirmed -> full reset
                    if (_ezPhaseMs != 0 || !sensors_anyOn()) {
                        Serial.println(F("[ENDZONE] Phase3 broken - full reset"));
                        _ezReset();
                    }
                }
                break;

            default:
                break;
        }
    }

    delay(5);
}
