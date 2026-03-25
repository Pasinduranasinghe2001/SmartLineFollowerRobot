// =========================================================================
//  main.cpp  -  EC6090 Mini-Project: Integrated Line-Following Robot
//
//  Task sequence:
//    1. Follow yellow line (PID + 7-channel MD0370 digital IR)
//    2. Detect obstacle  ->  slow approach
//    3. Stop at colorCheckDist, read TCS3200
//       |-- RED cube   ->  avoidance maneuver (bypass left side)
//       '-- GREEN cube ->  pick (close gate, carry to end zone)
//    4. End zone (line ends)  ->  open gate, drop green cube
//
//  Board  : ESP32 DevKit (38-pin)
//  Driver : L298N  (right=ch-A, left=ch-B)
//  Sensors: MD0370 x7 digital IR, HC-SR04, TCS3200
//  Servo  : single SG90/MG90S drop-gate
//           1 deg  = OPEN   (gripper releases cube)
//           110 deg = CLOSED (gripper holds cube)
//
//  Serial commands (115200 baud):
//    STATUS | SAVE | LOAD | SET KEY VALUE
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

static bool cubeOnBoard = false;

// End-zone debounce
static const unsigned long END_ZONE_DARK_MS = 500UL;
static unsigned long _darkSince = 0;

// DBG_VERBOSE timing
static unsigned long _lastVerboseMs = 0;

// =========================================================================
//  DEBUG BANNER
// =========================================================================
static void _printDebugBanner() {
#if DBG_DISABLE_OBSTACLE || DBG_DISABLE_COLOR  || DBG_DISABLE_SERVO  || \
    DBG_DISABLE_ENDZONE  || DBG_DISABLE_RECOVERY || DBG_DISABLE_SPEEDDROP || \
    DBG_VERBOSE
    Serial.println(F("\n+----------------------------------------+"));
    Serial.println(F(  "|   DEBUG MODE - NOT FOR COMPETITION!    |"));
    Serial.println(F(  "+----------------------------------------+"));
#if DBG_DISABLE_OBSTACLE
    Serial.println(F("  [DBG] OBSTACLE detection  : DISABLED"));
#endif
#if DBG_DISABLE_COLOR
    Serial.println(F("  [DBG] COLOUR detection    : DISABLED"));
#endif
#if DBG_DISABLE_SERVO
    Serial.println(F("  [DBG] SERVO gate          : DISABLED"));
#endif
#if DBG_DISABLE_ENDZONE
    Serial.println(F("  [DBG] END-ZONE drop       : DISABLED"));
#endif
#if DBG_DISABLE_RECOVERY
    Serial.println(F("  [DBG] LOST-LINE recovery  : DISABLED (motors stop on dark)"));
#endif
#if DBG_DISABLE_SPEEDDROP
    Serial.println(F("  [DBG] SPEED-DROP penalty  : DISABLED (flat baseSpeed always)"));
#endif
#if DBG_VERBOSE
    Serial.printf (  "  [DBG] VERBOSE output      : ON every %d ms\n",
                     DBG_VERBOSE_INTERVAL);
#endif
    Serial.println();
#endif
}

// =========================================================================
//  setup()
// =========================================================================
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

    _printDebugBanner();

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    _darkSince   = 0;
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();

#if DBG_DISABLE_SERVO
    Serial.println(F("[DBG] Servo disabled - skipping servo_close() at boot."));
#else
    servo_close();
#endif

    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// =========================================================================
//  loop()
// =========================================================================
void loop() {
    params_handleSerial();

    switch (robotState) {

        // Normal PID line following
        case ST_LINE_FOLLOW: {

#if DBG_DISABLE_OBSTACLE
            // Obstacle gating completely skipped
#else
            // FIX-P3: skip obstacle detection during post-pick cooldown
            // so the just-picked cube is not re-detected as RED immediately
            if (!robot_pickCooldownActive()) {
                float dist = ultrasonic_getCached();
                if (dist < P.obstacleSlowDist) {
                    Serial.printf("[STATE] Obstacle %.1f cm -> OBSTACLE_SLOW\n", dist);
                    robotState = ST_OBSTACLE_SLOW;
                    break;
                }
            } else {
                // Cooldown active - log once per second so we can see it
                static unsigned long _lastCoolLog = 0;
                if (millis() - _lastCoolLog > 1000UL) {
                    Serial.println(F("[PICK] Cooldown active - obstacle check suppressed"));
                    _lastCoolLog = millis();
                }
            }
#endif

            sensors_read();
            if (!sensors_allDark()) {
                sensors_updateLastSeenSide();
                if (robot_recoveryIdle() || sensors_isPathFoundPattern())
                    robot_followLine();
                else {
#if DBG_DISABLE_RECOVERY
                    motors_stop();
#else
                    robot_runLostLineRecovery();
#endif
                }
            } else {
#if DBG_DISABLE_RECOVERY
                motors_stop();
#else
                robot_runLostLineRecovery();
#endif
            }
            break;
        }

        // Slowing and approaching obstacle for colour read
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
#if DBG_DISABLE_COLOR
                Serial.println(F("[DBG] Colour detect disabled -> returning to LINE_FOLLOW."));
                robotState = ST_LINE_FOLLOW;
#else
                robotState = ST_COLOR_DETECT;
#endif
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
#if DBG_DISABLE_COLOR
            robotState = ST_LINE_FOLLOW;
#else
            ColorResult col = color_detect();
            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED -> AVOID"));      robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN -> PICK"));     robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown -> FOLLOW")); robotState = ST_LINE_FOLLOW; }
#endif
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
            cubeOnBoard = true;
            _darkSince  = 0;
            Serial.println(F("[STATE] Cube secured -> continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // End-zone drop (debounced)
#if DBG_DISABLE_ENDZONE
    // End-zone drop skipped entirely
#else
    if (cubeOnBoard) {
        bool allDarkNow = sensors_allDark();
        bool clearAhead = (ultrasonic_getCached() >= P.obstacleSlowDist);

        if (allDarkNow && clearAhead) {
            if (_darkSince == 0) {
                _darkSince = millis();
                Serial.println(F("[DROP] All-dark started - debouncing..."));
            } else if (millis() - _darkSince >= END_ZONE_DARK_MS) {
                Serial.printf("[DROP] End zone confirmed (%lu ms dark) -> opening gate\n",
                              millis() - _darkSince);
                motors_stop();
                delay(300);
#if DBG_DISABLE_SERVO
                Serial.println(F("[DBG] Servo disabled - skipping gate open/close."));
#else
                servo_open();
                delay(1200);
                servo_close();
#endif
                cubeOnBoard = false;
                _darkSince  = 0;
                Serial.println(F("[DROP] Done. Robot halted."));
                while (true) delay(100);
            }
        } else {
            if (_darkSince != 0) {
                Serial.println(F("[DROP] Dark interrupted - debounce reset."));
                _darkSince = 0;
            }
        }
    }
#endif

    delay(5);
}
