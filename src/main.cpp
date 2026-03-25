// =========================================================================
//  main.cpp  –  EC6090 Mini-Project: Integrated Line-Following Robot
//
//  Task sequence:
//    1. Follow yellow line (PID + 5-channel analog IR)
//    2. Detect obstacle  →  slow approach
//    3. Stop at colorCheckDist, read TCS3200
//       ├─ RED cube   →  avoidance maneuver (bypass left side)
//       └─ GREEN cube →  pick (close gate, carry to end zone)
//    4. End zone (line ends)  →  open gate, drop green cube
//
//  Board  : ESP32 DevKit (38-pin)
//  Driver : L298N  (right=ch-A, left=ch-B)
//  Sensors: BFD-1000 / TCRT5000 5-ch IR, HC-SR04, TCS3200
//  Servo  : single SG90/MG90S drop-gate
//
//  Serial commands (115200 baud):
//    CALIBRATE | STATUS | SAVE | LOAD | SET KEY VALUE
//
//  DEBUG: set flags in include/config.h to disable subsystems for testing
//    DBG_DISABLE_OBSTACLE  DBG_DISABLE_COLOR   DBG_DISABLE_SERVO
//    DBG_DISABLE_ENDZONE   DBG_DISABLE_RECOVERY  DBG_DISABLE_SPEEDDROP
//    DBG_VERBOSE
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

// ── BUG-03 end-zone debounce ───────────────────────────────────────────────────────
static const unsigned long END_ZONE_DARK_MS = 500UL;
static unsigned long _darkSince = 0;

// ── DBG_VERBOSE timing ────────────────────────────────────────────────────────────
static unsigned long _lastVerboseMs = 0;

// =========================================================================
//  DEBUG BANNER  –  printed once at boot so you know which flags are active
// =========================================================================
static void _printDebugBanner() {
#if DBG_DISABLE_OBSTACLE || DBG_DISABLE_COLOR  || DBG_DISABLE_SERVO  || \
    DBG_DISABLE_ENDZONE  || DBG_DISABLE_RECOVERY || DBG_DISABLE_SPEEDDROP || \
    DBG_VERBOSE
    Serial.println(F("\n┌────────────────────────────────────────┐"));
    Serial.println(F(  "│   DEBUG MODE – NOT FOR COMPETITION!     │"));
    Serial.println(F(  "└────────────────────────────────────────┘"));
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
//  DBG_VERBOSE: print sensors + PID + distance once per interval
// =========================================================================
#if DBG_VERBOSE
static void _verbosePrint() {
    unsigned long now = millis();
    if (now - _lastVerboseMs < DBG_VERBOSE_INTERVAL) return;
    _lastVerboseMs = now;

    // Raw IR bits
    extern bool sensors_getS(int i);  // defined in sensors.cpp
    Serial.printf("[V] IR=%d%d%d%d%d  dist=%.1f  state=%d  recov=%d\n",
        sensors_getS(0) ? 1 : 0,
        sensors_getS(1) ? 1 : 0,
        sensors_getS(2) ? 1 : 0,
        sensors_getS(3) ? 1 : 0,
        sensors_getS(4) ? 1 : 0,
        ultrasonic_getCached(),
        (int)robotState,
        robot_recoveryIdle() ? 0 : 1);
}
#endif

// =========================================================================
//  setup()
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] EC6090 Mini-Project – ESP32 Robot"));

    EEPROM.begin(EEPROM_SIZE);
    params_init();

    if (!params_loadEEPROM()) {
        Serial.println(F("[BOOT] No EEPROM data – using factory defaults."));
        Serial.println(F("[BOOT] Send: CALIBRATE  then  SAVE"));
    } else {
        Serial.println(F("[BOOT] EEPROM loaded OK."));
        params_printStatus();
    }

    motors_init();
    sensors_init();
    ultrasonic_init();
    color_init();
    servo_init();

    _printDebugBanner();   // ← prints active debug flags after all inits

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    _darkSince   = 0;
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();

#if DBG_DISABLE_SERVO
    Serial.println(F("[DBG] Servo disabled – skipping servo_close() at boot."));
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

#if DBG_VERBOSE
    sensors_read();        // ensure fresh data for verbose print
    _verbosePrint();
#endif

    switch (robotState) {

        // ── Normal PID line following ─────────────────────────────────────────────
        case ST_LINE_FOLLOW: {

#if DBG_DISABLE_OBSTACLE
            // Obstacle gating completely skipped — run pure PID only
#else
            float dist = ultrasonic_getCached();
            if (dist < P.obstacleSlowDist) {
                Serial.printf("[STATE] Obstacle %.1f cm → OBSTACLE_SLOW\n", dist);
                robotState = ST_OBSTACLE_SLOW;
                break;
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
                    Serial.println(F("[DBG] allDark in follow – recovery disabled, motors stopped."));
#else
                    robot_runLostLineRecovery();
#endif
                }
            } else {
#if DBG_DISABLE_RECOVERY
                motors_stop();
                Serial.println(F("[DBG] allDark – recovery disabled, motors stopped."));
#else
                robot_runLostLineRecovery();
#endif
            }
            break;
        }

        // ── Slowing and approaching obstacle for colour read ──────────────────
        case ST_OBSTACLE_SLOW: {
            float dist = ultrasonic_getCached();

            if (dist > P.obstacleSlowDist + 3.0f) {
                Serial.println(F("[STATE] Clear → LINE_FOLLOW"));
                robotState = ST_LINE_FOLLOW;
                break;
            }
            if (dist <= P.colorCheckDist) {
                motors_stop();
                Serial.printf("[STATE] dist %.1f cm → COLOR_DETECT\n", dist);

#if DBG_DISABLE_COLOR
                Serial.println(F("[DBG] Colour detect disabled → returning to LINE_FOLLOW."));
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

        // ── Stopped: read colour ────────────────────────────────────────────────
        case ST_COLOR_DETECT: {
            motors_stop();
            delay(200);

#if DBG_DISABLE_COLOR
            // Should not reach here when DBG_DISABLE_COLOR=1 because
            // ST_OBSTACLE_SLOW already redirects, but guard anyway.
            Serial.println(F("[DBG] ST_COLOR_DETECT reached with colour disabled → LINE_FOLLOW."));
            robotState = ST_LINE_FOLLOW;
#else
            ColorResult col = color_detect();
            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED → AVOID"));      robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN → PICK"));     robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown → FOLLOW")); robotState = ST_LINE_FOLLOW; }
#endif
            break;
        }

        // ── Bypass red cube ──────────────────────────────────────────────────────
        case ST_RED_AVOID:
            robot_executeRedAvoid();
            robotState = ST_LINE_FOLLOW;
            break;

        // ── Pick green cube ──────────────────────────────────────────────────────
        case ST_GREEN_PICK:
            robot_executeGreenPick();
            cubeOnBoard = true;
            _darkSince  = 0;
            Serial.println(F("[STATE] Cube secured → continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // ── End-zone drop (debounced) ────────────────────────────────────────────────
#if DBG_DISABLE_ENDZONE
    // End-zone drop skipped entirely — robot follows forever
#else
    if (cubeOnBoard) {
        bool allDarkNow = sensors_allDark();
        bool clearAhead = (ultrasonic_getCached() >= P.obstacleSlowDist);

        if (allDarkNow && clearAhead) {
            if (_darkSince == 0) {
                _darkSince = millis();
                Serial.println(F("[DROP] All-dark started – debouncing..."));
            } else if (millis() - _darkSince >= END_ZONE_DARK_MS) {
                Serial.printf("[DROP] End zone confirmed (%lu ms dark) → opening gate\n",
                              millis() - _darkSince);
                motors_stop();
                delay(300);
#if DBG_DISABLE_SERVO
                Serial.println(F("[DBG] Servo disabled – skipping gate open/close."));
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
                Serial.println(F("[DROP] Dark interrupted – debounce reset."));
                _darkSince = 0;
            }
        }
    }
#endif

    delay(5);
}
