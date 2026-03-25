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
#include "mqtt_params.h" // MQTT handling depends on Params, so include after params.h

static bool cubeOnBoard = false;   // true after green pick, until end-zone drop

// ── BUG-03 FIX: end-zone debounce ────────────────────────────────────────
//  PROBLEM (before fix):
//    The end-zone drop triggered the instant sensors_allDark() was true
//    while cubeOnBoard was set.  A tape gap, sensor lift-off on a bump,
//    or a T-junction during recovery all produce a momentary allDark()
//    reading that would falsely fire the drop sequence mid-track.
//
//  FIX:
//    Introduce a static timestamp `_darkSince`.  The drop only fires when
//    allDark() has been CONTINUOUSLY true for at least END_ZONE_DARK_MS.
//    The moment any IR sensor sees the line again the timer resets to 0,
//    so a brief gap never accumulates enough time to trigger the drop.
//
//  Tuning:
//    500 ms was chosen because:
//      • At baseSpeed=110, 500 ms ≈ 5–6 cm of travel – enough to confirm
//        the robot has genuinely passed the end of the track.
//      • Normal tape gaps on competition tracks are < 100 ms at this speed.
//    Adjust END_ZONE_DARK_MS via Serial: SET ENDZONE 400  (not yet a Param
//    field – hardcoded constant is sufficient for this project scope).
// ──────────────────────────────────────────────────────────────────────────
static const unsigned long END_ZONE_DARK_MS = 500UL;  // ms all-dark must persist
static unsigned long _darkSince = 0;                  // 0 = not currently dark

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] EC6090 Mini-Project \u2013 ESP32 Robot"));

    EEPROM.begin(EEPROM_SIZE);

    params_init();

    // BUG-10 FIX: load EEPROM BEFORE servo_init() so saved servo angles
    // are applied on the first write inside servo_init().
    if (!params_loadEEPROM()) {
        Serial.println(F("[BOOT] No EEPROM data \u2013 using factory defaults."));
        Serial.println(F("[BOOT] Send: CALIBRATE  then  SAVE"));
    } else {
        Serial.println(F("[BOOT] EEPROM loaded OK."));
        params_printStatus();
    }

    motors_init();
    sensors_init();
    ultrasonic_init();
    color_init();
    servo_init();       // uses LEDC timers 2 & 3  (motors use 0 & 1)
                        // reads P.servoHomeAngle – correct because EEPROM already loaded

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    _darkSince   = 0;              // BUG-03: initialise debounce timer
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();
    servo_close();      // confirm gate closed at boot

    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    params_handleSerial();

    switch (robotState) {

        // ── Normal PID line following ─────────────────────────────────────
        case ST_LINE_FOLLOW: {
            float dist = ultrasonic_getCached();

            if (dist < P.obstacleSlowDist) {
                Serial.printf("[STATE] Obstacle %.1f cm \u2192 OBSTACLE_SLOW\n", dist);
                robotState = ST_OBSTACLE_SLOW;
                break;
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

        // ── Slowing and approaching obstacle for colour read ──────────────
        case ST_OBSTACLE_SLOW: {
            float dist = ultrasonic_getCached();

            if (dist > P.obstacleSlowDist + 3.0f) {
                Serial.println(F("[STATE] Clear \u2192 LINE_FOLLOW"));
                robotState = ST_LINE_FOLLOW;
                break;
            }
            if (dist <= P.colorCheckDist) {
                motors_stop();
                Serial.printf("[STATE] dist %.1f cm \u2192 COLOR_DETECT\n", dist);
                robotState = ST_COLOR_DETECT;
                break;
            }

            // Still closing: follow line at reduced base speed
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

        // ── Stopped: read colour ──────────────────────────────────────────
        case ST_COLOR_DETECT: {
            motors_stop();
            delay(200);
            ColorResult col = color_detect();

            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED \u2192 AVOID"));      robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN \u2192 PICK"));     robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown \u2192 FOLLOW")); robotState = ST_LINE_FOLLOW; }
            break;
        }

        // ── Bypass red cube (blocking U-shape maneuver) ───────────────────
        case ST_RED_AVOID:
            robot_executeRedAvoid();
            robotState = ST_LINE_FOLLOW;
            break;

        // ── Pick green cube (gate closes, robot carries to end zone) ──────
        // BUG-02 FIX: servo_close() removed from here; it lives exclusively
        // inside robot_executeGreenPick() to avoid double-write servo jerk.
        case ST_GREEN_PICK:
            robot_executeGreenPick();          // servo_close() is inside here
            cubeOnBoard = true;
            _darkSince  = 0;                   // BUG-03: reset debounce on pick
            Serial.println(F("[STATE] Cube secured \u2192 continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // ── End-zone drop  (BUG-03 FIXED: debounced) ─────────────────────────
    //
    //  Condition to DROP:
    //    1. cubeOnBoard         – we actually picked a green cube
    //    2. sensors_allDark()   – all 5 IR sensors see no line
    //    3. dist >= slowDist    – no obstacle directly ahead (open space)
    //    4. CONTINUOUSLY dark   – allDark() has been true for ≥500 ms
    //
    //  Condition to RESET timer:
    //    Any IR sensor sees the line again (¬allDark) – we’re still on track.
    //
    if (cubeOnBoard) {
        bool allDarkNow = sensors_allDark();
        bool clearAhead = (ultrasonic_getCached() >= P.obstacleSlowDist);

        if (allDarkNow && clearAhead) {
            if (_darkSince == 0) {
                // First loop iteration where condition is true – start timer
                _darkSince = millis();
                Serial.println(F("[DROP] All-dark started – debouncing..."));
            } else if (millis() - _darkSince >= END_ZONE_DARK_MS) {
                // Dark has persisted long enough – genuine end zone reached
                Serial.printf("[DROP] End zone confirmed (%lu ms dark) \u2192 opening gate\n",
                              millis() - _darkSince);
                motors_stop();
                delay(300);
                servo_open();
                delay(1200);
                servo_close();
                cubeOnBoard = false;
                _darkSince  = 0;
                Serial.println(F("[DROP] Done. Robot halted."));
                while (true) delay(100);
            }
            // else: still accumulating dark time – do nothing this loop tick
        } else {
            // Line visible again or obstacle ahead – reset debounce timer
            if (_darkSince != 0) {
                Serial.println(F("[DROP] Dark interrupted – debounce reset."));
                _darkSince = 0;
            }
        }
    }

    delay(5);
}
