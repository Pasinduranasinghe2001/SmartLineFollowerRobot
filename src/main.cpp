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

static bool cubeOnBoard = false;   // true after green pick, until end-zone drop

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] EC6090 Mini-Project – ESP32 Robot"));

    EEPROM.begin(EEPROM_SIZE);

    params_init();

    // BUG-10 note: load EEPROM BEFORE servo_init() so saved servo angles
    // are applied on first write inside servo_init().
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
    servo_init();       // uses LEDC timers 2 & 3 (motors use 0 & 1)
                        // reads P.servoHomeAngle which is now correct

    robotState   = ST_LINE_FOLLOW;
    cubeOnBoard  = false;
    lastSeenSide = SIDE_UNKNOWN;
    robot_resetRecovery();
    servo_close();      // confirm gate closed at start

    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    params_handleSerial();

    switch (robotState) {

        // ── Normal PID line following ───────────────────────────────────────────
        case ST_LINE_FOLLOW: {
            float dist = ultrasonic_getCached();

            if (dist < P.obstacleSlowDist) {
                Serial.printf("[STATE] Obstacle %.1f cm → OBSTACLE_SLOW\n", dist);
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
                robotState = ST_COLOR_DETECT;
                break;
            }

            // Still closing: follow line at slower base speed
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
            ColorResult col = color_detect();

            if      (col == COLOR_RED)   { Serial.println(F("[STATE] RED → AVOID"));      robotState = ST_RED_AVOID;   }
            else if (col == COLOR_GREEN) { Serial.println(F("[STATE] GREEN → PICK"));     robotState = ST_GREEN_PICK;  }
            else                         { Serial.println(F("[STATE] Unknown → FOLLOW")); robotState = ST_LINE_FOLLOW; }
            break;
        }

        // ── Bypass red cube (blocking U-shape maneuver) ─────────────────────────
        case ST_RED_AVOID:
            robot_executeRedAvoid();
            robotState = ST_LINE_FOLLOW;
            break;

        // ── Pick green cube (blocking; gate closes over cube) ──────────────────
        // BUG-02 FIX:
        //   Removed the `servo_close()` call that was here before
        //   robot_executeGreenPick().  robot_executeGreenPick() already
        //   calls servo_close() as its very first action (see robot.cpp).
        //   Calling it twice caused the servo to receive the same angle
        //   command twice in rapid succession, producing a brief jerk that
        //   could dislodge the cube from the pocket during the approach.
        //   The single authoritative servo_close() now lives exclusively
        //   inside robot_executeGreenPick().
        case ST_GREEN_PICK:
            robot_executeGreenPick();          // servo_close() is inside here
            cubeOnBoard = true;
            Serial.println(F("[STATE] Cube secured → continue to end zone"));
            robotState = ST_LINE_FOLLOW;
            break;
    }

    // ── End-zone drop ────────────────────────────────────────────────────────────
    // All 5 sensors dark + clear ahead = robot has gone past the end of line
    if (cubeOnBoard &&
        sensors_allDark() &&
        ultrasonic_getCached() >= P.obstacleSlowDist) {
        Serial.println(F("[DROP] End zone → opening gate"));
        motors_stop();
        delay(300);
        servo_open();
        delay(1200);
        servo_close();
        cubeOnBoard = false;
        Serial.println(F("[DROP] Done. Robot halted."));
        while (true) delay(100);
    }

    delay(5);
}
