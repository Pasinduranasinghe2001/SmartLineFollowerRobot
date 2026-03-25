// =========================================================================
//  config.h  –  Pin assignments, LEDC, EEPROM constants
//               + compile-time DEBUG FLAGS
//
//  EC6090 Mini-Project  |  ESP32 Dev Module (38-pin, WROOM-32)
//
//  FINAL TESTED PIN MAP
//  ─────────────────────────────────────────────────────────────────────
//  MOTOR DRIVER (L298N)
//    ENA → GPIO5    IN1 → GPIO18   IN2 → GPIO19
//    ENB → GPIO23   IN3 → GPIO21   IN4 → GPIO22
//
//  IR SENSORS (5-channel analog)
//    IR1 → GPIO32   IR2 → GPIO33   IR3 → GPIO34
//    IR4 → GPIO35   IR5 → GPIO27
//    ⚠ GPIO 32,33,34,35 are INPUT-ONLY (no OUTPUT, no internal pull-up/down)
//
//  COLOR SENSOR (TCS3200)
//    S0 → GPIO14   S1 → GPIO15   S2 → GPIO26
//    S3 → GPIO25   OUT → GPIO2
//    ⚠ GPIO 2 (TCS OUT) is a boot-strapping pin. As an INPUT it is safe;
//      do NOT connect a pull-up resistor to it.
//
//  ULTRASONIC (HC-SR04)
//    TRIG → GPIO17   ECHO → GPIO16
//    ⚠ ECHO outputs 5V – use 1kΩ + 2kΩ voltage divider before GPIO16
//
//  SERVO
//    Signal → GPIO13
//
//  All pins confirmed accessible on the 38-pin ESP32 Dev Module board.
// =========================================================================
#pragma once
#include <Arduino.h>

// =========================================================================
//  DEBUG FLAGS
//  ───────────────────────────────────────────────────────────────────────────
//  Each flag is an independent compile-time switch.
//  Set a flag to 1 to DISABLE that subsystem during a debug/test session.
//  Set it back to 0 for competition / production.
//
//  All flags default to 0 — fully-enabled production behaviour.
//
//  FLAG                     0 (default)              1 (disabled)
//  ───────────────────────────────────────────────────────────────────────────
//  DBG_DISABLE_OBSTACLE     uses ultrasonic          ignores ultrasonic
//                           distance gating          robot never enters
//                                                    OBSTACLE_SLOW
//
//  DBG_DISABLE_COLOR        reads TCS3200            skips colour read,
//                           and acts on RED/GREEN    always goes to
//                                                    LINE_FOLLOW after stop
//
//  DBG_DISABLE_SERVO        servo moves on open/     all servo_open() and
//                           close calls              servo_close() calls
//                                                    are silently skipped
//
//  DBG_DISABLE_ENDZONE      drop gate opens when     end-zone drop is
//                           end of track reached     never triggered;
//                                                    robot always follows
//
//  DBG_DISABLE_RECOVERY     lost-line recovery       on allDark() robot
//                           state machine runs       just stops motors
//                           normally                 (isolates PID only)
//
//  DBG_DISABLE_SPEEDDROP    dynBase reduced by        dynBase always equals
//                           |error| * speedDrop      P.baseSpeed on straights
//                                                    and curves
//
//  DBG_VERBOSE              normal serial output     extra per-loop sensor,
//                                                    PID, and distance
//                                                    values printed every
//                                                    DBG_VERBOSE_INTERVAL ms
// =========================================================================

#define DBG_DISABLE_OBSTACLE   0   // 1 = robot ignores ultrasonic, PID only
#define DBG_DISABLE_COLOR      0   // 1 = skip TCS3200 colour read entirely
#define DBG_DISABLE_SERVO      0   // 1 = servo_open/close calls are no-ops
#define DBG_DISABLE_ENDZONE    0   // 1 = never trigger end-zone gate drop
#define DBG_DISABLE_RECOVERY   0   // 1 = replace recovery with motors_stop()
#define DBG_DISABLE_SPEEDDROP  0   // 1 = dynBase = baseSpeed always (flat speed)

#define DBG_VERBOSE            0   // 1 = print sensors + PID every N ms
#define DBG_VERBOSE_INTERVAL   200 // ms between verbose prints (ignored if DBG_VERBOSE=0)

// =========================================================================
//  PIN ASSIGNMENTS
// =========================================================================

// ─── IR Sensors  (input-only ADC GPIOs) ──────────────────────────────────────
//  S1 = far-left   …   S5 = far-right
extern const int IR_PIN[7];          // { 32, 33, 34, 35, 27, 36, 39  }

// ─── Right Motor  (L298N channel A) ──────────────────────────────────────────
extern const int PIN_ENA;            //  5  – PWM via LEDC channel 0
extern const int PIN_IN1;            // 18
extern const int PIN_IN2;            // 19

// ─── Left Motor  (L298N channel B) ───────────────────────────────────────────
extern const int PIN_ENB;            // 23  – PWM via LEDC channel 1
extern const int PIN_IN3;            // 21
extern const int PIN_IN4;            // 22

// ─── Servo  (single servo drop-gate / gripper) ──────────────────────────────
extern const int PIN_SERVO;          // 13

// ─── HC-SR04 Ultrasonic ─────────────────────────────────────────────────────
//  ⚠  ECHO outputs 5 V – use 1 kΩ / 2 kΩ voltage divider → 3.3 V
extern const int PIN_TRIG;           // 17
extern const int PIN_ECHO;           // 16

// ─── TCS3200 Color Sensor ─────────────────────────────────────────────────────
//  Frequency scale: S0=HIGH, S1=LOW  →  20%
extern const int PIN_CS_S0;          // 14
extern const int PIN_CS_S1;          // 15
extern const int PIN_CS_S2;          // 26
extern const int PIN_CS_S3;          // 25
extern const int PIN_CS_OUT;         //  2  (boot-strapping pin, INPUT is safe)

// ─── LEDC (ESP32 hardware PWM) ─────────────────────────────────────────────────
#define LEDC_FREQ           20000     // 20 kHz carrier (above audible range)
#define LEDC_RESOLUTION        8     // 8-bit → duty 0–255
#define LEDC_CH_ENA            0     // channel 0 → right motor ENA
#define LEDC_CH_ENB            1     // channel 1 → left  motor ENB
//  ESP32Servo uses LEDC timers 2 & 3 (allocated in servo_gate.cpp)

// ─── EEPROM ────────────────────────────────────────────────────────────────────
#define EEPROM_SIZE          512
#define EEPROM_MAGIC        0xAE     // bumped 0xAD→0xAE: pin remap, re-cal required
#define EEPROM_ADDR_MAGIC      0
#define EEPROM_ADDR_CAL        1     // 5 sensors × 3 ints × 4 bytes = 60 B
#define EEPROM_ADDR_PARAMS   100

// ─── Physical motor inversion ──────────────────────────────────────────────────
#define PHYS_RIGHT_INVERT  false
#define PHYS_LEFT_INVERT   false
