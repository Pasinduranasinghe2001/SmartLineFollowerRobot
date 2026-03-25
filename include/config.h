// =========================================================================
//  config.h  -  Pin assignments, LEDC, EEPROM constants
//               + compile-time DEBUG FLAGS
//               + MQTT Wi-Fi configuration
//
//  EC6090 Mini-Project  |  ESP32 Dev Module (38-pin, WROOM-32)
//
//  FINAL TESTED PIN MAP
//  -------------------------------------------------------------------------
//  MOTOR DRIVER (L298N)
//    ENA -> GPIO5    IN1 -> GPIO18   IN2 -> GPIO19
//    ENB -> GPIO23   IN3 -> GPIO21   IN4 -> GPIO22
//
//  IR SENSORS (7-channel MD0370 digital)
//    IR0 -> GPIO32   IR1 -> GPIO33   IR2 -> GPIO34
//    IR3 -> GPIO35   IR4 -> GPIO27   IR5 -> GPIO36   IR6 -> GPIO39
//    GPIO 34,35,36,39 are INPUT-ONLY (no OUTPUT, no internal pull-up/down)
//
//  COLOR SENSOR (TCS3200)
//    S0 -> GPIO14   S1 -> GPIO15   S2 -> GPIO26
//    S3 -> GPIO25   OUT -> GPIO2
//    GPIO 2 (TCS OUT) is a boot-strapping pin. As an INPUT it is safe.
//
//  ULTRASONIC (HC-SR04)
//    TRIG -> GPIO17   ECHO -> GPIO16
//    ECHO outputs 5V - use 1kOhm + 2kOhm voltage divider before GPIO16
//
//  SERVO
//    Signal -> GPIO13
//
//  All pins confirmed accessible on the 38-pin ESP32 Dev Module board.
// =========================================================================
#pragma once
#include <Arduino.h>

// =========================================================================
//  DEBUG FLAGS
//  -------------------------------------------------------------------------
//  Set a flag to 1 to DISABLE that subsystem during a debug/test session.
//  Set it back to 0 for competition / production.
// =========================================================================

#define DBG_DISABLE_OBSTACLE   0   // 1 = robot ignores ultrasonic, PID only
#define DBG_DISABLE_COLOR      0   // 1 = skip TCS3200 colour read entirely
#define DBG_DISABLE_SERVO      0   // 1 = servo_open/close calls are no-ops
#define DBG_DISABLE_ENDZONE    0   // 1 = never trigger end-zone gate drop
#define DBG_DISABLE_RECOVERY   0   // 1 = replace recovery with motors_stop()
#define DBG_DISABLE_SPEEDDROP  0   // 1 = dynBase = baseSpeed always (flat speed)

#define DBG_VERBOSE            0   // 1 = print sensors + PID every N ms
#define DBG_VERBOSE_INTERVAL   200 // ms between verbose prints

// =========================================================================
//  MQTT / Wi-Fi CONFIGURATION
//  -------------------------------------------------------------------------
//  DBG_DISABLE_MQTT = 0  ->  Wi-Fi + MQTT enabled (default)
//  DBG_DISABLE_MQTT = 1  ->  Wi-Fi completely skipped, robot runs offline
//                            (use for competition / no router available)
//
//  MQTT_BROKER  - IP address of your MQTT broker on the local network
//                 Options:
//                   Mosquitto on laptop : your laptop IP (e.g. 192.168.1.100)
//                   Mosquitto on Pi     : Pi IP address
//                   Cloud broker (test) : "broker.hivemq.com" port 1883
//
//  Topics:
//    robot/params/set    <- publish JSON  {"key":"KP","value":"18.5"}
//    robot/params/save   <- publish any payload to save EEPROM
//    robot/params/load   <- publish any payload to load EEPROM
//    robot/params/status <- publish any payload to print current params
//    robot/params/ack    -> ESP32 publishes ack after each SET
// =========================================================================

#define DBG_DISABLE_MQTT       0          // 1 = skip WiFi entirely

#define MQTT_WIFI_SSID         "Dialog 4G 720"
#define MQTT_WIFI_PASS         "82601557"
#define MQTT_BROKER            "192.168.1.100"  // broker IP on your LAN
#define MQTT_PORT              1883
#define MQTT_CLIENT_ID         "ESP32Robot"

#define MQTT_TOPIC_SET         "robot/params/set"
#define MQTT_TOPIC_SAVE        "robot/params/save"
#define MQTT_TOPIC_LOAD        "robot/params/load"
#define MQTT_TOPIC_STATUS      "robot/params/status"
#define MQTT_TOPIC_ACK         "robot/params/ack"

// =========================================================================
//  PIN ASSIGNMENTS
// =========================================================================

// IR Sensors (input-only digital GPIOs, MD0370)
//  S0=far-left ... S6=far-right
extern const int IR_PIN[7];          // { 32, 33, 34, 35, 27, 36, 39 }

// Right Motor (L298N channel A)
extern const int PIN_ENA;            //  5  - PWM via LEDC channel 0
extern const int PIN_IN1;            // 18
extern const int PIN_IN2;            // 19

// Left Motor (L298N channel B)
extern const int PIN_ENB;            // 23  - PWM via LEDC channel 1
extern const int PIN_IN3;            // 21
extern const int PIN_IN4;            // 22

// Servo (single servo drop-gate / gripper)
extern const int PIN_SERVO;          // 13

// HC-SR04 Ultrasonic
//  ECHO outputs 5V - use 1kOhm / 2kOhm voltage divider -> 3.3V
extern const int PIN_TRIG;           // 17
extern const int PIN_ECHO;           // 16

// TCS3200 Color Sensor
extern const int PIN_CS_S0;          // 14
extern const int PIN_CS_S1;          // 15
extern const int PIN_CS_S2;          // 26
extern const int PIN_CS_S3;          // 25
extern const int PIN_CS_OUT;         //  2

// LEDC (ESP32 hardware PWM)
#define LEDC_FREQ           20000    // 20 kHz carrier
#define LEDC_RESOLUTION         8   // 8-bit duty 0-255
#define LEDC_CH_ENA             0   // channel 0 -> right motor ENA
#define LEDC_CH_ENB             1   // channel 1 -> left  motor ENB

// EEPROM
#define EEPROM_SIZE          512
#define EEPROM_MAGIC        0xAE
#define EEPROM_ADDR_MAGIC      0
#define EEPROM_ADDR_CAL        1
#define EEPROM_ADDR_PARAMS   100

// Physical motor inversion
#define PHYS_RIGHT_INVERT  false
#define PHYS_LEFT_INVERT   false
