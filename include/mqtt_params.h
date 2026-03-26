// =========================================================================
//  mqtt_params.h  -  MQTT-based parameter tuning over Wi-Fi
//
//  Subscribes to:
//    robot/params/set    <- {"key":"KP","value":"18.5"}
//    robot/params/save   <- any payload -> params_saveEEPROM()
//    robot/params/load   <- any payload -> params_loadEEPROM()
//    robot/params/status <- any payload -> params_printStatus()
//
//  Publishes to:
//    robot/params/ack    -> {"key":"KP","value":"18.5","ok":true}
//
//  Usage:
//    mqtt_init()  - call once in setup()  (after params_init)
//    mqtt_loop()  - call once per loop()  (non-blocking)
//
//  If DBG_DISABLE_MQTT = 1 in config.h, both functions are no-ops.
// =========================================================================
#pragma once

void mqtt_init();
void mqtt_loop();
