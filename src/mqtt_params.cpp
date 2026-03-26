// =========================================================================
//  mqtt_params.cpp  -  MQTT parameter tuning (non-blocking)
//
//  Library: PubSubClient (knolleary) + ArduinoJson (bblanchon)
//
//  All Params fields - quick reference:
//    PID:
//      KP  KD  FILTER  SPEEDDROP  WIDTHKP
//      BASE  FAST  SLOW  TURN  SHARP  RECOVER  SEARCH
//      REVERSE  REVERSE_BIAS  FORWARD_REC  MINSPEED
//      TIMEOUT_L  TIMEOUT_R  FORWARD_TIME
//      LTRIM  RTRIM
//    Obstacle / Avoidance:
//      APPSPD  AVDSPD  PCKSPD
//      REVTIME  FWDTIME  T90TIME
//      SLWDIST  COLDIST  PCKDIST
//      REDTHR  GRNTHR
//    Servo:
//      SVHOME  SVPICK
//    Physic 3 - Curvature-Adaptive Speed:
//      CURVTHR    curveDetectThresh  (float, e.g. 0.5)
//      CURVSPD    curveSlowSpeed     (int,   e.g. 100)
//      CURVLOOPS  curveConfirmLoops  (int,   e.g. 4)
//    Physic 4 - Obstacle Side Memory:
//      AVDSIDE    avoidPreferRight   (0=always left, 1=auto)
// =========================================================================
#include <Arduino.h>
#include "config.h"

#if !DBG_DISABLE_MQTT

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "params.h"
#include "mqtt_params.h"

static WiFiClient   _wifiClient;
static PubSubClient _mqtt(_wifiClient);

// ── _applySet() ──────────────────────────────────────────────────────────
static bool _applySet(const String& key, const String& val) {
    // PID / line-follow
    if      (key == "KP")           { P.kp                  = val.toFloat(); return true; }
    else if (key == "KD")           { P.kd                  = val.toFloat(); return true; }
    else if (key == "FILTER")       { P.posFilter           = val.toFloat(); return true; }
    else if (key == "SPEEDDROP")    { P.speedDrop           = val.toFloat(); return true; }
    else if (key == "WIDTHKP")      { P.widthKp             = val.toFloat(); return true; }
    else if (key == "BASE")         { P.baseSpeed           = val.toInt();   return true; }
    else if (key == "FAST")         { P.fastSpeed           = val.toInt();   return true; }
    else if (key == "SLOW")         { P.slowSpeed           = val.toInt();   return true; }
    else if (key == "TURN")         { P.turnSpeed           = val.toInt();   return true; }
    else if (key == "SHARP")        { P.sharpSpeed          = val.toInt();   return true; }
    else if (key == "RECOVER")      { P.recoverSpeed        = val.toInt();   return true; }
    else if (key == "SEARCH")       { P.searchSpeed         = val.toInt();   return true; }
    else if (key == "REVERSE")      { P.reverseSpeed        = val.toInt();   return true; }
    else if (key == "REVERSE_BIAS") { P.reverseBiasDelta    = val.toInt();   return true; }
    else if (key == "FORWARD_REC")  { P.forwardRecoverSpeed = val.toInt();   return true; }
    else if (key == "MINSPEED")     { P.minSpeed            = val.toInt();   return true; }
    else if (key == "TIMEOUT_L")    { P.timeoutLeft         = (unsigned long)val.toInt(); return true; }
    else if (key == "TIMEOUT_R")    { P.timeoutRight        = (unsigned long)val.toInt(); return true; }
    else if (key == "FORWARD_TIME") { P.forwardRecoverTime  = (unsigned long)val.toInt(); return true; }
    else if (key == "LTRIM")        { P.leftTrim            = val.toInt();   return true; }
    else if (key == "RTRIM")        { P.rightTrim           = val.toInt();   return true; }
    // Obstacle / avoidance
    else if (key == "APPSPD")       { P.approachSpeed       = val.toInt();   return true; }
    else if (key == "AVDSPD")       { P.avoidSpeed          = val.toInt();   return true; }
    else if (key == "PCKSPD")       { P.pickApproachSpeed   = val.toInt();   return true; }
    else if (key == "REVTIME")      { P.reverseAvoidTime    = (unsigned long)val.toInt(); return true; }
    else if (key == "FWDTIME")      { P.forwardAvoidTime    = (unsigned long)val.toInt(); return true; }
    else if (key == "T90TIME")      { P.turn90AvoidTime     = (unsigned long)val.toInt(); return true; }
    else if (key == "SLWDIST")      { P.obstacleSlowDist    = val.toFloat(); return true; }
    else if (key == "COLDIST")      { P.colorCheckDist      = val.toFloat(); return true; }
    else if (key == "PCKDIST")      { P.greenPickDist       = val.toFloat(); return true; }
    else if (key == "REDTHR")       { P.redThresh           = val.toInt();   return true; }
    else if (key == "GRNTHR")       { P.greenThresh         = val.toInt();   return true; }
    // Servo
    else if (key == "SVHOME")       { P.servoHomeAngle      = val.toInt();   return true; }
    else if (key == "SVPICK")       { P.servoPickAngle      = val.toInt();   return true; }
    // Physic 3: curvature-adaptive speed
    else if (key == "CURVTHR")      { P.curveDetectThresh   = val.toFloat(); return true; }
    else if (key == "CURVSPD")      { P.curveSlowSpeed      = val.toInt();   return true; }
    else if (key == "CURVLOOPS")    { P.curveConfirmLoops   = val.toInt();   return true; }
    // Physic 4: obstacle side memory
    else if (key == "AVDSIDE")      { P.avoidPreferRight    = val.toInt();   return true; }
    return false;
}

// ── MQTT callback ────────────────────────────────────────────────────────
static void _onMessage(char* topic, byte* payload, unsigned int len) {
    String t = String(topic);

    if (t == MQTT_TOPIC_SAVE) {
        params_saveEEPROM();
        _mqtt.publish(MQTT_TOPIC_ACK, "{\"action\":\"save\",\"ok\":true}");
        return;
    }
    if (t == MQTT_TOPIC_LOAD) {
        bool ok = params_loadEEPROM();
        if (ok) params_printStatus();
        _mqtt.publish(MQTT_TOPIC_ACK,
            ok ? "{\"action\":\"load\",\"ok\":true}"
               : "{\"action\":\"load\",\"ok\":false,\"reason\":\"no EEPROM data\"}");
        return;
    }
    if (t == MQTT_TOPIC_STATUS) {
        params_printStatus();
        _mqtt.publish(MQTT_TOPIC_ACK, "{\"action\":\"status\",\"ok\":true}");
        return;
    }
    if (t == MQTT_TOPIC_SET) {
        StaticJsonDocument<192> doc;
        DeserializationError err = deserializeJson(doc, payload, len);
        if (err) {
            Serial.printf("[MQTT] JSON parse error: %s\n", err.c_str());
            _mqtt.publish(MQTT_TOPIC_ACK, "{\"ok\":false,\"reason\":\"bad JSON\"}");
            return;
        }
        String key = doc["key"]   | "";
        String val = doc["value"] | "";
        key.toUpperCase(); key.trim(); val.trim();
        if (key.length() == 0) {
            _mqtt.publish(MQTT_TOPIC_ACK, "{\"ok\":false,\"reason\":\"missing key\"}");
            return;
        }
        if (_applySet(key, val)) {
            Serial.printf("[MQTT] SET %s = %s\n", key.c_str(), val.c_str());
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"key\":\"%s\",\"value\":\"%s\",\"ok\":true}",
                     key.c_str(), val.c_str());
            _mqtt.publish(MQTT_TOPIC_ACK, ack);
        } else {
            Serial.printf("[MQTT] Unknown key: %s\n", key.c_str());
            char ack[128];
            snprintf(ack, sizeof(ack),
                     "{\"key\":\"%s\",\"ok\":false,\"reason\":\"unknown key\"}",
                     key.c_str());
            _mqtt.publish(MQTT_TOPIC_ACK, ack);
        }
    }
}

// ── Reconnect (single non-blocking attempt) ──────────────────────────────
static void _reconnect() {
    if (_mqtt.connected()) return;
    Serial.printf("[MQTT] Reconnecting to %s:%d ... ", MQTT_BROKER, MQTT_PORT);
    if (_mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println("OK");
        _mqtt.subscribe(MQTT_TOPIC_SET);
        _mqtt.subscribe(MQTT_TOPIC_SAVE);
        _mqtt.subscribe(MQTT_TOPIC_LOAD);
        _mqtt.subscribe(MQTT_TOPIC_STATUS);
        _mqtt.publish(MQTT_TOPIC_ACK,
            "{\"event\":\"connected\",\"client\":\"" MQTT_CLIENT_ID "\"}");
        params_printStatus();
    } else {
        Serial.printf("failed (rc=%d) - offline\n", _mqtt.state());
    }
}

// ── Public API ───────────────────────────────────────────────────────────
void mqtt_init() {
#if DBG_DISABLE_MQTT
    Serial.println(F("[MQTT] Disabled (DBG_DISABLE_MQTT=1)."));
    return;
#endif
    Serial.printf("[MQTT] Connecting WiFi SSID: %s ...\n", MQTT_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(MQTT_WIFI_SSID, MQTT_WIFI_PASS);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000UL) {
        delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[MQTT] WiFi OK. IP: %s\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\n[MQTT] WiFi timeout - running offline."));
        return;
    }
    _mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    _mqtt.setCallback(_onMessage);
    _reconnect();
}

void mqtt_loop() {
#if DBG_DISABLE_MQTT
    return;
#endif
    if (WiFi.status() != WL_CONNECTED) return;
    _reconnect();
    _mqtt.loop();
}

#else
void mqtt_init() {}
void mqtt_loop() {}
#endif
