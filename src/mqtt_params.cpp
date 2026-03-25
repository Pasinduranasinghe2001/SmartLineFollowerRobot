// =========================================================================
//  mqtt_params.cpp  -  MQTT parameter tuning (non-blocking)
//
//  Library: PubSubClient (knolleary) + ArduinoJson (bblanchon)
//
//  Non-blocking design:
//    _reconnect() tries ONE connect attempt and returns immediately.
//    If the broker is unreachable the robot loop is not affected at all.
//    mqtt_loop() is a no-op when WiFi is disconnected.
//
//  All Params fields from params.h are accessible via SET commands.
//  JSON format: {"key":"KP","value":"18.5"}
//
//  Quick reference (serial monitor or MQTT client):
//    SET KP 18.5          -> P.kp
//    SET KD 5.0           -> P.kd
//    SET FILTER 0.50      -> P.posFilter
//    SET SPEEDDROP 4.0    -> P.speedDrop
//    SET BASE 160         -> P.baseSpeed
//    SET FAST 210         -> P.fastSpeed
//    SET SLOW 120         -> P.slowSpeed
//    SET TURN 130         -> P.turnSpeed
//    SET SHARP 150        -> P.sharpSpeed
//    SET RECOVER 110      -> P.recoverSpeed
//    SET SEARCH 100       -> P.searchSpeed
//    SET REVERSE 90       -> P.reverseSpeed
//    SET REVERSE_BIAS 18  -> P.reverseBiasDelta
//    SET FORWARD_REC 100  -> P.forwardRecoverSpeed
//    SET MINSPEED 40      -> P.minSpeed
//    SET TIMEOUT_L 1800   -> P.timeoutLeft
//    SET TIMEOUT_R 4000   -> P.timeoutRight
//    SET FORWARD_TIME 160 -> P.forwardRecoverTime
//    SET WIDTHKP 6.0      -> P.widthKp
//    SET LTRIM 0          -> P.leftTrim
//    SET RTRIM 0          -> P.rightTrim
//    SET APPSPD 70        -> P.approachSpeed
//    SET AVDSPD 100       -> P.avoidSpeed
//    SET PCKSPD 50        -> P.pickApproachSpeed
//    SET REVTIME 700      -> P.reverseAvoidTime
//    SET FWDTIME 1400     -> P.forwardAvoidTime
//    SET T90TIME 650      -> P.turn90AvoidTime
//    SET SLWDIST 17.0     -> P.obstacleSlowDist
//    SET COLDIST 9.0      -> P.colorCheckDist
//    SET PCKDIST 5.5      -> P.greenPickDist
//    SET REDTHR 120       -> P.redThresh
//    SET GRNTHR 100       -> P.greenThresh
//    SET SVHOME 110       -> P.servoHomeAngle
//    SET SVPICK 1         -> P.servoPickAngle
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

// -------------------------------------------------------------------------
//  _applySet()  -  apply a key/value pair to the Params struct P
//  Returns true if key was recognised.
// -------------------------------------------------------------------------
static bool _applySet(const String& key, const String& val) {
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
    else if (key == "SVHOME")       { P.servoHomeAngle      = val.toInt();   return true; }
    else if (key == "SVPICK")       { P.servoPickAngle      = val.toInt();   return true; }
    return false;
}

// -------------------------------------------------------------------------
//  MQTT message callback
// -------------------------------------------------------------------------
static void _onMessage(char* topic, byte* payload, unsigned int len) {
    String t = String(topic);

    // ── robot/params/save ─────────────────────────────────────────────────
    if (t == MQTT_TOPIC_SAVE) {
        params_saveEEPROM();
        _mqtt.publish(MQTT_TOPIC_ACK, "{\"action\":\"save\",\"ok\":true}");
        return;
    }

    // ── robot/params/load ─────────────────────────────────────────────────
    if (t == MQTT_TOPIC_LOAD) {
        bool ok = params_loadEEPROM();
        if (ok) params_printStatus();
        _mqtt.publish(MQTT_TOPIC_ACK,
            ok ? "{\"action\":\"load\",\"ok\":true}"
               : "{\"action\":\"load\",\"ok\":false,\"reason\":\"no EEPROM data\"}");
        return;
    }

    // ── robot/params/status ───────────────────────────────────────────────
    if (t == MQTT_TOPIC_STATUS) {
        params_printStatus();
        _mqtt.publish(MQTT_TOPIC_ACK, "{\"action\":\"status\",\"ok\":true}");
        return;
    }

    // ── robot/params/set  {"key":"KP","value":"18.5"} ────────────────────
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
        key.toUpperCase();
        key.trim();
        val.trim();

        if (key.length() == 0) {
            _mqtt.publish(MQTT_TOPIC_ACK, "{\"ok\":false,\"reason\":\"missing key\"}");
            return;
        }

        if (_applySet(key, val)) {
            Serial.printf("[MQTT] SET %s = %s\n", key.c_str(), val.c_str());
            // Build ACK: {"key":"KP","value":"18.5","ok":true}
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

// -------------------------------------------------------------------------
//  _reconnect()  -  single non-blocking attempt; returns immediately
// -------------------------------------------------------------------------
static void _reconnect() {
    if (_mqtt.connected()) return;
    Serial.printf("[MQTT] Reconnecting to %s:%d ... ", MQTT_BROKER, MQTT_PORT);
    if (_mqtt.connect(MQTT_CLIENT_ID)) {
        Serial.println("OK");
        _mqtt.subscribe(MQTT_TOPIC_SET);
        _mqtt.subscribe(MQTT_TOPIC_SAVE);
        _mqtt.subscribe(MQTT_TOPIC_LOAD);
        _mqtt.subscribe(MQTT_TOPIC_STATUS);
        // Announce presence
        _mqtt.publish(MQTT_TOPIC_ACK,
            "{\"event\":\"connected\",\"client\":\"" MQTT_CLIENT_ID "\"}");
        params_printStatus();  // dump current params to Serial on connect
    } else {
        Serial.printf("failed (rc=%d) - offline\n", _mqtt.state());
    }
}

// =========================================================================
//  Public API
// =========================================================================
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
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[MQTT] WiFi OK. IP: %s\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\n[MQTT] WiFi timeout - running offline."));
        return;  // broker will be retried inside mqtt_loop
    }

    _mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    _mqtt.setCallback(_onMessage);
    _reconnect();
}

void mqtt_loop() {
#if DBG_DISABLE_MQTT
    return;
#endif
    if (WiFi.status() != WL_CONNECTED) return;  // skip silently when offline
    _reconnect();
    _mqtt.loop();
}

#else   // DBG_DISABLE_MQTT = 1  ->  provide empty stubs so linker is happy
void mqtt_init() {}
void mqtt_loop() {}
#endif
