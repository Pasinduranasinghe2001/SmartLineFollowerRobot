// =========================================================================
//  params.cpp  –  Default values, EEPROM, serial command handler
// =========================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"
#include "config.h"
#include "sensors.h"    // for calBlack / calWhite / calThresh arrays

// ── Default parameter values ──────────────────────────────────────────────
Params P = {
    // PID / line-follow
    160,    // baseSpeed
    210,    // fastSpeed
    120,     // slowSpeed
    130,    // turnSpeed
    150,    // sharpSpeed
    110,    // recoverSpeed
    100,    // searchSpeed
    90,     // reverseSpeed
    18,     // reverseBiasDelta
    60,     // forwardRecoverSpeed
    40,     // minSpeed

    1800UL, // timeoutLeft  (ms)
    4000UL, // timeoutRight (ms)
    160UL,  // forwardRecoverTime (ms)

    16.0f,  // kp
    10.0f,  // kd
    0.65f,  // posFilter
    6.0f,   // widthKp

    8,      // leftTrim
    0,      // rightTrim

    // Obstacle / color / servo
    70,     // approachSpeed
    100,    // avoidSpeed
    50,     // pickApproachSpeed

    700UL,  // reverseAvoidTime  (~7 cm)
    1400UL, // forwardAvoidTime  (~15 cm)
    650UL,  // turn90AvoidTime   ← tune on your floor first

    17.0f,  // obstacleSlowDist (cm)
    9.0f,   // colorCheckDist   (cm)
    5.5f,   // greenPickDist    (cm)

    120,    // redThresh   (smaller = brighter red response)
    100,    // greenThresh

    109,    // servoHomeAngle  (gate CLOSED)
    183     // servoPickAngle  (gate OPEN)
};

// ─────────────────────────────────────────────────────────────────────────
void params_init() { /* nothing – defaults already set above */ }

// ── EEPROM ────────────────────────────────────────────────────────────────
void params_saveEEPROM() {
    EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    int addr = EEPROM_ADDR_CAL;
    for (int i = 0; i < 5; i++) {
        EEPROM.put(addr, calBlack[i]);  addr += sizeof(int);
        EEPROM.put(addr, calWhite[i]);  addr += sizeof(int);
        EEPROM.put(addr, calThresh[i]); addr += sizeof(int);
    }
    EEPROM.put(EEPROM_ADDR_PARAMS, P);
    EEPROM.commit();   // required on ESP32
    Serial.println(F("[EEPROM] Saved."));
}

bool params_loadEEPROM() {
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) return false;
    int addr = EEPROM_ADDR_CAL;
    for (int i = 0; i < 5; i++) {
        EEPROM.get(addr, calBlack[i]);  addr += sizeof(int);
        EEPROM.get(addr, calWhite[i]);  addr += sizeof(int);
        EEPROM.get(addr, calThresh[i]); addr += sizeof(int);
    }
    EEPROM.get(EEPROM_ADDR_PARAMS, P);
    Serial.println(F("[EEPROM] Loaded."));
    return true;
}

// ── Status print ──────────────────────────────────────────────────────────
void params_printStatus() {
    Serial.println(F("──────── PID / LINE-FOLLOW ────────"));
    Serial.printf("BASE=%d FAST=%d SLOW=%d\n", P.baseSpeed, P.fastSpeed, P.slowSpeed);
    Serial.printf("TURN=%d SHARP=%d RECOVER=%d SEARCH=%d\n",
                  P.turnSpeed, P.sharpSpeed, P.recoverSpeed, P.searchSpeed);
    Serial.printf("REVERSE=%d REVERSE_BIAS=%d FORWARD_REC=%d MINSPEED=%d\n",
                  P.reverseSpeed, P.reverseBiasDelta, P.forwardRecoverSpeed, P.minSpeed);
    Serial.printf("TIMEOUT_L=%lu TIMEOUT_R=%lu FORWARD_TIME=%lu\n",
                  P.timeoutLeft, P.timeoutRight, P.forwardRecoverTime);
    Serial.printf("KP=%.2f KD=%.2f FILTER=%.2f WIDTHKP=%.2f\n",
                  P.kp, P.kd, P.posFilter, P.widthKp);
    Serial.printf("LTRIM=%d RTRIM=%d\n", P.leftTrim, P.rightTrim);

    Serial.println(F("──────── OBSTACLE / COLOR / SERVO ────────"));
    Serial.printf("APPSPD=%d AVDSPD=%d PCKSPD=%d\n",
                  P.approachSpeed, P.avoidSpeed, P.pickApproachSpeed);
    Serial.printf("REVTIME=%lu FWDTIME=%lu T90TIME=%lu\n",
                  P.reverseAvoidTime, P.forwardAvoidTime, P.turn90AvoidTime);
    Serial.printf("SLWDIST=%.1f COLDIST=%.1f PCKDIST=%.2f\n",
                  P.obstacleSlowDist, P.colorCheckDist, P.greenPickDist);
    Serial.printf("REDTHR=%d GRNTHR=%d\n", P.redThresh, P.greenThresh);
    Serial.printf("SVHOME=%d SVPICK=%d\n", P.servoHomeAngle, P.servoPickAngle);
}

// ── Serial command handler ────────────────────────────────────────────────
void params_handleSerial() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "CALIBRATE") { extern void sensors_calibrate(); sensors_calibrate(); return; }
    if (cmd == "STATUS")    { params_printStatus(); return; }
    if (cmd == "SAVE")      { params_saveEEPROM();  return; }
    if (cmd == "LOAD") {
        if (params_loadEEPROM()) params_printStatus();
        else Serial.println(F("[EEPROM] No saved data."));
        return;
    }

    if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println(F("Usage: SET KEY VALUE")); return; }
        String key = cmd.substring(4, sp);
        String val = cmd.substring(sp + 1);
        val.trim();

        bool found = true;
        if      (key == "BASE")          P.baseSpeed           = val.toInt();
        else if (key == "FAST")          P.fastSpeed           = val.toInt();
        else if (key == "SLOW")          P.slowSpeed           = val.toInt();
        else if (key == "TURN")          P.turnSpeed           = val.toInt();
        else if (key == "SHARP")         P.sharpSpeed          = val.toInt();
        else if (key == "RECOVER")       P.recoverSpeed        = val.toInt();
        else if (key == "SEARCH")        P.searchSpeed         = val.toInt();
        else if (key == "REVERSE")       P.reverseSpeed        = val.toInt();
        else if (key == "REVERSE_BIAS")  P.reverseBiasDelta    = val.toInt();
        else if (key == "FORWARD_REC")   P.forwardRecoverSpeed = val.toInt();
        else if (key == "MINSPEED")      P.minSpeed            = val.toInt();
        else if (key == "TIMEOUT_L")     P.timeoutLeft         = (unsigned long)val.toInt();
        else if (key == "TIMEOUT_R")     P.timeoutRight        = (unsigned long)val.toInt();
        else if (key == "FORWARD_TIME")  P.forwardRecoverTime  = (unsigned long)val.toInt();
        else if (key == "KP")            P.kp                  = val.toFloat();
        else if (key == "KD")            P.kd                  = val.toFloat();
        else if (key == "FILTER")        P.posFilter           = val.toFloat();
        else if (key == "WIDTHKP")       P.widthKp             = val.toFloat();
        else if (key == "LTRIM")         P.leftTrim            = val.toInt();
        else if (key == "RTRIM")         P.rightTrim           = val.toInt();
        else if (key == "APPSPD")        P.approachSpeed       = val.toInt();
        else if (key == "AVDSPD")        P.avoidSpeed          = val.toInt();
        else if (key == "PCKSPD")        P.pickApproachSpeed   = val.toInt();
        else if (key == "REVTIME")       P.reverseAvoidTime    = (unsigned long)val.toInt();
        else if (key == "FWDTIME")       P.forwardAvoidTime    = (unsigned long)val.toInt();
        else if (key == "T90TIME")       P.turn90AvoidTime     = (unsigned long)val.toInt();
        else if (key == "SLWDIST")       P.obstacleSlowDist    = val.toFloat();
        else if (key == "COLDIST")       P.colorCheckDist      = val.toFloat();
        else if (key == "PCKDIST")       P.greenPickDist       = val.toFloat();
        else if (key == "REDTHR")        P.redThresh           = val.toInt();
        else if (key == "GRNTHR")        P.greenThresh         = val.toInt();
        else if (key == "SVHOME")        P.servoHomeAngle      = val.toInt();
        else if (key == "SVPICK")        P.servoPickAngle      = val.toInt();
        else found = false;

        if (found) { Serial.printf("[SET] %s = %s\n", key.c_str(), val.c_str()); }
        else       { Serial.printf("Unknown key: %s\n", key.c_str()); }
        return;
    }

    Serial.println(F("Commands: CALIBRATE | STATUS | SAVE | LOAD | SET KEY VALUE"));
}
