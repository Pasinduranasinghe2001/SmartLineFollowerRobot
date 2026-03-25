// =========================================================================
//  params.cpp  -  Default values, EEPROM, serial command handler
// =========================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"
#include "config.h"

Params P = {
    // PID / line-follow
    140,    // baseSpeed
    180,    // fastSpeed
    120,    // slowSpeed
    150,    // turnSpeed
    150,    // sharpSpeed
    150,    // recoverSpeed
    150,    // searchSpeed
    180,    // reverseSpeed
    25,     // reverseBiasDelta
    140,    // forwardRecoverSpeed
    150,    // minSpeed

    2000UL, // timeoutLeft  (ms)
    1800UL, // timeoutRight (ms)
    800UL,  // forwardRecoverTime (ms)

    25.0f,  // kp
    9.0f,   // kd
    0.0f,  // posFilter
    6.0f,   // widthKp
    2.0f,   // speedDrop

    0,      // leftTrim
    0,      // rightTrim

    // Physic 3: curvature-adaptive speed
    0.55f,  // curveDetectThresh
    140,    // curveSlowSpeed
    4,      // curveConfirmLoops

    // Physic 4: obstacle side memory
    0,      // avoidPreferRight (0=always left)

    // Obstacle / color / servo
    70,     // approachSpeed
    100,    // avoidSpeed
    50,     // pickApproachSpeed

    500UL,  // reverseAvoidTime
    1400UL, // forwardAvoidTime
    650UL,  // turn90AvoidTime

    17.0f,  // obstacleSlowDist (cm)
    8.0f,   // colorCheckDist   (cm)
    6.0f,   // greenPickDist    (cm)

    120,    // redThresh
    100,    // greenThresh

    50,    // servoHomeAngle  (gate CLOSED = 110 deg)
    120       // servoPickAngle  (gate OPEN   =   1 deg)
};

// ─────────────────────────────────────────────────────────────────────────────
void params_init() { /* defaults already set above */ }

// ── EEPROM ───────────────────────────────────────────────────────────────────
void params_saveEEPROM() {
    EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.put(EEPROM_ADDR_PARAMS, P);
    EEPROM.commit();
    Serial.println(F("[EEPROM] Saved."));
}

bool params_loadEEPROM() {
    if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) {
        Serial.println(F("[EEPROM] Magic mismatch - using firmware defaults."));
        return false;
    }
    EEPROM.get(EEPROM_ADDR_PARAMS, P);
    Serial.println(F("[EEPROM] Loaded."));
    return true;
}

void params_printStatus() {
    Serial.println(F("-------- PID / LINE-FOLLOW --------"));
    Serial.printf("BASE=%d FAST=%d SLOW=%d\n",   P.baseSpeed, P.fastSpeed, P.slowSpeed);
    Serial.printf("TURN=%d SHARP=%d RECOVER=%d SEARCH=%d\n",
                  P.turnSpeed, P.sharpSpeed, P.recoverSpeed, P.searchSpeed);
    Serial.printf("REVERSE=%d BIAS=%d FWD_REC=%d MIN=%d\n",
                  P.reverseSpeed, P.reverseBiasDelta, P.forwardRecoverSpeed, P.minSpeed);
    Serial.printf("TIMEOUT_L=%lu TIMEOUT_R=%lu FWD_TIME=%lu\n",
                  P.timeoutLeft, P.timeoutRight, P.forwardRecoverTime);
    Serial.printf("KP=%.2f KD=%.2f FILTER=%.2f WIDTHKP=%.2f SPEEDDROP=%.2f\n",
                  P.kp, P.kd, P.posFilter, P.widthKp, P.speedDrop);
    Serial.printf("LTRIM=%d RTRIM=%d\n", P.leftTrim, P.rightTrim);

    Serial.println(F("-------- PHYSIC 3: CURVE-ADAPTIVE SPEED --------"));
    Serial.printf("CURVTHR=%.2f CURVSPD=%d CURVLOOPS=%d\n",
                  P.curveDetectThresh, P.curveSlowSpeed, P.curveConfirmLoops);

    Serial.println(F("-------- PHYSIC 4: OBSTACLE SIDE MEMORY --------"));
    Serial.printf("AVDSIDE=%d (%s)\n",
                  P.avoidPreferRight,
                  P.avoidPreferRight ? "auto side-select" : "always LEFT");

    Serial.println(F("-------- OBSTACLE / COLOR / SERVO --------"));
    Serial.printf("APPSPD=%d AVDSPD=%d PCKSPD=%d\n",
                  P.approachSpeed, P.avoidSpeed, P.pickApproachSpeed);
    Serial.printf("REVTIME=%lu FWDTIME=%lu T90TIME=%lu\n",
                  P.reverseAvoidTime, P.forwardAvoidTime, P.turn90AvoidTime);
    Serial.printf("SLWDIST=%.1f COLDIST=%.1f PCKDIST=%.2f\n",
                  P.obstacleSlowDist, P.colorCheckDist, P.greenPickDist);
    Serial.printf("REDTHR=%d GRNTHR=%d\n",  P.redThresh,      P.greenThresh);
    Serial.printf("SVHOME=%d SVPICK=%d\n",  P.servoHomeAngle, P.servoPickAngle);

    Serial.println(F("-------- END-ZONE DETECTOR --------"));
    Serial.printf("ENDZONEMS=%lu  (pattern: 1111111->0000000->1111111, each phase >=%lu ms)\n",
                  P.endZoneHoldMs, P.endZoneHoldMs);

    Serial.println(F("-------- IR SENSORS (MD0370) --------"));
    Serial.printf("Count=7  Digital  activeLevel=%s\n",
#if LINE_ACTIVE_LOW
                  "LOW"
#else
                  "HIGH"
#endif
    );
}

void params_handleSerial() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "STATUS") { params_printStatus(); return; }
    if (cmd == "SAVE")   { params_saveEEPROM();  return; }
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
        if      (key == "BASE")         P.baseSpeed           = val.toInt();
        else if (key == "FAST")         P.fastSpeed           = val.toInt();
        else if (key == "SLOW")         P.slowSpeed           = val.toInt();
        else if (key == "TURN")         P.turnSpeed           = val.toInt();
        else if (key == "SHARP")        P.sharpSpeed          = val.toInt();
        else if (key == "RECOVER")      P.recoverSpeed        = val.toInt();
        else if (key == "SEARCH")       P.searchSpeed         = val.toInt();
        else if (key == "REVERSE")      P.reverseSpeed        = val.toInt();
        else if (key == "REVERSE_BIAS") P.reverseBiasDelta    = val.toInt();
        else if (key == "FORWARD_REC")  P.forwardRecoverSpeed = val.toInt();
        else if (key == "MINSPEED")     P.minSpeed            = val.toInt();
        else if (key == "TIMEOUT_L")    P.timeoutLeft         = (unsigned long)val.toInt();
        else if (key == "TIMEOUT_R")    P.timeoutRight        = (unsigned long)val.toInt();
        else if (key == "FORWARD_TIME") P.forwardRecoverTime  = (unsigned long)val.toInt();
        else if (key == "KP")           P.kp                  = val.toFloat();
        else if (key == "KD")           P.kd                  = val.toFloat();
        else if (key == "FILTER")       P.posFilter           = val.toFloat();
        else if (key == "WIDTHKP")      P.widthKp             = val.toFloat();
        else if (key == "SPEEDDROP")    P.speedDrop           = val.toFloat();
        else if (key == "LTRIM")        P.leftTrim            = val.toInt();
        else if (key == "RTRIM")        P.rightTrim           = val.toInt();
        else if (key == "CURVTHR")      P.curveDetectThresh   = val.toFloat();
        else if (key == "CURVSPD")      P.curveSlowSpeed      = val.toInt();
        else if (key == "CURVLOOPS")    P.curveConfirmLoops   = val.toInt();
        else if (key == "AVDSIDE")      P.avoidPreferRight    = val.toInt();
        else if (key == "APPSPD")       P.approachSpeed       = val.toInt();
        else if (key == "AVDSPD")       P.avoidSpeed          = val.toInt();
        else if (key == "PCKSPD")       P.pickApproachSpeed   = val.toInt();
        else if (key == "REVTIME")      P.reverseAvoidTime    = (unsigned long)val.toInt();
        else if (key == "FWDTIME")      P.forwardAvoidTime    = (unsigned long)val.toInt();
        else if (key == "T90TIME")      P.turn90AvoidTime     = (unsigned long)val.toInt();
        else if (key == "SLWDIST")      P.obstacleSlowDist    = val.toFloat();
        else if (key == "COLDIST")      P.colorCheckDist      = val.toFloat();
        else if (key == "PCKDIST")      P.greenPickDist       = val.toFloat();
        else if (key == "REDTHR")       P.redThresh           = val.toInt();
        else if (key == "GRNTHR")       P.greenThresh         = val.toInt();
        else if (key == "SVHOME")       P.servoHomeAngle      = val.toInt();
        else if (key == "SVPICK")       P.servoPickAngle      = val.toInt();
        else if (key == "ENDZONEMS")    P.endZoneHoldMs       = (unsigned long)val.toInt();
        else found = false;

        if (found) Serial.printf("[SET] %s = %s\n", key.c_str(), val.c_str());
        else       Serial.printf("[SET] Unknown key: %s\n", key.c_str());
        return;
    }

    Serial.println(F("Commands: STATUS | SAVE | LOAD | SET KEY VALUE"));
}
