// =========================================================================
//  test/line_follow_7ir/main.cpp
//  STANDALONE 7-IR LINE FOLLOWER TEST
//  -------------------------------------------------------------------------
//  Purpose:
//    Verify that the two newly added outer IR sensors (S6, S7) integrate
//    correctly with the existing 5-sensor PID before merging into main.
//
//  What this sketch tests:
//    ─ All 7 ADC reads are valid and calibrate correctly
//    ─ Weighted-average position spans −6 … +6 (was −4 … +4 with 5 sensors)
//    ─ PID corrections are proportional across the wider sensor array
//    ─ Lost-line recovery uses the two new outer sensors to detect
//      sharper turns earlier
//    ─ EEPROM saves / loads 7-sensor cal data correctly
//    ─ Serial STATUS shows all 7 sensor values
//
//  Hardware additions vs. main project:
//    S6 → GPIO36 (input-only, no pull, VP pin)
//    S7 → GPIO39 (input-only, no pull, VN pin)
//    Both are ADC1 channels, safe for analogRead on ESP32.
//
//  Pin summary (all 7):
//    S1=GPIO32  S2=GPIO33  S3=GPIO34  S4=GPIO35  S5=GPIO27
//    S6=GPIO36  S7=GPIO39
//
//  Motors, LEDC, EEPROM, Serial commands: identical to main project.
//  This is a SELF-CONTAINED file — does NOT include src/ files.
//  To run: select env:test_7ir in platformio.ini and Upload.
// =========================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

// ───────────────────────────────────────────────────────────────────────────
//  CONFIGURATION  — edit these to match your wiring
// ───────────────────────────────────────────────────────────────────────────

// 7 IR sensor GPIO pins  (left → right physical order)
// S1–S5  are the existing sensors from the main project
// S6, S7 are the two newly added outer sensors
static const int IR_PIN[7] = { 32, 33, 34, 35, 27, 36, 39 };
//                              S1  S2  S3  S4  S5  S6  S7
//                         far-left               far-right

// Sensor weights for weighted-average PID position
// Symmetric around centre (S4 = 0).  Span: -6 … +6
// With 5 sensors span was -4 … +4; outer pair extends detection range.
static const int WEIGHT[7] = { -6, -4, -2, 0, 2, 4, 6 };
//                               S1  S2  S3 S4 S5 S6 S7

// ── Motor pins (L298N) ───────────────────────────────────────────────────────
static const int PIN_ENA = 5;
static const int PIN_IN1 = 18;
static const int PIN_IN2 = 19;
static const int PIN_ENB = 23;
static const int PIN_IN3 = 21;
static const int PIN_IN4 = 22;

// ── LEDC PWM ─────────────────────────────────────────────────────────────────────
#define LEDC_FREQ       20000
#define LEDC_RES        8
#define LEDC_CH_R       0
#define LEDC_CH_L       1

// ── EEPROM ───────────────────────────────────────────────────────────────────────
#define EEPROM_SZ       512
#define MAGIC_BYTE      0xB1   // unique magic: won't clash with main project 0xAE
#define ADDR_MAGIC      0
#define ADDR_CAL        4      // 7 sensors x 3 ints x 4 B = 84 bytes
#define ADDR_PARAMS     100

// ───────────────────────────────────────────────────────────────────────────
//  TUNABLE PARAMETERS
//  All settable live: SET BASE 160  etc.
// ───────────────────────────────────────────────────────────────────────────
struct Params {
    int   baseSpeed;          // 160
    int   minSpeed;           //  40   inner-wheel floor
    int   searchSpeed;        // 100   pivot speed in recovery
    int   reverseSpeed;       //  90   reverse in recovery
    int   forwardRecSpeed;    // 100   creep forward in recovery
    unsigned long timeoutMs;  // 2000  max recovery pivot time (ms)
    float kp;                 //  16.0
    float kd;                 //   5.0
    float posFilter;          //   0.50
    float speedDrop;          //   4.0  PWM per unit |error|
    int   leftTrim;           //   0
    int   rightTrim;          //   0
};

Params P = {
    160, 40, 100, 90, 100, 2000UL,
    16.0f, 5.0f, 0.50f, 4.0f,
    0, 0
};

// ───────────────────────────────────────────────────────────────────────────
//  SENSOR STATE
// ───────────────────────────────────────────────────────────────────────────
int  calBlack[7], calWhite[7], calThresh[7];
int  irRaw[7], irStrength[7];
bool irOn[7];

// ───────────────────────────────────────────────────────────────────────────
//  PID STATE
// ───────────────────────────────────────────────────────────────────────────
static float _filteredPos = 0.0f;
static float _lastError   = 0.0f;
static float _lastPidPos  = 0.0f;

// ───────────────────────────────────────────────────────────────────────────
//  RECOVERY STATE
// ───────────────────────────────────────────────────────────────────────────
enum SeenSide  { SIDE_UNKNOWN, SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT };
enum RecovMode { REC_IDLE, REC_REVERSE, REC_PIVOT };

SeenSide  lastSeen  = SIDE_UNKNOWN;
RecovMode recovMode = REC_IDLE;
unsigned long recovStart = 0;

// =========================================================================
//  MOTOR HELPERS  (BUG-3 fix: zero PWM before direction change)
// =========================================================================
static void _pwmR(int v) { ledcWrite(LEDC_CH_R, constrain(v, 0, 255)); }
static void _pwmL(int v) { ledcWrite(LEDC_CH_L, constrain(v, 0, 255)); }

static void rightFwd(int s) {
    s = constrain(s + P.rightTrim, 0, 255);
    _pwmR(0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
    _pwmR(s);
}
static void rightBwd(int s) {
    s = constrain(s + P.rightTrim, 0, 255);
    _pwmR(0);
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    _pwmR(s);
}
static void leftFwd(int s) {
    s = constrain(s + P.leftTrim, 0, 255);
    _pwmL(0);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
    _pwmL(s);
}
static void leftBwd(int s) {
    s = constrain(s + P.leftTrim, 0, 255);
    _pwmL(0);
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    _pwmL(s);
}
static void motorsStop() {
    ledcWrite(LEDC_CH_R, 0); ledcWrite(LEDC_CH_L, 0);
    digitalWrite(PIN_ENA, LOW); digitalWrite(PIN_ENB, LOW);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}
static void driveStraight(int s) { rightFwd(s); leftFwd(s); }
static void driveBack(int s)     { rightBwd(s); leftBwd(s); }
static void pivotLeft(int s)     { rightFwd(s); leftBwd(s); }
static void pivotRight(int s)    { rightBwd(s); leftFwd(s); }

// =========================================================================
//  SENSOR HELPERS
// =========================================================================
static void readSensors() {
    for (int i = 0; i < 7; i++) {
        int r = analogRead(IR_PIN[i]);
        irRaw[i] = r;
        irOn[i]  = (r <= calThresh[i]);
        int bRef = calBlack[i];
        int wRef = calWhite[i];
        if (bRef <= wRef + 20) bRef = wRef + 200;
        irStrength[i] = (int)constrain(map(r, bRef, wRef, 0, 1000), 0, 1000);
    }
}

static bool allDark() {
    for (int i = 0; i < 7; i++) if (irOn[i]) return false;
    return true;
}
static bool anyOn() {
    for (int i = 0; i < 7; i++) if (irOn[i]) return true;
    return false;
}

// Bit pattern: bit6=S1(far-left) … bit0=S7(far-right)
static int sensorBits() {
    int b = 0;
    for (int i = 0; i < 7; i++) if (irOn[i]) b |= (1 << (6 - i));
    return b;
}

// Centre found — at least one of the middle three sensors is on
static bool centreFound() {
    return irOn[2] || irOn[3] || irOn[4];
}

// Outer-left sensors S1 or S2 — sharp left turn
static bool sharpLeft() { return irOn[0] || irOn[1]; }

// Outer-right sensors S6 or S7 — sharp right turn  (NEW sensors)
static bool sharpRight() { return irOn[5] || irOn[6]; }

static void updateLastSeen() {
    // Outer pair (new sensors) override first — strongest signal
    if (irOn[0] || irOn[1]) lastSeen = SIDE_LEFT;
    if (irOn[5] || irOn[6]) lastSeen = SIDE_RIGHT;    // S6, S7
    // Inner pair refines if outers are off
    if (!irOn[0] && !irOn[1] && !irOn[5] && !irOn[6]) {
        if (irOn[3])  lastSeen = SIDE_CENTER;
    }
}

// =========================================================================
//  WEIGHTED-AVERAGE PID POSITION
//  Range: -6 (far-left) … +6 (far-right)
//  NOTE: the wider span means kp / kd corrections are proportionally
//  larger on the outer sensors — you may need to reduce kp slightly
//  compared to the 5-sensor setup (start with kp=12, kd=4).
// =========================================================================
static float computePos() {
    long wsum = 0, total = 0;
    for (int i = 0; i < 7; i++) {
        wsum  += (long)WEIGHT[i] * (long)irStrength[i];
        total += (long)irStrength[i];
    }
    if (total < 50) return _lastPidPos;
    _lastPidPos = (float)wsum / (float)total;
    return _lastPidPos;
}

// =========================================================================
//  EEPROM  (7-sensor layout, magic 0xB1)
// =========================================================================
static void saveEEPROM() {
    EEPROM.write(ADDR_MAGIC, MAGIC_BYTE);
    int addr = ADDR_CAL;
    for (int i = 0; i < 7; i++) {
        EEPROM.put(addr, calBlack[i]);  addr += sizeof(int);
        EEPROM.put(addr, calWhite[i]);  addr += sizeof(int);
        EEPROM.put(addr, calThresh[i]); addr += sizeof(int);
    }
    EEPROM.put(ADDR_PARAMS, P);
    EEPROM.commit();
    Serial.println(F("[EEPROM] Saved (7-sensor)."));
}

static bool loadEEPROM() {
    if (EEPROM.read(ADDR_MAGIC) != MAGIC_BYTE) return false;
    int addr = ADDR_CAL;
    for (int i = 0; i < 7; i++) {
        EEPROM.get(addr, calBlack[i]);  addr += sizeof(int);
        EEPROM.get(addr, calWhite[i]);  addr += sizeof(int);
        EEPROM.get(addr, calThresh[i]); addr += sizeof(int);
    }
    EEPROM.get(ADDR_PARAMS, P);
    Serial.println(F("[EEPROM] Loaded (7-sensor)."));
    return true;
}

// =========================================================================
//  CALIBRATION  (interactive, 2-phase)
// =========================================================================
static void calibrate() {
    Serial.println(F("\n=== 7-IR CALIBRATION ==="));
    Serial.println(F("PHASE 1: Place ALL 7 sensors over FLOOR. Send any key..."));
    while (!Serial.available()) delay(50);
    while ( Serial.available()) Serial.read();

    long accB[7] = {};
    for (int r = 0; r < 80; r++) {
        for (int i = 0; i < 7; i++) accB[i] += analogRead(IR_PIN[i]);
        delay(10);
    }
    for (int i = 0; i < 7; i++) calBlack[i] = (int)(accB[i] / 80);

    Serial.println(F("PHASE 2: Place ALL 7 sensors over the LINE. Send any key..."));
    while (!Serial.available()) delay(50);
    while ( Serial.available()) Serial.read();

    long accW[7] = {};
    for (int r = 0; r < 80; r++) {
        for (int i = 0; i < 7; i++) accW[i] += analogRead(IR_PIN[i]);
        delay(10);
    }
    for (int i = 0; i < 7; i++) {
        calWhite[i]  = (int)(accW[i] / 80);
        calThresh[i] = (calBlack[i] + calWhite[i]) / 2;
    }

    Serial.println(F("=== CALIBRATION DONE ==="));
    for (int i = 0; i < 7; i++) {
        Serial.printf("  S%d  Floor=%4d  Line=%4d  Thresh=%4d  GPIO=%d\n",
                      i+1, calBlack[i], calWhite[i], calThresh[i], IR_PIN[i]);
    }
    Serial.println(F("Send SAVE to persist."));
}

// =========================================================================
//  STATUS PRINT
// =========================================================================
static void printStatus() {
    readSensors();
    Serial.println(F("\n────── 7-IR TEST – STATUS ──────"));
    Serial.printf("BASE=%d MIN=%d SEARCH=%d REVERSE=%d FWDREC=%d\n",
                  P.baseSpeed, P.minSpeed, P.searchSpeed,
                  P.reverseSpeed, P.forwardRecSpeed);
    Serial.printf("KP=%.2f KD=%.2f FILTER=%.2f SPEEDDROP=%.2f\n",
                  P.kp, P.kd, P.posFilter, P.speedDrop);
    Serial.printf("LTRIM=%d RTRIM=%d  TIMEOUT=%lu ms\n",
                  P.leftTrim, P.rightTrim, P.timeoutMs);

    Serial.println(F("── Live sensor readings ──"));
    for (int i = 0; i < 7; i++) {
        Serial.printf("  S%d(GPIO%2d): raw=%4d  str=%4d  on=%d  |  cal B=%4d W=%4d T=%4d\n",
                      i+1, IR_PIN[i], irRaw[i], irStrength[i], irOn[i] ? 1 : 0,
                      calBlack[i], calWhite[i], calThresh[i]);
    }

    // Visual bar of which sensors see the line
    Serial.print(F("  Bitmap: "));
    for (int i = 0; i < 7; i++) Serial.print(irOn[i] ? "X" : ".");
    Serial.println();

    float pos = computePos();
    Serial.printf("  PID position = %.2f  (range -6 to +6)\n", pos);
}

// =========================================================================
//  SERIAL COMMAND HANDLER
// =========================================================================
static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "CALIBRATE") { calibrate();   return; }
    if (cmd == "STATUS")    { printStatus();  return; }
    if (cmd == "SAVE")      { saveEEPROM();   return; }
    if (cmd == "LOAD") {
        if (loadEEPROM()) printStatus();
        else Serial.println(F("[EEPROM] No 7-IR data found."));
        return;
    }

    if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println(F("Usage: SET KEY VALUE")); return; }
        String key = cmd.substring(4, sp);
        String val = cmd.substring(sp + 1); val.trim();
        bool ok = true;
        if      (key == "BASE")       P.baseSpeed       = val.toInt();
        else if (key == "MIN")        P.minSpeed        = val.toInt();
        else if (key == "SEARCH")     P.searchSpeed     = val.toInt();
        else if (key == "REVERSE")    P.reverseSpeed    = val.toInt();
        else if (key == "FWDREC")     P.forwardRecSpeed = val.toInt();
        else if (key == "TIMEOUT")    P.timeoutMs       = (unsigned long)val.toInt();
        else if (key == "KP")         P.kp              = val.toFloat();
        else if (key == "KD")         P.kd              = val.toFloat();
        else if (key == "FILTER")     P.posFilter       = val.toFloat();
        else if (key == "SPEEDDROP")  P.speedDrop       = val.toFloat();
        else if (key == "LTRIM")      P.leftTrim        = val.toInt();
        else if (key == "RTRIM")      P.rightTrim       = val.toInt();
        else ok = false;
        if (ok) Serial.printf("[SET] %s = %s\n", key.c_str(), val.c_str());
        else    Serial.printf("Unknown key: %s\n", key.c_str());
        return;
    }

    if (cmd == "HELP") {
        Serial.println(F("Commands:"));
        Serial.println(F("  CALIBRATE  – 2-phase floor/line calibration"));
        Serial.println(F("  STATUS     – print params + live 7-sensor readings"));
        Serial.println(F("  SAVE / LOAD"));
        Serial.println(F("  SET BASE 160 | SET KP 12 | SET KD 4 | SET SPEEDDROP 4"));
        Serial.println(F("  SET LTRIM n  | SET RTRIM n  (motor balance)"));
        Serial.println(F("  SET FILTER 0.50 | SET TIMEOUT 2000"));
        return;
    }

    Serial.println(F("? – type HELP for command list"));
}

// =========================================================================
//  PID LINE FOLLOW  (7-sensor version)
//  Identical algorithm to main project; only array size and weight span differ.
//  The wider span (-6…+6 vs -4…+4) means the PID "sees" the line deviating
//  earlier, allowing earlier correction and smoother cornering.
// =========================================================================
static void followLine() {
    float raw   = computePos();
    _filteredPos = P.posFilter * _filteredPos + (1.0f - P.posFilter) * raw;

    float error      = _filteredPos;
    float derivative = error - _lastError;
    float correction = P.kp * error + P.kd * derivative;
    _lastError = error;

    int dynBase = P.baseSpeed - (int)(fabsf(error) * P.speedDrop);
    dynBase = constrain(dynBase, P.minSpeed, P.baseSpeed);

    // BUG-3 fix applied: direction set inside rightFwd/leftFwd primitives
    // BUG-6 fix applied: speedDrop is tunable (not hardcoded)
    // C3 fix applied:    lower bound is P.minSpeed, not 0
    int lSpd = constrain(dynBase + (int)correction, P.minSpeed, 255);
    int rSpd = constrain(dynBase - (int)correction, P.minSpeed, 255);

    leftFwd(lSpd);
    rightFwd(rSpd);
}

// =========================================================================
//  LOST-LINE RECOVERY  (simplified 3-stage for test sketch)
//
//  Stage 1 REC_REVERSE: reverse briefly in the biased direction
//  Stage 2 REC_PIVOT  : pivot toward lastSeen side until line found
//                       or timeout → back to REC_IDLE
//
//  The two new outer sensors (S6/S7) mean sharpLeft()/sharpRight() now
//  trigger earlier, so the robot enters recovery less often than with 5 IR.
// =========================================================================
static void runRecovery() {
    if (recovMode == REC_IDLE) {
        recovMode  = REC_REVERSE;
        recovStart = millis();
    }

    if (recovMode == REC_REVERSE) {
        // Reverse with slight bias toward last seen side
        if      (lastSeen == SIDE_RIGHT) { rightBwd(P.reverseSpeed + 10); leftBwd(P.reverseSpeed - 10); }
        else if (lastSeen == SIDE_LEFT)  { rightBwd(P.reverseSpeed - 10); leftBwd(P.reverseSpeed + 10); }
        else                              driveBack(P.reverseSpeed);

        readSensors();
        if (anyOn()) { recovMode = REC_IDLE; return; }   // found line while reversing
        if (millis() - recovStart > 400UL) {              // 400 ms reverse max
            recovMode  = REC_PIVOT;
            recovStart = millis();
        }
        return;
    }

    if (recovMode == REC_PIVOT) {
        if      (lastSeen == SIDE_LEFT)  pivotLeft(P.searchSpeed);
        else if (lastSeen == SIDE_RIGHT) pivotRight(P.searchSpeed);
        else                             pivotLeft(P.searchSpeed);  // default

        readSensors();
        if (centreFound() || anyOn()) {
            recovMode = REC_IDLE;
            return;
        }
        if (millis() - recovStart > P.timeoutMs) {
            motorsStop();
            recovMode = REC_IDLE;
            Serial.println(F("[REC] Timeout – line not found. Stopped."));
        }
    }
}

// =========================================================================
//  VERBOSE PRINT  – compact one-line per loop for Serial Plotter / Monitor
// =========================================================================
static unsigned long _lastPrint = 0;
static void verbosePrint() {
    if (millis() - _lastPrint < 150) return;
    _lastPrint = millis();
    // Format: S1..S7 bitmap | pos | recov
    Serial.printf("[T] ");
    for (int i = 0; i < 7; i++) Serial.print(irOn[i] ? "X" : ".");
    float pos = computePos();
    Serial.printf("  pos=%+.2f  recov=%d\n", pos, (recovMode != REC_IDLE) ? 1 : 0);
}

// =========================================================================
//  SETUP
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] 7-IR Line Follower Test"));
    Serial.println(F("  S1-S5: existing sensors"));
    Serial.println(F("  S6=GPIO36, S7=GPIO39: NEW outer sensors"));

    EEPROM.begin(EEPROM_SZ);

    // Motor pins
    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
    ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_ENA, LEDC_CH_R);
    ledcSetup(LEDC_CH_L, LEDC_FREQ, LEDC_RES); ledcAttachPin(PIN_ENB, LEDC_CH_L);
    motorsStop();

    // IR sensor pins  (all input-only on ESP32; no pinMode OUTPUT)
    for (int i = 0; i < 7; i++) pinMode(IR_PIN[i], INPUT);

    // Factory defaults
    for (int i = 0; i < 7; i++) {
        calBlack[i]  = 3600;
        calWhite[i]  =  400;
        calThresh[i] = 2000;
    }

    if (!loadEEPROM()) {
        Serial.println(F("[BOOT] No EEPROM data. Run CALIBRATE first."));
    } else {
        printStatus();
    }

    _filteredPos = 0.0f;
    _lastError   = 0.0f;
    recovMode    = REC_IDLE;
    lastSeen     = SIDE_UNKNOWN;

    Serial.println(F("[BOOT] Ready. Commands: CALIBRATE | STATUS | SAVE | HELP"));
    Serial.println(F("[BOOT] Starting in 2 s..."));
    delay(2000);
    Serial.println(F("[BOOT] Running."));
}

// =========================================================================
//  LOOP
// =========================================================================
void loop() {
    handleSerial();
    readSensors();
    updateLastSeen();
    verbosePrint();

    if (!allDark()) {
        // Line visible — run PID
        if (recovMode != REC_IDLE) recovMode = REC_IDLE;  // abort recovery
        followLine();
    } else {
        // All dark — run recovery
        runRecovery();
    }

    delay(5);
}
