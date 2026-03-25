// =========================================================================
//  test/test_obstacle_debug/main.cpp
//  OBSTACLE DETECTION + TCS3200 COLOUR DEBUG SKETCH
//  -------------------------------------------------------------------------
//  PURPOSE
//    Isolate and validate the HC-SR04 ultrasonic sensor and TCS3200 colour
//    sensor BEFORE integrating them into the main state machine.
//    No motors move. No IR sensors needed. No EEPROM required.
//
//  WHAT IT DOES EACH LOOP (every 50 ms)
//    1. Reads HC-SR04 distance
//    2. Prints a live ASCII bar chart of the distance
//    3. Classifies distance into zone:
//         CLEAR       dist >= SLOW_DIST
//         SLOW        STOP_DIST < dist < SLOW_DIST
//         STOP+COLOR  dist <= STOP_DIST  → triggers TCS3200 read automatically
//    4. When in STOP+COLOR zone:
//         • Reads TCS3200 R, G, B three times and averages
//         • Prints raw values and decision (RED / GREEN / NONE)
//         • Prints whether current thresholds would trigger RED_AVOID or GREEN_PICK
//
//  SERIAL COMMANDS (115200 baud)
//    STATUS          print all thresholds
//    SET SLOW  17    set obstacle slow distance  (cm)
//    SET STOP  9     set colour-check stop distance (cm)
//    SET REDTHR  120 set red   detection threshold (lower = more sensitive)
//    SET GRNTHR  100 set green detection threshold
//    SET SAMPLES 3   number of colour reads to average (1–5)
//    COLOUR          force a manual colour read right now
//    DISTANCE        print one distance reading immediately
//    STREAM ON       enable continuous 50-ms distance stream (default ON)
//    STREAM OFF      pause the stream
//
//  PINS (matching main project config.h)
//    TRIG → GPIO17   ECHO → GPIO16  (⚠ voltage divider on ECHO!)
//    TCS S0→GPIO14  S1→GPIO15  S2→GPIO26  S3→GPIO25  OUT→GPIO2
//
//  Flash:
//    pio run --environment test_obstacle_debug --target upload
// =========================================================================
#include <Arduino.h>

// ── HC-SR04 pins ───────────────────────────────────────────────────────────────────
#define PIN_TRIG   17
#define PIN_ECHO   16   // ⚠ 5V output – use 1kΩ/2kΩ voltage divider to 3.3V!

// ── TCS3200 pins ────────────────────────────────────────────────────────────────
#define PIN_CS_S0   14
#define PIN_CS_S1   15
#define PIN_CS_S2   26
#define PIN_CS_S3   25
#define PIN_CS_OUT   2   // boot-strap pin – INPUT is safe

// ───────────────────────────────────────────────────────────────────────────
//  TUNABLE THRESHOLDS
//  All changeable live via SET command without reflashing.
// ───────────────────────────────────────────────────────────────────────────
static float slowDist     = 17.0f;   // cm: enter SLOW zone
static float stopDist     =  9.0f;   // cm: stop and check colour
static int   redThresh    = 120;     // TCS3200: lower = stronger red
static int   greenThresh  = 100;     // TCS3200: lower = stronger green
static int   numSamples   = 3;       // colour reads to average (1–5)
static bool  streamOn     = true;    // continuous distance stream

// ── Runtime state ─────────────────────────────────────────────────────────────────
static unsigned long _lastDistMs  = 0;
static unsigned long _lastColourMs = 0;
static bool _inColourZone = false;

// How often to re-run colour detection while stopped (ms)
// Prevents hammering the TCS3200 while the cube stays in range.
#define COLOUR_RECHECK_MS  1500UL

// =========================================================================
//  HC-SR04
// =========================================================================
static float readDistance() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);   // 30 ms = ~500 cm max
    if (dur == 0) return 999.0f;                   // open / no echo
    return dur * 0.034f / 2.0f;
}

// =========================================================================
//  TCS3200
// =========================================================================
static int _readChannel(bool s2, bool s3) {
    digitalWrite(PIN_CS_S2, s2 ? HIGH : LOW);
    digitalWrite(PIN_CS_S3, s3 ? HIGH : LOW);
    delay(30);   // photodiode settle
    return (int)pulseIn(PIN_CS_OUT, LOW, 100000UL);
}

static void readRGB(int &r, int &g, int &b) {
    r = _readChannel(false, false);   // Red   filter
    g = _readChannel(true,  true);    // Green filter
    b = _readChannel(false, true);    // Blue  filter
}

// =========================================================================
//  COLOUR DETECTION + REPORT
//  Prints a detailed breakdown every time it runs so you can see exactly
//  which channel fired, what the margins are, and what the robot would do.
// =========================================================================
static void runColourDetect() {
    Serial.println(F("\n┌──────────────────────────────────────┐"));
    Serial.println(F("│  TCS3200 COLOUR READ                 │"));
    Serial.println(F("└──────────────────────────────────────┘"));

    long rSum = 0, gSum = 0, bSum = 0;
    int  ns = constrain(numSamples, 1, 5);

    for (int i = 0; i < ns; i++) {
        int r, g, b;
        readRGB(r, g, b);
        rSum += r; gSum += g; bSum += b;
        Serial.printf("  sample %d:  R=%-5d G=%-5d B=%-5d\n", i+1, r, g, b);
        delay(10);
    }

    int r = (int)(rSum / ns);
    int g = (int)(gSum / ns);
    int b = (int)(bSum / ns);

    Serial.println(F("  ────────────────────────────"));
    Serial.printf("  AVERAGE    R=%-5d G=%-5d B=%-5d\n", r, g, b);
    Serial.printf("  THRESHOLDS redThr=%-4d  grnThr=%-4d\n", redThresh, greenThresh);

    // ── margin analysis ───────────────────────────────────────────────────
    // TCS3200: lower pulse = brighter that colour
    // Red cube:   R lowest, R < redThresh, R <= G, R <= B
    // Green cube: G lowest, G < greenThresh, G <= R, G <= B
    bool redOk   = (r < redThresh)   && (r <= g) && (r <= b);
    bool greenOk = (g < greenThresh) && (g <= r) && (g <= b);

    Serial.println(F("  ── Channel dominance check ──────────────"));
    Serial.printf("  R lowest?  %s  (R<=G: %s  R<=B: %s)\n",
                  (r<=g&&r<=b)?"YES":"NO",
                  r<=g?"YES":"NO", r<=b?"YES":"NO");
    Serial.printf("  G lowest?  %s  (G<=R: %s  G<=B: %s)\n",
                  (g<=r&&g<=b)?"YES":"NO",
                  g<=r?"YES":"NO", g<=b?"YES":"NO");

    Serial.println(F("  ── Threshold check ─────────────────────"));
    Serial.printf("  R < redThresh  (%d < %d)? %s   margin=%d\n",
                  r, redThresh,   r<redThresh?"YES":"NO", redThresh - r);
    Serial.printf("  G < grnThresh  (%d < %d)? %s   margin=%d\n",
                  g, greenThresh, g<greenThresh?"YES":"NO", greenThresh - g);

    // ── ASCII colour bar ──────────────────────────────────────────────────
    // Invert for display: higher bar = stronger colour
    // Max raw ~400 (very bright), typical background ~2000+
    // Clamp display to 0-2000 range for readability
    auto bar = [](const char* label, int val) {
        int inv = constrain(2000 - val, 0, 2000);  // invert: bright = tall
        int bars = inv / 100;                       // 0-20 bars
        Serial.printf("  %s [%-20s] raw=%d\n", label,
            String('|').length() ? String('|' + String(bars > 0 ? String(bars,'|') : String(""))).c_str() : "",
            val);
        Serial.printf("  %-2s ", label);
        for (int i = 0; i < 20; i++) Serial.print(i < bars ? "|" : " ");
        Serial.printf(" raw=%-5d\n", val);
    };
    Serial.println(F("  ── Strength bars (longer = brighter) ───"));
    bar("R", r);
    bar("G", g);
    bar("B", b);

    // ── DECISION ─────────────────────────────────────────────────────────────
    Serial.println(F("  ── DECISION ──────────────────────────"));
    if (redOk) {
        Serial.println(F("  >>> RESULT: RED  → robot would execute RED_AVOID"));
    } else if (greenOk) {
        Serial.println(F("  >>> RESULT: GREEN → robot would execute GREEN_PICK"));
    } else {
        Serial.println(F("  >>> RESULT: NONE / UNKNOWN → robot resumes LINE_FOLLOW"));
        Serial.println(F("  TIP: Adjust SET REDTHR or SET GRNTHR if wrong colour detected"));
    }
    Serial.println();
}

// =========================================================================
//  ASCII DISTANCE BAR
//  Shows a visual ruler from 0 cm to 40 cm.
//  Threshold markers show SLOW and STOP zones.
// =========================================================================
//  Example output:
//  dist=12.4 cm  [============|----!----]  SLOW
//                              ↑STOP  ↑SLOW
// =========================================================================
static void printDistBar(float dist) {
    const float MAX_CM = 40.0f;
    const int   BAR_W  = 30;

    int pos      = (int)((constrain(dist, 0.0f, MAX_CM) / MAX_CM) * BAR_W);
    int stopPos  = (int)((stopDist  / MAX_CM) * BAR_W);
    int slowPos  = (int)((slowDist  / MAX_CM) * BAR_W);

    // Zone label
    const char* zone;
    if      (dist >= slowDist) zone = "CLEAR     ";
    else if (dist >  stopDist) zone = "SLOW      ";
    else                       zone = "STOP+COLOR";

    Serial.printf("dist=%5.1f cm  [", dist);
    for (int i = 0; i < BAR_W; i++) {
        if (i == pos)     Serial.print("|");   // robot position
        else if (i == stopPos) Serial.print("!");   // STOP threshold
        else if (i == slowPos) Serial.print(":");   // SLOW threshold
        else if (i < pos) Serial.print("=");   // filled
        else              Serial.print("-");   // empty
    }
    Serial.printf("]  %s\n", zone);
}

// =========================================================================
//  STATUS PRINT
// =========================================================================
static void printStatus() {
    Serial.println(F("\n─── OBSTACLE DEBUG STATUS ───"));
    Serial.printf("  slowDist   = %.1f cm  (SET SLOW %.0f)\n",  slowDist, slowDist);
    Serial.printf("  stopDist   = %.1f cm  (SET STOP %.0f)\n",  stopDist, stopDist);
    Serial.printf("  redThresh  = %d        (SET REDTHR %d)\n",  redThresh, redThresh);
    Serial.printf("  greenThresh= %d        (SET GRNTHR %d)\n",  greenThresh, greenThresh);
    Serial.printf("  numSamples = %d        (SET SAMPLES %d)\n", numSamples, numSamples);
    Serial.printf("  stream     = %s       (STREAM ON/OFF)\n",   streamOn ? "ON " : "OFF");
    Serial.println(F("  Pins: TRIG=17 ECHO=16  S0=14 S1=15 S2=26 S3=25 OUT=2"));
}

// =========================================================================
//  SERIAL COMMAND HANDLER
// =========================================================================
static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "STATUS") { printStatus(); return; }

    if (cmd == "COLOUR" || cmd == "COLOR") {
        Serial.println(F("[CMD] Manual colour read:"));
        runColourDetect();
        return;
    }

    if (cmd == "DISTANCE") {
        float d = readDistance();
        Serial.printf("[CMD] Distance = %.1f cm\n", d);
        printDistBar(d);
        return;
    }

    if (cmd == "STREAM ON")  { streamOn = true;  Serial.println(F("[CMD] Stream ON"));  return; }
    if (cmd == "STREAM OFF") { streamOn = false; Serial.println(F("[CMD] Stream OFF")); return; }

    if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println(F("Usage: SET KEY VALUE")); return; }
        String k = cmd.substring(4, sp);
        String v = cmd.substring(sp + 1); v.trim();
        bool ok = true;
        if      (k == "SLOW")    slowDist    = v.toFloat();
        else if (k == "STOP")    stopDist    = v.toFloat();
        else if (k == "REDTHR")  redThresh   = v.toInt();
        else if (k == "GRNTHR")  greenThresh = v.toInt();
        else if (k == "SAMPLES") numSamples  = constrain(v.toInt(), 1, 5);
        else ok = false;
        if (ok) {
            Serial.printf("[SET] %s = %s\n", k.c_str(), v.c_str());
            printStatus();
        } else {
            Serial.printf("Unknown key: %s\n", k.c_str());
        }
        return;
    }

    if (cmd == "HELP") {
        Serial.println(F("Commands:"));
        Serial.println(F("  STATUS"));
        Serial.println(F("  DISTANCE          – single distance read"));
        Serial.println(F("  COLOUR            – manual TCS3200 read"));
        Serial.println(F("  STREAM ON/OFF     – toggle continuous stream"));
        Serial.println(F("  SET SLOW   <cm>   – obstacle slow threshold (default 17)"));
        Serial.println(F("  SET STOP   <cm>   – colour-check stop threshold (default 9)"));
        Serial.println(F("  SET REDTHR <val>  – red detection threshold (default 120)"));
        Serial.println(F("  SET GRNTHR <val>  – green detection threshold (default 100)"));
        Serial.println(F("  SET SAMPLES <1-5> – colour reads to average (default 3)"));
        return;
    }

    Serial.println(F("? type HELP"));
}

// =========================================================================
//  SETUP
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=============================================="));
    Serial.println(F("  OBSTACLE DETECTION DEBUG SKETCH"));
    Serial.println(F("  HC-SR04 + TCS3200  |  No motors, No IR"));
    Serial.println(F("=============================================="));
    Serial.println(F("  ⚠  Ensure voltage divider on ECHO pin!"));
    Serial.println(F("  ⚠  TCS3200 OUT connected to GPIO2 (INPUT safe)"));
    Serial.println();

    // HC-SR04
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);

    // TCS3200 – 20% frequency scale (S0=H, S1=L)
    pinMode(PIN_CS_S0,  OUTPUT); pinMode(PIN_CS_S1, OUTPUT);
    pinMode(PIN_CS_S2,  OUTPUT); pinMode(PIN_CS_S3, OUTPUT);
    pinMode(PIN_CS_OUT, INPUT);
    digitalWrite(PIN_CS_S0, HIGH);
    digitalWrite(PIN_CS_S1, LOW);

    printStatus();
    Serial.println(F("\n[BOOT] Running. Type HELP for commands."));
    Serial.println(F("[BOOT] Distance stream active every 50 ms."));
    Serial.println(F("       │                              │"));
    Serial.println(F("       !=STOP mark  :=SLOW mark    |=robot"));
    Serial.println();
    delay(500);
}

// =========================================================================
//  LOOP
// =========================================================================
void loop() {
    handleSerial();

    unsigned long now = millis();

    // ── Distance stream (every 50 ms) ───────────────────────────────────────
    if (streamOn && (now - _lastDistMs >= 50UL)) {
        _lastDistMs = now;
        float dist = readDistance();
        printDistBar(dist);

        // ── Zone transition detection ───────────────────────────────────
        if (dist <= stopDist) {
            // In colour-check zone
            if (!_inColourZone) {
                // Just entered the zone – run colour detect immediately
                _inColourZone = true;
                Serial.println(F(">>> Entered STOP+COLOR zone!"));
                runColourDetect();
                _lastColourMs = millis();
            } else if (now - _lastColourMs >= COLOUR_RECHECK_MS) {
                // Re-check colour every 1.5 s while cube stays in range
                Serial.println(F(">>> Re-checking colour..."));
                runColourDetect();
                _lastColourMs = millis();
            }
        } else {
            // Outside colour zone
            if (_inColourZone) {
                Serial.println(F(">>> Left STOP+COLOR zone – object removed or moved."));
            }
            _inColourZone = false;
        }
    }

    delay(5);
}
