// =========================================================================
//  test/test_servo_diag/main.cpp
//  SERVO / ARM HARDWARE DIAGNOSTIC
//  -------------------------------------------------------------------------
//  PURPOSE
//    Fully exercise the servo and arm mechanism before robot integration.
//    Checks:
//      1. Servo reaches and holds the HOME (gate-closed) angle
//      2. Servo reaches and holds the PICK (gate-open) angle
//      3. Gradual sweep does not cause jitter or stall
//      4. Pulse width mapping is correct (500–2400 µs range)
//      5. BUG-5 safe-attach sequence works (no boot jerk)
//      6. Repeated open/close cycles are mechanically stable
//
//  NO other hardware required:
//    – No motors     – No IR sensors
//    – No ultrasonic – No colour sensor
//    Only: ESP32 DevKit + SG90/MG90S servo on GPIO13 + USB cable
//
//  WIRING
//    Servo signal (orange/yellow) → GPIO 13
//    Servo VCC    (red)           → 5V  (use external 5V supply if arm stalls)
//    Servo GND    (brown/black)   → GND (shared with ESP32 GND)
//
//  FLASH
//    pio run --environment test_servo_diag --target upload
//
//  SERIAL COMMANDS (115200 baud)
//    TEST JITTER   – holds home angle 5 s, reports if arm vibrates
//    TEST RANGE    – sweeps full 500–2400 µs range, finds mechanical limits
//    TEST STEP     – steps through every 10° from home to pick and back
//    TEST SWEEP    – slow full open/close sweep (1°/15ms, same as production)
//    TEST PICK     – full pick-and-place sequence (home→pick→home)
//    TEST CENTRE   – moves to 90° (servo electrical centre)
//    TEST BOUNCE   – 10× rapid open/close to check for stall / gear noise
//    TEST ALL      – runs all tests in sequence with 2 s gap between each
//
//    SET ANGLE  <0-270> – move to an arbitrary angle immediately
//    SET HOME   <angle> – change home angle  (default 109)
//    SET PICK   <angle> – change pick angle  (default 183)
//    SET SPEED  <ms>    – step delay ms for sweeps (default 15)
//    STATUS             – print all current settings + live pulse width
//    HOLD               – stop all movement, hold current angle
// =========================================================================
#include <Arduino.h>
#include <ESP32Servo.h>

// ─── Pin ─────────────────────────────────────────────────────────────────────
#define SERVO_PIN       13
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2400
#define SERVO_FREQ_HZ   50

// ─── Defaults matching main project params ───────────────────────────────────
static int   homeAngle  = 109;    // gate CLOSED
static int   pickAngle  = 183;    // gate OPEN
static int   stepDelay  = 15;     // ms per degree in sweep (production value)
static int   curAngle   = 109;    // tracks last commanded angle

// ─── Servo object ────────────────────────────────────────────────────────────
static Servo sv;

// =========================================================================
//  PULSE WIDTH HELPER
//  Converts angle → µs so you can verify with an oscilloscope or logic
//  analyser that the ESP32 PWM signal is within the servo's spec.
//  Formula matches ESP32Servo internals:
//    pw = MIN_US + (angle / 270.0) * (MAX_US - MIN_US)
//  Note: ESP32Servo maps 0–270° by default; SG90 physically limits to ~180°.
// =========================================================================
static int angleToPulse(int angle) {
    return (int)(SERVO_MIN_US + ((float)angle / 270.0f) * (SERVO_MAX_US - SERVO_MIN_US));
}

// =========================================================================
//  CORE MOVE FUNCTIONS
// =========================================================================
static void goTo(int angle) {
    angle    = constrain(angle, 0, 270);
    curAngle = angle;
    sv.write(angle);
    Serial.printf("  → angle=%3d°  pulse=%4d µs\n", angle, angleToPulse(angle));
}

static void sweep(int from, int to, int dly) {
    Serial.printf("  sweep %d° → %d°  (%d ms/step)\n", from, to, dly);
    if (to >= from) {
        for (int a = from; a <= to; a++) { sv.write(a); delay(dly); }
    } else {
        for (int a = from; a >= to; a--) { sv.write(a); delay(dly); }
    }
    curAngle = to;
    Serial.printf("  done. Final angle=%d°  pulse=%d µs\n", to, angleToPulse(to));
}

// =========================================================================
//  INDIVIDUAL DIAGNOSTIC TESTS
// =========================================================================

// ── TEST 1: JITTER ────────────────────────────────────────────────────────
//  Holds the home angle for 5 seconds.
//  Watch the arm: it should be rock-still.
//  If it buzzes or drifts: LEDC timer conflict, bad power supply, or
//  servo damaged. Check that no other LEDC channels share timer 2 or 3.
void testJitter() {
    Serial.println(F("\n[TEST JITTER] Hold HOME angle 5 s – arm must be still"));
    goTo(homeAngle);
    delay(300);
    Serial.println(F("  Holding... watch the arm for vibration."));
    for (int i = 5; i > 0; i--) {
        Serial.printf("  %d s remaining...\n", i);
        delay(1000);
    }
    Serial.println(F("  [JITTER] Done. Was arm still? (Y expected)"));
}

// ── TEST 2: RANGE ────────────────────────────────────────────────────────
//  Steps from 0° to 270° in 10° increments, pausing 400 ms at each step.
//  Prints pulse width at each step.
//  EXPECTED: arm moves smoothly until it hits its physical hard-stop.
//  NOTE: SG90 stalls above ~180°. Listen for grinding – stop test if heard.
void testRange() {
    Serial.println(F("\n[TEST RANGE] Step 0° → 270° in 10° steps"));
    Serial.println(F("  ⚠ Stop if you hear grinding (hard-stop reached)."));
    for (int a = 0; a <= 270; a += 10) {
        goTo(a);
        delay(400);
    }
    Serial.println(F("  [RANGE] Returning to HOME."));
    goTo(homeAngle);
    delay(600);
    Serial.println(F("  [RANGE] Done."));
}

// ── TEST 3: STEP ─────────────────────────────────────────────────────────
//  Steps every 10° between homeAngle and pickAngle.
//  Useful for finding the exact pick angle for your gripper geometry.
void testStep() {
    Serial.printf("\n[TEST STEP] %d° → %d° in 10° steps, then return\n",
                  homeAngle, pickAngle);
    int lo = min(homeAngle, pickAngle);
    int hi = max(homeAngle, pickAngle);
    Serial.println(F("  Forward:"));
    for (int a = lo; a <= hi; a += 10) { goTo(a); delay(600); }
    Serial.println(F("  Reverse:"));
    for (int a = hi; a >= lo; a -= 10) { goTo(a); delay(600); }
    goTo(homeAngle);
    Serial.println(F("  [STEP] Done."));
}

// ── TEST 4: SWEEP ─────────────────────────────────────────────────────────
//  Slow 1°/stepDelay ms sweep (identical to production servo_moveTo()).
//  This is the EXACT motion the robot uses during pick-and-place.
void testSweep() {
    Serial.println(F("\n[TEST SWEEP] Production-speed open/close (1°/step)"));
    Serial.printf("  HOME=%d° → PICK=%d° → HOME=%d°  stepDelay=%d ms\n",
                  homeAngle, pickAngle, homeAngle, stepDelay);
    goTo(homeAngle); delay(400);
    sweep(homeAngle, pickAngle, stepDelay);
    delay(500);
    sweep(pickAngle, homeAngle, stepDelay);
    delay(400);
    Serial.println(F("  [SWEEP] Done."));
}

// ── TEST 5: PICK ──────────────────────────────────────────────────────────
//  Full pick-and-place sequence matching robot.cpp executeGreenPick():
//    1. Confirm at home
//    2. Sweep to pick angle
//    3. Hold 500 ms (cube gripped)
//    4. Sweep back to home
//  Timing values match production code.
void testPick() {
    Serial.println(F("\n[TEST PICK] Full production pick sequence"));
    Serial.println(F("  Place a cube in front of the gripper now."));
    Serial.println(F("  Starting in 3 s..."));
    delay(3000);

    Serial.println(F("  Step 1: Confirm HOME"));
    goTo(homeAngle); delay(400);

    Serial.println(F("  Step 2: Open gate (sweep to PICK)"));
    sweep(homeAngle, pickAngle, stepDelay);

    Serial.println(F("  Step 3: Hold 500 ms (gripping)"));
    delay(500);

    Serial.println(F("  Step 4: Close gate (sweep to HOME)"));
    sweep(pickAngle, homeAngle, stepDelay);
    delay(400);

    Serial.println(F("  [PICK] Done. Check cube is held securely."));
}

// ── TEST 6: CENTRE ────────────────────────────────────────────────────────
//  Moves to electrical centre (90°).
//  Useful for centering the horn before mounting the arm linkage.
void testCentre() {
    Serial.println(F("\n[TEST CENTRE] Moving to 90° (electrical centre)"));
    Serial.printf("  pulse = %d µs\n", angleToPulse(90));
    goTo(90);
    delay(800);
    Serial.println(F("  Arm is at centre. Use this position to mount the horn."));
    Serial.println(F("  Returning to HOME in 5 s..."));
    delay(5000);
    goTo(homeAngle);
    Serial.println(F("  [CENTRE] Done."));
}

// ── TEST 7: BOUNCE ────────────────────────────────────────────────────────
//  10 rapid open/close cycles (no sweep, instant write).
//  Tests for:
//    • Gear stripping under rapid direction reversal
//    • LEDC timer glitches causing unintended angles
//    • Mechanical binding in the gate linkage
void testBounce() {
    Serial.println(F("\n[TEST BOUNCE] 10x rapid open/close (instant, no sweep)"));
    Serial.println(F("  Listen for grinding or clicking."));
    for (int i = 1; i <= 10; i++) {
        Serial.printf("  cycle %2d: open  ", i);
        goTo(pickAngle);  delay(300);
        Serial.printf("close\n");
        goTo(homeAngle);  delay(300);
    }
    Serial.println(F("  [BOUNCE] Done. Any grinding heard?"));
}

// ── TEST ALL ──────────────────────────────────────────────────────────────
void testAll() {
    Serial.println(F("\n========================================"));
    Serial.println(F("  TEST ALL  – running every diagnostic"));
    Serial.println(F("========================================"));
    testCentre();  delay(2000);
    testJitter();  delay(2000);
    testStep();    delay(2000);
    testSweep();   delay(2000);
    testBounce();  delay(2000);
    testPick();    delay(2000);
    // testRange() intentionally last – may reach hard stop
    testRange();   delay(2000);
    Serial.println(F("\n[TEST ALL] Complete."));
}

// =========================================================================
//  STATUS PRINT
// =========================================================================
static void printStatus() {
    Serial.println(F("\n─── SERVO DIAG STATUS ───"));
    Serial.printf("  PIN_SERVO  = GPIO %d\n",     SERVO_PIN);
    Serial.printf("  homeAngle  = %d°   pulse=%d µs   (SET HOME %d)\n",
                  homeAngle, angleToPulse(homeAngle), homeAngle);
    Serial.printf("  pickAngle  = %d°   pulse=%d µs   (SET PICK %d)\n",
                  pickAngle, angleToPulse(pickAngle), pickAngle);
    Serial.printf("  curAngle   = %d°   pulse=%d µs\n",
                  curAngle,  angleToPulse(curAngle));
    Serial.printf("  stepDelay  = %d ms/step        (SET SPEED %d)\n",
                  stepDelay, stepDelay);
    Serial.printf("  PWM range  = %d – %d µs  @ %d Hz\n",
                  SERVO_MIN_US, SERVO_MAX_US, SERVO_FREQ_HZ);
    Serial.printf("  LEDC timers 2 & 3 (motors use 0 & 1 – no conflict)\n");
    Serial.println(F("  Commands: TEST JITTER|RANGE|STEP|SWEEP|PICK|CENTRE|BOUNCE|ALL"));
    Serial.println(F("            SET ANGLE|HOME|PICK|SPEED <val>  |  HOLD  |  STATUS"));
}

// =========================================================================
//  SERIAL COMMAND HANDLER
// =========================================================================
static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "STATUS")        { printStatus();  return; }
    if (cmd == "HOLD")          { Serial.printf("[HOLD] Holding at %d°\n", curAngle); return; }

    if (cmd == "TEST JITTER")   { testJitter();   return; }
    if (cmd == "TEST RANGE")    { testRange();    return; }
    if (cmd == "TEST STEP")     { testStep();     return; }
    if (cmd == "TEST SWEEP")    { testSweep();    return; }
    if (cmd == "TEST PICK")     { testPick();     return; }
    if (cmd == "TEST CENTRE")   { testCentre();   return; }
    if (cmd == "TEST CENTER")   { testCentre();   return; }
    if (cmd == "TEST BOUNCE")   { testBounce();   return; }
    if (cmd == "TEST ALL")      { testAll();      return; }

    if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println(F("Usage: SET KEY VALUE")); return; }
        String k = cmd.substring(4, sp);
        String v = cmd.substring(sp + 1); v.trim();
        bool ok = true;
        if      (k == "ANGLE") { int a = constrain(v.toInt(), 0, 270); goTo(a); return; }
        else if (k == "HOME")  { homeAngle = constrain(v.toInt(), 0, 270);
                                  Serial.printf("[SET] HOME=%d°  pulse=%d µs\n",
                                                homeAngle, angleToPulse(homeAngle)); }
        else if (k == "PICK")  { pickAngle = constrain(v.toInt(), 0, 270);
                                  Serial.printf("[SET] PICK=%d°  pulse=%d µs\n",
                                                pickAngle, angleToPulse(pickAngle)); }
        else if (k == "SPEED") { stepDelay = constrain(v.toInt(), 1, 100);
                                  Serial.printf("[SET] stepDelay=%d ms\n", stepDelay); }
        else ok = false;
        if (!ok) Serial.printf("Unknown key: %s\n", k.c_str());
        return;
    }

    if (cmd == "HELP") {
        Serial.println(F("\nSERVO DIAGNOSTIC COMMANDS"));
        Serial.println(F("  Tests:"));
        Serial.println(F("    TEST JITTER  – hold HOME 5 s, check for vibration"));
        Serial.println(F("    TEST RANGE   – full 0°→270° range sweep"));
        Serial.println(F("    TEST STEP    – 10° steps between HOME and PICK"));
        Serial.println(F("    TEST SWEEP   – production-speed open/close"));
        Serial.println(F("    TEST PICK    – full pick-and-place sequence"));
        Serial.println(F("    TEST CENTRE  – move to 90° (horn mounting reference)"));
        Serial.println(F("    TEST BOUNCE  – 10x rapid open/close stress test"));
        Serial.println(F("    TEST ALL     – run every test in sequence"));
        Serial.println(F("  Settings:"));
        Serial.println(F("    SET ANGLE <0-270>  – move to angle immediately"));
        Serial.println(F("    SET HOME  <angle>  – set gate-closed angle (default 109)"));
        Serial.println(F("    SET PICK  <angle>  – set gate-open  angle (default 183)"));
        Serial.println(F("    SET SPEED <ms>     – sweep step delay (default 15)"));
        Serial.println(F("  Other:"));
        Serial.println(F("    STATUS  – print all settings + pulse widths"));
        Serial.println(F("    HOLD    – stop and hold current angle"));
        return;
    }

    Serial.println(F("? type HELP for command list"));
}

// =========================================================================
//  SETUP  – BUG-5 safe-attach sequence (matches servo_gate.cpp)
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n================================================"));
    Serial.println(F("  SERVO / ARM HARDWARE DIAGNOSTIC"));
    Serial.println(F("  Signal → GPIO13  |  VCC → 5V  |  GND → GND"));
    Serial.println(F("================================================"));

    // ── BUG-5 safe-attach (mirrors servo_gate.cpp servo_init()) ──────────
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    sv.setPeriodHertz(SERVO_FREQ_HZ);

    // Write home BEFORE attach so first PWM pulse = home (no boot jerk)
    sv.write(homeAngle);
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    delay(600);
    sv.detach();
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    sv.write(homeAngle);
    curAngle = homeAngle;

    Serial.printf("[BOOT] Servo at HOME=%d°  pulse=%d µs\n",
                  homeAngle, angleToPulse(homeAngle));

    printStatus();
    Serial.println(F("\n[BOOT] Type HELP for commands."));
    Serial.println(F("[BOOT] Quick start: TEST ALL"));
    Serial.println();
}

// =========================================================================
//  LOOP  – just handle serial commands, servo moves on demand only
// =========================================================================
void loop() {
    handleSerial();
    delay(10);
}
