// =========================================================================
//  test/test_servo_diag/main.cpp
//  SERVO / ARM HARDWARE DIAGNOSTIC  —  MG995
//  -------------------------------------------------------------------------
//  SERVO SPECS  (Tower Pro MG995)
//    Operating voltage : 4.8 V – 6.0 V  (USE EXTERNAL SUPPLY — not ESP32 3V3)
//    Stall torque      : 9.4 kg·cm @ 4.8 V  /  11 kg·cm @ 6 V
//    No-load speed     : 0.20 s/60°  @ 4.8 V  /  0.16 s/60° @ 6 V
//    Pulse width range : 500 µs (0°) – 2500 µs (180°)
//    PWM frequency     : 50 Hz (20 ms period)
//    Stall current     : up to 1 A  → MUST use external 5–6 V supply!
//    Physical range    : 0° – 180°  (hard-stop at both ends)
//
//  ⚠ POWER WARNING
//    The MG995 can draw 1 A at stall.  Powering from the ESP32 5V pin (USB
//    limited to ~500 mA shared) WILL brownout the ESP32 mid-test.
//    → Use a dedicated 5 V / 2 A supply or the robot's L298N 5 V output.
//    → Share GND between supply and ESP32.
//
//  PURPOSE
//    Fully exercise the MG995 arm mechanism before robot integration.
//    Tests: jitter, range, step positions, production sweep, full pick
//    sequence, horn-mounting centre, rapid bounce stress.
//
//  NO other hardware required:
//    – No motors     – No IR sensors
//    – No ultrasonic – No colour sensor
//
//  WIRING
//    Servo signal (orange) → GPIO 13
//    Servo VCC    (red)    → External 5–6 V supply  ← NOT ESP32 VIN!
//    Servo GND    (brown)  → GND  (shared with ESP32 GND)
//
//  FLASH
//    pio run --environment test_servo_diag --target upload
//
//  SERIAL COMMANDS (115200 baud)
//    TEST JITTER   – holds HOME angle 5 s, check for vibration/buzz
//    TEST RANGE    – steps 0°→180° in 10° increments (MG995 physical limit)
//    TEST STEP     – 10° steps between HOME and PICK angles
//    TEST SWEEP    – production-speed (1°/20ms) open/close
//    TEST PICK     – full pick-and-place sequence with 3 s countdown
//    TEST CENTRE   – moves to 90° (horn mounting reference)
//    TEST BOUNCE   – 10× rapid open/close stress test
//    TEST ALL      – all tests in sequence
//
//    SET ANGLE <0-180> – move to angle immediately
//    SET HOME  <angle> – set gate-closed angle (default 109)
//    SET PICK  <angle> – set gate-open  angle (default 160)
//    SET SPEED <ms>    – sweep step delay ms  (default 20)
//    STATUS            – print all settings + live pulse widths
//    HOLD              – stop movement, hold current angle
//    HELP              – command list
// =========================================================================
#include <Arduino.h>
#include <ESP32Servo.h>

// ─── MG995 pulse/frequency spec ──────────────────────────────────────────────
#define SERVO_PIN       13
#define SERVO_MIN_US    500    // 0°   (MG995 spec)
#define SERVO_MAX_US    2500   // 180° (MG995 spec — SG90 is 2400, MG995 needs 2500)
#define SERVO_FREQ_HZ   50
#define SERVO_MAX_DEG   180   // MG995 physical hard-stop

// ─── Defaults ────────────────────────────────────────────────────────────────
//  Suggested safe starting angles for MG995 gripper.
//  Tune with TEST STEP then SET HOME / SET PICK.
static int homeAngle = 109;   // gate CLOSED  (same as production)
static int pickAngle = 160;   // gate OPEN    (reduced from 183 — MG995 hits 180° limit)
static int stepDelay = 20;    // ms/° in sweep  (MG995 slower than SG90 under load)
static int curAngle  = 109;

// ─── Servo object ─────────────────────────────────────────────────────────────
static Servo sv;

// =========================================================================
//  PULSE WIDTH HELPER
//  MG995 maps 0–180° to 500–2500 µs linearly.
//  Formula: pw = 500 + (angle / 180.0) * 2000
//  Print these alongside angle so you can verify on a scope.
// =========================================================================
static int angleToPulse(int angle) {
    return (int)(SERVO_MIN_US +
                 ((float)constrain(angle, 0, SERVO_MAX_DEG) / (float)SERVO_MAX_DEG)
                 * (SERVO_MAX_US - SERVO_MIN_US));
}

// =========================================================================
//  CORE MOVE FUNCTIONS
// =========================================================================
static void goTo(int angle) {
    angle    = constrain(angle, 0, SERVO_MAX_DEG);
    curAngle = angle;
    sv.write(angle);
    Serial.printf("  → %3d°   pulse=%4d µs\n", angle, angleToPulse(angle));
}

static void sweep(int from, int to, int dly) {
    from = constrain(from, 0, SERVO_MAX_DEG);
    to   = constrain(to,   0, SERVO_MAX_DEG);
    Serial.printf("  sweep %d° → %d°  (%d ms/step)\n", from, to, dly);
    if (to >= from) {
        for (int a = from; a <= to; a++) { sv.write(a); delay(dly); }
    } else {
        for (int a = from; a >= to; a--) { sv.write(a); delay(dly); }
    }
    curAngle = to;
    Serial.printf("  done → %d°   pulse=%d µs\n", to, angleToPulse(to));
}

// =========================================================================
//  DIAGNOSTIC TESTS
// =========================================================================

// ── TEST 1: JITTER ────────────────────────────────────────────────────────
//  MG995 can buzz/hunt when the PWM signal has jitter or the supply voltage
//  sags. Hold still = supply is clean and LEDC timer has no conflicts.
void testJitter() {
    Serial.println(F("\n[TEST JITTER] Hold HOME 5 s — MG995 must be rock-still"));
    Serial.println(F("  If buzzing: check power supply (needs ≥5V/1A external)"));
    Serial.println(F("  If drifting: LEDC timer conflict with motors (timer 2/3 only)"));
    goTo(homeAngle);
    delay(400);
    for (int i = 5; i > 0; i--) {
        Serial.printf("  %d s remaining...\n", i);
        delay(1000);
    }
    Serial.println(F("  [JITTER] Done. Was arm still?"));
}

// ── TEST 2: RANGE ─────────────────────────────────────────────────────────
//  Steps 0° → 180° in 10° increments.
//  MG995 HARD-STOPS at both ends — going beyond causes grinding.
//  Test stops at 180° (not 270° like SG90 sketch).
void testRange() {
    Serial.println(F("\n[TEST RANGE] 0° → 180° in 10° steps (MG995 physical limit)"));
    Serial.println(F("  ⚠ STOP at 180°. Do NOT command beyond — hard-stop will grind gears."));
    for (int a = 0; a <= SERVO_MAX_DEG; a += 10) {
        goTo(a);
        delay(500);   // MG995 needs more settle time than SG90
    }
    Serial.println(F("  [RANGE] Returning to HOME."));
    sweep(SERVO_MAX_DEG, homeAngle, stepDelay);
    Serial.println(F("  [RANGE] Done."));
}

// ── TEST 3: STEP ──────────────────────────────────────────────────────────
//  10° steps between homeAngle and pickAngle, 700 ms hold per step.
//  Use this to find exact angles for your gripper linkage geometry.
//  Watch the physical gate — note angle where it fully closes / fully opens.
void testStep() {
    Serial.printf("\n[TEST STEP] %d° ↔ %d°  in 10° steps (700 ms hold)\n",
                  homeAngle, pickAngle);
    int lo = min(homeAngle, pickAngle);
    int hi = max(homeAngle, pickAngle);
    Serial.println(F("  → Forward:"));
    for (int a = lo; a <= hi; a += 10) { goTo(a); delay(700); }
    Serial.println(F("  → Reverse:"));
    for (int a = hi; a >= lo; a -= 10) { goTo(a); delay(700); }
    goTo(homeAngle);
    Serial.println(F("  [STEP] Done. Note HOME and PICK angles, then SET HOME/PICK."));
}

// ── TEST 4: SWEEP ─────────────────────────────────────────────────────────
//  1°/stepDelay ms — identical to production servo_moveTo().
//  MG995 default is 20 ms/° (slower than SG90's 15 ms/°) to handle the
//  higher inertia of the metal-gear mechanism under gripper load.
void testSweep() {
    Serial.println(F("\n[TEST SWEEP] Production-speed sweep (1°/step)"));
    Serial.printf("  HOME=%d° → PICK=%d° → HOME=%d°  stepDelay=%d ms\n",
                  homeAngle, pickAngle, homeAngle, stepDelay);
    Serial.println(F("  This is the exact motion used in robot pick-and-place."));
    goTo(homeAngle); delay(400);
    sweep(homeAngle, pickAngle, stepDelay);
    delay(600);
    sweep(pickAngle, homeAngle, stepDelay);
    delay(400);
    Serial.println(F("  [SWEEP] Done."));
}

// ── TEST 5: PICK ──────────────────────────────────────────────────────────
//  Full pick-and-place sequence matching robot.cpp executeGreenPick().
//  Load timing: 3 s to place cube, then automatic sequence runs.
void testPick() {
    Serial.println(F("\n[TEST PICK] Full pick-and-place sequence"));
    Serial.println(F("  Place a cube in front of the gripper."));
    Serial.println(F("  Starting in 3 s..."));
    for (int i = 3; i > 0; i--) { Serial.printf("  %d...\n", i); delay(1000); }

    Serial.println(F("  1. Confirm HOME (gate closed)"));
    goTo(homeAngle); delay(400);

    Serial.println(F("  2. Sweep to PICK (gate open)"));
    sweep(homeAngle, pickAngle, stepDelay);

    Serial.println(F("  3. Hold 500 ms — cube should be gripped"));
    delay(500);

    Serial.println(F("  4. Sweep back to HOME (gate closed)"));
    sweep(pickAngle, homeAngle, stepDelay);
    delay(400);

    Serial.println(F("  [PICK] Done. Is cube secured?"));
    Serial.println(F("  TIP: If cube drops, decrease PICK angle (SET PICK 155)"));
    Serial.println(F("  TIP: If cube not gripped, increase PICK angle (SET PICK 165)"));
}

// ── TEST 6: CENTRE ────────────────────────────────────────────────────────
//  Move to 90° electrical centre (1500 µs) — use this to mount the horn.
//  With the horn at 90°, your HOME and PICK angles are symmetrical and you
//  have equal range in both directions.
void testCentre() {
    Serial.println(F("\n[TEST CENTRE] Moving to 90° (1500 µs — electrical centre)"));
    Serial.println(F("  Mount the servo horn at this position."));
    Serial.printf("  pulse = %d µs\n", angleToPulse(90));
    goTo(90);
    delay(800);
    Serial.println(F("  Holding 5 s — mount your horn now if needed."));
    for (int i = 5; i > 0; i--) { Serial.printf("  %d s...\n", i); delay(1000); }
    Serial.println(F("  Returning to HOME."));
    sweep(90, homeAngle, stepDelay);
    Serial.println(F("  [CENTRE] Done."));
}

// ── TEST 7: BOUNCE ────────────────────────────────────────────────────────
//  10× rapid open/close without sweeping.
//  MG995 metal gears are robust but still check for:
//    • Clicking on direction reversal (worn gear tooth)
//    • Supply voltage sag (LED dims, ESP32 resets)
//    • Servo missing final position (inertia overrun)
void testBounce() {
    Serial.println(F("\n[TEST BOUNCE] 10x rapid open/close (instant write, 500 ms hold)"));
    Serial.println(F("  Listen for clicking gears or supply brownout."));
    Serial.println(F("  MG995 hold time is 500 ms (more inertia than SG90)."));
    for (int i = 1; i <= 10; i++) {
        Serial.printf("  cycle %2d  OPEN  → ", i);
        goTo(pickAngle);  delay(500);
        Serial.printf("CLOSE\n");
        goTo(homeAngle);  delay(500);
    }
    Serial.println(F("  [BOUNCE] Done. Any clicking or brownout?"));
}

// ── TEST ALL ───────────────────────────────────────────────────────────────
void testAll() {
    Serial.println(F("\n========================================"));
    Serial.println(F("  TEST ALL — MG995 full diagnostic"));
    Serial.println(F("========================================"));
    testCentre(); delay(2000);
    testJitter(); delay(2000);
    testStep();   delay(2000);
    testSweep();  delay(2000);
    testBounce(); delay(2000);
    testPick();   delay(2000);
    testRange();  delay(2000);   // RANGE last — approaches hard-stop
    Serial.println(F("\n[TEST ALL] Complete."));
}

// =========================================================================
//  STATUS PRINT
// =========================================================================
static void printStatus() {
    Serial.println(F("\n─── MG995 SERVO DIAG STATUS ───"));
    Serial.printf("  Servo       = Tower Pro MG995\n");
    Serial.printf("  PIN_SERVO   = GPIO %d\n",       SERVO_PIN);
    Serial.printf("  PWM range   = %d – %d µs  @ %d Hz\n",
                  SERVO_MIN_US, SERVO_MAX_US, SERVO_FREQ_HZ);
    Serial.printf("  homeAngle   = %3d°   pulse=%4d µs   (SET HOME %d)\n",
                  homeAngle, angleToPulse(homeAngle), homeAngle);
    Serial.printf("  pickAngle   = %3d°   pulse=%4d µs   (SET PICK %d)\n",
                  pickAngle, angleToPulse(pickAngle), pickAngle);
    Serial.printf("  curAngle    = %3d°   pulse=%4d µs\n",
                  curAngle,  angleToPulse(curAngle));
    Serial.printf("  stepDelay   = %d ms/step  (SET SPEED %d)\n",
                  stepDelay, stepDelay);
    Serial.println(F("  LEDC timers 2 & 3  (motors use 0 & 1 — no conflict)"));
    Serial.println(F("  ⚠ Power: external 5–6 V / 2 A required for MG995!"));
}

// =========================================================================
//  SERIAL COMMAND HANDLER
// =========================================================================
static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "STATUS")       { printStatus(); return; }
    if (cmd == "HOLD")         { Serial.printf("[HOLD] Holding at %d°\n", curAngle); return; }

    if (cmd == "TEST JITTER")  { testJitter();  return; }
    if (cmd == "TEST RANGE")   { testRange();   return; }
    if (cmd == "TEST STEP")    { testStep();    return; }
    if (cmd == "TEST SWEEP")   { testSweep();   return; }
    if (cmd == "TEST PICK")    { testPick();    return; }
    if (cmd == "TEST CENTRE")  { testCentre();  return; }
    if (cmd == "TEST CENTER")  { testCentre();  return; }
    if (cmd == "TEST BOUNCE")  { testBounce();  return; }
    if (cmd == "TEST ALL")     { testAll();     return; }

    if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println(F("Usage: SET KEY VALUE")); return; }
        String k = cmd.substring(4, sp);
        String v = cmd.substring(sp + 1); v.trim();
        bool ok = true;
        if      (k == "ANGLE") { goTo(constrain(v.toInt(), 0, SERVO_MAX_DEG)); return; }
        else if (k == "HOME")  { homeAngle = constrain(v.toInt(), 0, SERVO_MAX_DEG);
                                  Serial.printf("[SET] HOME=%d°  pulse=%d µs\n",
                                                homeAngle, angleToPulse(homeAngle)); }
        else if (k == "PICK")  { pickAngle = constrain(v.toInt(), 0, SERVO_MAX_DEG);
                                  Serial.printf("[SET] PICK=%d°  pulse=%d µs\n",
                                                pickAngle, angleToPulse(pickAngle)); }
        else if (k == "SPEED") { stepDelay = constrain(v.toInt(), 5, 100);
                                  Serial.printf("[SET] stepDelay=%d ms/step\n", stepDelay); }
        else ok = false;
        if (!ok) Serial.printf("Unknown key: %s\n", k.c_str());
        return;
    }

    if (cmd == "HELP") {
        Serial.println(F("\nMG995 SERVO DIAGNOSTIC COMMANDS"));
        Serial.println(F("  Tests:"));
        Serial.println(F("    TEST JITTER  — hold HOME 5 s, check buzz/drift"));
        Serial.println(F("    TEST RANGE   — 0°→180° full range (MG995 limit)"));
        Serial.println(F("    TEST STEP    — 10° steps HOME↔PICK, tune angles"));
        Serial.println(F("    TEST SWEEP   — production-speed 1°/step open/close"));
        Serial.println(F("    TEST PICK    — full pick sequence with countdown"));
        Serial.println(F("    TEST CENTRE  — move to 90° for horn mounting"));
        Serial.println(F("    TEST BOUNCE  — 10x rapid stress test"));
        Serial.println(F("    TEST ALL     — run all tests"));
        Serial.println(F("  Settings:"));
        Serial.println(F("    SET ANGLE <0-180>  — move immediately"));
        Serial.println(F("    SET HOME  <angle>  — gate-closed angle (default 109)"));
        Serial.println(F("    SET PICK  <angle>  — gate-open  angle (default 160)"));
        Serial.println(F("    SET SPEED <ms>     — sweep step delay (default 20)"));
        Serial.println(F("  Other:  STATUS | HOLD"));
        return;
    }

    Serial.println(F("? type HELP"));
}

// =========================================================================
//  SETUP  — BUG-5 safe-attach for MG995
//  MG995 settle delay increased to 800 ms (heavier mechanism than SG90).
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n================================================"));
    Serial.println(F("  MG995 SERVO / ARM HARDWARE DIAGNOSTIC"));
    Serial.println(F("  Signal → GPIO13  |  VCC → Ext 5-6V  |  GND → GND"));
    Serial.println(F("  ⚠ DO NOT power MG995 from ESP32 3V3 or USB 5V!"));
    Serial.println(F("================================================"));

    // BUG-5 safe-attach — mirrors servo_gate.cpp servo_init()
    // Write home BEFORE attach to prevent boot jerk.
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    sv.setPeriodHertz(SERVO_FREQ_HZ);
    sv.write(homeAngle);                              // pre-load angle
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US); // start PWM at home
    delay(800);                                       // MG995 needs 800 ms to settle
    sv.detach();
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US); // re-attach clean
    sv.write(homeAngle);
    curAngle = homeAngle;

    Serial.printf("[BOOT] MG995 at HOME=%d°  pulse=%d µs\n",
                  homeAngle, angleToPulse(homeAngle));
    printStatus();
    Serial.println(F("\n[BOOT] Quick start: TEST ALL"));
    Serial.println(F("[BOOT] Type HELP for full command list."));
    Serial.println();
}

// =========================================================================
//  LOOP
// =========================================================================
void loop() {
    handleSerial();
    delay(10);
}
