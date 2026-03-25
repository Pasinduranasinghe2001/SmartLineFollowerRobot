// =========================================================================
//  MG996R Servo Test  |  GPIO13  |  External 5V/2A supply REQUIRED
// =========================================================================
//  MG996R SPECS
//    Pulse range  : 900 us (0 deg) -> 2100 us (180 deg)
//    PWM freq     : 50 Hz
//    Torque       : 9.4 kg.cm @ 4.8V  /  11 kg.cm @ 6V
//    Speed        : 0.19 s/60deg @ 4.8V
//    Stall current: up to 2.5A  -> use dedicated 5V/2A supply!
//    Physical range: 0 - 180 deg
//
//  WIRING
//    Signal (orange) -> GPIO 13
//    VCC    (red)    -> External 5V/2A  (NOT ESP32 pin!)
//    GND    (brown)  -> GND shared with ESP32
//
//  FLASH:   pio run --environment test_servo_diag --target upload
//  MONITOR: pio device monitor --environment test_servo_diag
//
//  COMMANDS (115200 baud)
//    OPEN            move to PICK angle
//    CLOSE           move to HOME angle
//    CENTRE          move to 90 deg (use for horn mounting)
//    SWEEP           slow open -> close cycle
//    STATUS          print HOME / PICK / current angle + pulse width
//    SET HOME <deg>  change gate-closed angle (0-180)
//    SET PICK <deg>  change gate-open  angle (0-180)
//    SET ANGLE <deg> jump to any angle immediately
//    SET SPEED <ms>  change sweep step delay (default 15)
// =========================================================================
#include <Arduino.h>
#include <ESP32Servo.h>

// MG996R pulse range
#define SERVO_PIN     13
#define SERVO_MIN_US  900    // 0 deg
#define SERVO_MAX_US  2100   // 180 deg
#define SERVO_MAX_DEG 180

Servo sv;
int homeAngle = 90;    // gate CLOSED  - tune with TEST STEP
int pickAngle = 150;   // gate OPEN    - tune with TEST STEP
int stepDelay = 15;    // ms per degree in sweep
int curAngle  = 90;

// pulse width display helper
static int toPulse(int deg) {
    return (int)(SERVO_MIN_US +
           ((float)constrain(deg,0,SERVO_MAX_DEG) / SERVO_MAX_DEG)
           * (SERVO_MAX_US - SERVO_MIN_US));
}

void goTo(int angle) {
    angle = constrain(angle, 0, SERVO_MAX_DEG);
    curAngle = angle;
    sv.write(angle);
    Serial.printf("=> %d deg  (%d us)\n", angle, toPulse(angle));
}

void sweep(int from, int to) {
    from = constrain(from, 0, SERVO_MAX_DEG);
    to   = constrain(to,   0, SERVO_MAX_DEG);
    if (to >= from) for (int a = from; a <= to; a++) { sv.write(a); delay(stepDelay); }
    else            for (int a = from; a >= to; a--) { sv.write(a); delay(stepDelay); }
    curAngle = to;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MG996R Servo Test ===");
    Serial.println("Signal=GPIO13  VCC=Ext5V/2A  GND=GND");
    Serial.println("Pulse: 900us(0deg) - 2100us(180deg)");
    Serial.println("! Do NOT power from ESP32 USB - needs 2A external supply");
    Serial.println("Commands: OPEN CLOSE CENTRE SWEEP STATUS SET HOME/PICK/ANGLE/SPEED <val>\n");

    // BUG-5 safe attach - write home BEFORE attach to prevent boot jerk
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    sv.setPeriodHertz(50);
    sv.write(homeAngle);                              // pre-load
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US); // first pulse = home
    delay(800);                                       // settle
    sv.detach();
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US); // re-attach clean
    sv.write(homeAngle);
    curAngle = homeAngle;

    Serial.printf("Ready. HOME=%d(%dus)  PICK=%d(%dus)\n",
                  homeAngle, toPulse(homeAngle),
                  pickAngle, toPulse(pickAngle));
}

void loop() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "OPEN") {
        Serial.println("[OPEN]");
        goTo(pickAngle);

    } else if (cmd == "CLOSE") {
        Serial.println("[CLOSE]");
        goTo(homeAngle);

    } else if (cmd == "CENTRE" || cmd == "CENTER") {
        Serial.println("[CENTRE] 90 deg - mount horn here");
        goTo(90);

    } else if (cmd == "SWEEP") {
        Serial.printf("[SWEEP] HOME(%d) -> PICK(%d) -> HOME(%d)  %dms/step\n",
                      homeAngle, pickAngle, homeAngle, stepDelay);
        sweep(homeAngle, pickAngle);
        delay(500);
        sweep(pickAngle, homeAngle);
        Serial.println("Done.");

    } else if (cmd == "STATUS") {
        Serial.println("--- MG996R STATUS ---");
        Serial.printf("  HOME  = %3d deg  (%d us)\n", homeAngle, toPulse(homeAngle));
        Serial.printf("  PICK  = %3d deg  (%d us)\n", pickAngle, toPulse(pickAngle));
        Serial.printf("  CUR   = %3d deg  (%d us)\n", curAngle,  toPulse(curAngle));
        Serial.printf("  SPEED = %d ms/step\n",        stepDelay);
        Serial.printf("  PWM   = %d-%d us @ 50Hz  GPIO%d\n",
                      SERVO_MIN_US, SERVO_MAX_US, SERVO_PIN);

    } else if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println("Usage: SET HOME/PICK/ANGLE/SPEED <value>"); return; }
        String k = cmd.substring(4, sp);
        int    v = cmd.substring(sp + 1).toInt();
        if      (k == "HOME")  { homeAngle = constrain(v, 0, SERVO_MAX_DEG);
                                  Serial.printf("HOME=%d deg (%d us)\n", homeAngle, toPulse(homeAngle)); }
        else if (k == "PICK")  { pickAngle = constrain(v, 0, SERVO_MAX_DEG);
                                  Serial.printf("PICK=%d deg (%d us)\n", pickAngle, toPulse(pickAngle)); }
        else if (k == "ANGLE") { goTo(v); }
        else if (k == "SPEED") { stepDelay = constrain(v, 5, 100);
                                  Serial.printf("SPEED=%d ms/step\n", stepDelay); }
        else Serial.println("Unknown key. Use HOME PICK ANGLE SPEED");

    } else {
        Serial.println("? OPEN | CLOSE | CENTRE | SWEEP | STATUS | SET HOME/PICK/ANGLE/SPEED <val>");
    }
}
