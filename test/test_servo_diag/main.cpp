// MG995 Servo Test  |  GPIO13  |  External 5-6V supply required
// Flash: pio run --environment test_servo_diag --target upload
// Monitor: pio device monitor --environment test_servo_diag
//
// Commands (115200 baud):
//   OPEN      move to PICK angle
//   CLOSE     move to HOME angle
//   CENTRE    move to 90deg
//   SET HOME <angle>
//   SET PICK <angle>
//   SET ANGLE <angle>
//   SWEEP     full open/close sweep
//   STATUS

#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN    13
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

Servo sv;
int homeAngle = 109;
int pickAngle = 160;
int curAngle  = 109;

void goTo(int angle) {
    angle = constrain(angle, 0, 180);
    curAngle = angle;
    sv.write(angle);
    int pulse = (int)(500 + ((float)angle / 180.0f) * 2000);
    Serial.printf("=> %d deg  (%d us)\n", angle, pulse);
}

void sweep(int from, int to) {
    if (to > from) for (int a = from; a <= to; a++) { sv.write(a); delay(20); }
    else           for (int a = from; a >= to; a--) { sv.write(a); delay(20); }
    curAngle = to;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MG995 Servo Test ===");
    Serial.println("Signal=GPIO13  VCC=Ext5V  GND=GND");
    Serial.println("Commands: OPEN CLOSE CENTRE SWEEP STATUS SET HOME/PICK/ANGLE <val>");

    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    sv.setPeriodHertz(50);
    sv.write(homeAngle);
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    delay(800);
    sv.detach();
    sv.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    sv.write(homeAngle);
    Serial.printf("Ready. HOME=%d  PICK=%d\n", homeAngle, pickAngle);
}

void loop() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    if (cmd == "OPEN")   { Serial.println("OPEN");   goTo(pickAngle); }
    else if (cmd == "CLOSE")  { Serial.println("CLOSE");  goTo(homeAngle); }
    else if (cmd == "CENTRE" || cmd == "CENTER") {
        Serial.println("CENTRE"); goTo(90);
    }
    else if (cmd == "SWEEP") {
        Serial.println("SWEEP HOME->PICK->HOME");
        sweep(homeAngle, pickAngle); delay(500);
        sweep(pickAngle, homeAngle);
        Serial.println("Done.");
    }
    else if (cmd == "STATUS") {
        Serial.printf("HOME=%d  PICK=%d  CUR=%d\n", homeAngle, pickAngle, curAngle);
    }
    else if (cmd.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4);
        if (sp < 0) { Serial.println("SET HOME/PICK/ANGLE <value>"); return; }
        String k = cmd.substring(4, sp);
        int    v = cmd.substring(sp + 1).toInt();
        if      (k == "HOME")  { homeAngle = constrain(v, 0, 180); Serial.printf("HOME=%d\n", homeAngle); }
        else if (k == "PICK")  { pickAngle = constrain(v, 0, 180); Serial.printf("PICK=%d\n", pickAngle); }
        else if (k == "ANGLE") { goTo(v); }
        else Serial.println("Unknown. Use HOME PICK ANGLE");
    }
    else { Serial.println("? OPEN CLOSE CENTRE SWEEP STATUS SET HOME/PICK/ANGLE <val>"); }
}
