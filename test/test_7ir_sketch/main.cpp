// =========================================================================
//  test/test_7ir_sketch/main.cpp
//  STANDALONE 7-IR LINE FOLLOWER TEST FIRMWARE
//  -------------------------------------------------------------------------
//  This is a FIRMWARE sketch, not a Unity test.
//  It is compiled by env:test_7ir in platformio.ini.
//
//  Flash with:
//    pio run --environment test_7ir --target upload
//
//  DO NOT run with:  pio test --environment test_7ir
//  (test_7ir is not listed as a test environment)
//
//  Hardware:
//    S1-S5 : GPIO 32, 33, 34, 35, 27  (existing sensors)
//    S6    : GPIO 36  (new outer sensor, VP pin, ADC1_CH0)
//    S7    : GPIO 39  (new outer sensor, VN pin, ADC1_CH3)
//    Motors: L298N same as main project
// =========================================================================
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

// ── 7 IR sensor GPIO pins  (left → right) ──────────────────────────────────
static const int IR_PIN[7] = { 32, 33, 34, 35, 27, 36, 39 };
//                              S1  S2  S3  S4  S5  S6  S7
static const int WEIGHT[7]  = { -6, -4, -2,  0,  2,  4,  6 };

// ── Motor pins (L298N) ───────────────────────────────────────────────────────
static const int PIN_ENA = 5,  PIN_IN1 = 18, PIN_IN2 = 19;
static const int PIN_ENB = 23, PIN_IN3 = 21, PIN_IN4 = 22;

#define LEDC_FREQ  20000
#define LEDC_RES   8
#define LEDC_CH_R  0
#define LEDC_CH_L  1

// ── EEPROM ───────────────────────────────────────────────────────────────────────
#define EEPROM_SZ  512
#define MAGIC_BYTE 0xB1
#define ADDR_MAGIC 0
#define ADDR_CAL   4
#define ADDR_PARAMS 100

// ── Tunable parameters ────────────────────────────────────────────────────────
struct Params {
    int   baseSpeed;       // 160
    int   minSpeed;        //  40
    int   searchSpeed;     // 100
    int   reverseSpeed;    //  90
    int   forwardRecSpeed; // 100
    unsigned long timeoutMs; // 2000
    float kp;              //  12.0  (lower than 5-IR due to wider weight span)
    float kd;              //   4.0
    float posFilter;       //   0.50
    float speedDrop;       //   4.0
    int   leftTrim;        //   0
    int   rightTrim;       //   0
};
Params P = { 160, 40, 100, 90, 100, 2000UL, 12.0f, 4.0f, 0.50f, 4.0f, 0, 0 };

// ── Sensor state ─────────────────────────────────────────────────────────────────
int  calBlack[7], calWhite[7], calThresh[7];
int  irRaw[7], irStr[7];
bool irOn[7];

// ── PID state ────────────────────────────────────────────────────────────────────
static float _filt = 0, _lastErr = 0, _lastPos = 0;

// ── Recovery state ───────────────────────────────────────────────────────────────
enum SeenSide  { SIDE_UNKNOWN, SIDE_LEFT, SIDE_CENTER, SIDE_RIGHT };
enum RecovMode { REC_IDLE, REC_REVERSE, REC_PIVOT };
SeenSide  lastSeen  = SIDE_UNKNOWN;
RecovMode recovMode = REC_IDLE;
unsigned long recovStart = 0;

// =========================================================================
//  MOTOR HELPERS
// =========================================================================
static void _pwmR(int v) { ledcWrite(LEDC_CH_R, constrain(v,0,255)); }
static void _pwmL(int v) { ledcWrite(LEDC_CH_L, constrain(v,0,255)); }
static void rFwd(int s)  { s=constrain(s+P.rightTrim,0,255); _pwmR(0); digitalWrite(PIN_IN1,LOW);  digitalWrite(PIN_IN2,HIGH); _pwmR(s); }
static void rBwd(int s)  { s=constrain(s+P.rightTrim,0,255); _pwmR(0); digitalWrite(PIN_IN1,HIGH); digitalWrite(PIN_IN2,LOW);  _pwmR(s); }
static void lFwd(int s)  { s=constrain(s+P.leftTrim, 0,255); _pwmL(0); digitalWrite(PIN_IN3,LOW);  digitalWrite(PIN_IN4,HIGH); _pwmL(s); }
static void lBwd(int s)  { s=constrain(s+P.leftTrim, 0,255); _pwmL(0); digitalWrite(PIN_IN3,HIGH); digitalWrite(PIN_IN4,LOW);  _pwmL(s); }
static void motorsStop() { ledcWrite(LEDC_CH_R,0); ledcWrite(LEDC_CH_L,0); digitalWrite(PIN_IN1,LOW); digitalWrite(PIN_IN2,LOW); digitalWrite(PIN_IN3,LOW); digitalWrite(PIN_IN4,LOW); }
static void fwd(int s)        { rFwd(s);  lFwd(s); }
static void back(int s)       { rBwd(s);  lBwd(s); }
static void pivotLeft(int s)  { rFwd(s);  lBwd(s); }
static void pivotRight(int s) { rBwd(s);  lFwd(s); }

// =========================================================================
//  SENSOR HELPERS
// =========================================================================
static void readSensors() {
    for (int i=0;i<7;i++) {
        int r=analogRead(IR_PIN[i]); irRaw[i]=r; irOn[i]=(r<=calThresh[i]);
        int b=calBlack[i], w=calWhite[i];
        if (b<=w+20) b=w+200;
        irStr[i]=(int)constrain(map(r,b,w,0,1000),0,1000);
    }
}
static bool allDark() { for(int i=0;i<7;i++) if(irOn[i]) return false; return true; }
static bool anyOn()   { for(int i=0;i<7;i++) if(irOn[i]) return true;  return false; }
static bool centreFound() { return irOn[2]||irOn[3]||irOn[4]; }
static void updateLastSeen() {
    if (irOn[0]||irOn[1])  lastSeen = SIDE_LEFT;
    if (irOn[5]||irOn[6])  lastSeen = SIDE_RIGHT;
    if (!irOn[0]&&!irOn[1]&&!irOn[5]&&!irOn[6]&&irOn[3]) lastSeen = SIDE_CENTER;
}
static float computePos() {
    long ws=0,tot=0;
    for(int i=0;i<7;i++){ws+=(long)WEIGHT[i]*(long)irStr[i];tot+=(long)irStr[i];}
    if(tot<50) return _lastPos;
    _lastPos=(float)ws/(float)tot; return _lastPos;
}

// =========================================================================
//  EEPROM
// =========================================================================
static void saveEEPROM() {
    EEPROM.write(ADDR_MAGIC,MAGIC_BYTE);
    int a=ADDR_CAL;
    for(int i=0;i<7;i++){EEPROM.put(a,calBlack[i]);a+=4;EEPROM.put(a,calWhite[i]);a+=4;EEPROM.put(a,calThresh[i]);a+=4;}
    EEPROM.put(ADDR_PARAMS,P); EEPROM.commit();
    Serial.println(F("[EEPROM] Saved."));
}
static bool loadEEPROM() {
    if(EEPROM.read(ADDR_MAGIC)!=MAGIC_BYTE) return false;
    int a=ADDR_CAL;
    for(int i=0;i<7;i++){EEPROM.get(a,calBlack[i]);a+=4;EEPROM.get(a,calWhite[i]);a+=4;EEPROM.get(a,calThresh[i]);a+=4;}
    EEPROM.get(ADDR_PARAMS,P); Serial.println(F("[EEPROM] Loaded.")); return true;
}

// =========================================================================
//  CALIBRATION
// =========================================================================
static void calibrate() {
    Serial.println(F("\n=== 7-IR CAL === PHASE 1: ALL sensors over FLOOR. Send any key..."));
    while(!Serial.available()) delay(50); while(Serial.available()) Serial.read();
    long aB[7]={};
    for(int r=0;r<80;r++){for(int i=0;i<7;i++) aB[i]+=analogRead(IR_PIN[i]); delay(10);}
    for(int i=0;i<7;i++) calBlack[i]=(int)(aB[i]/80);

    Serial.println(F("PHASE 2: ALL sensors over LINE. Send any key..."));
    while(!Serial.available()) delay(50); while(Serial.available()) Serial.read();
    long aW[7]={};
    for(int r=0;r<80;r++){for(int i=0;i<7;i++) aW[i]+=analogRead(IR_PIN[i]); delay(10);}
    for(int i=0;i<7;i++){calWhite[i]=(int)(aW[i]/80);calThresh[i]=(calBlack[i]+calWhite[i])/2;}

    Serial.println(F("=== CAL DONE ==="));
    for(int i=0;i<7;i++)
        Serial.printf("  S%d GPIO%2d  Floor=%4d  Line=%4d  Thresh=%4d\n",
                      i+1,IR_PIN[i],calBlack[i],calWhite[i],calThresh[i]);
    Serial.println(F("Send SAVE."));
}

// =========================================================================
//  STATUS PRINT
// =========================================================================
static void printStatus() {
    readSensors();
    Serial.println(F("\n─── 7-IR STATUS ───"));
    Serial.printf("BASE=%d MIN=%d KP=%.2f KD=%.2f FILTER=%.2f DROP=%.2f\n",
                  P.baseSpeed,P.minSpeed,P.kp,P.kd,P.posFilter,P.speedDrop);
    Serial.printf("SEARCH=%d REVERSE=%d FWDREC=%d TIMEOUT=%lu\n",
                  P.searchSpeed,P.reverseSpeed,P.forwardRecSpeed,P.timeoutMs);
    Serial.printf("LTRIM=%d RTRIM=%d\n",P.leftTrim,P.rightTrim);
    for(int i=0;i<7;i++)
        Serial.printf("  S%d(G%2d) raw=%4d str=%4d on=%d  B=%4d W=%4d T=%4d\n",
                      i+1,IR_PIN[i],irRaw[i],irStr[i],irOn[i]?1:0,
                      calBlack[i],calWhite[i],calThresh[i]);
    Serial.print(F("  Bitmap: "));
    for(int i=0;i<7;i++) Serial.print(irOn[i]?"X":".");
    Serial.printf("  pos=%+.2f\n",computePos());
}

// =========================================================================
//  SERIAL COMMANDS
// =========================================================================
static void handleSerial() {
    if(!Serial.available()) return;
    String cmd=Serial.readStringUntil('\n'); cmd.trim(); cmd.toUpperCase();
    if(cmd=="CALIBRATE"){calibrate();return;}
    if(cmd=="STATUS")   {printStatus();return;}
    if(cmd=="SAVE")     {saveEEPROM();return;}
    if(cmd=="LOAD")     {if(loadEEPROM())printStatus();else Serial.println(F("No data."));return;}
    if(cmd=="HELP"){
        Serial.println(F("CALIBRATE | STATUS | SAVE | LOAD"));
        Serial.println(F("SET BASE|MIN|SEARCH|REVERSE|FWDREC|TIMEOUT|KP|KD|FILTER|DROP|LTRIM|RTRIM VALUE"));
        return;
    }
    if(cmd.startsWith("SET ")){
        int sp=cmd.indexOf(' ',4); if(sp<0){Serial.println(F("SET KEY VALUE"));return;}
        String k=cmd.substring(4,sp), v=cmd.substring(sp+1); v.trim();
        bool ok=true;
        if(k=="BASE")    P.baseSpeed       =v.toInt();
        else if(k=="MIN")     P.minSpeed        =v.toInt();
        else if(k=="SEARCH")  P.searchSpeed     =v.toInt();
        else if(k=="REVERSE") P.reverseSpeed    =v.toInt();
        else if(k=="FWDREC")  P.forwardRecSpeed =v.toInt();
        else if(k=="TIMEOUT") P.timeoutMs       =(unsigned long)v.toInt();
        else if(k=="KP")      P.kp              =v.toFloat();
        else if(k=="KD")      P.kd              =v.toFloat();
        else if(k=="FILTER")  P.posFilter       =v.toFloat();
        else if(k=="DROP")    P.speedDrop       =v.toFloat();
        else if(k=="LTRIM")   P.leftTrim        =v.toInt();
        else if(k=="RTRIM")   P.rightTrim       =v.toInt();
        else ok=false;
        if(ok) Serial.printf("[SET] %s=%s\n",k.c_str(),v.c_str());
        else   Serial.printf("Unknown: %s\n",k.c_str());
        return;
    }
    Serial.println(F("? type HELP"));
}

// =========================================================================
//  PID FOLLOW
// =========================================================================
static void followLine() {
    float raw=computePos();
    _filt=P.posFilter*_filt+(1.0f-P.posFilter)*raw;
    float err=_filt, deriv=err-_lastErr, corr=P.kp*err+P.kd*deriv;
    _lastErr=err;
    int base=constrain(P.baseSpeed-(int)(fabsf(err)*P.speedDrop),P.minSpeed,P.baseSpeed);
    lFwd(constrain(base+(int)corr,P.minSpeed,255));
    rFwd(constrain(base-(int)corr,P.minSpeed,255));
}

// =========================================================================
//  LOST-LINE RECOVERY
// =========================================================================
static void runRecovery() {
    if(recovMode==REC_IDLE){recovMode=REC_REVERSE;recovStart=millis();}
    if(recovMode==REC_REVERSE){
        if(lastSeen==SIDE_RIGHT){rBwd(P.reverseSpeed+10);lBwd(P.reverseSpeed-10);}
        else if(lastSeen==SIDE_LEFT){rBwd(P.reverseSpeed-10);lBwd(P.reverseSpeed+10);}
        else back(P.reverseSpeed);
        readSensors();
        if(anyOn()){recovMode=REC_IDLE;return;}
        if(millis()-recovStart>400UL){recovMode=REC_PIVOT;recovStart=millis();}
        return;
    }
    if(recovMode==REC_PIVOT){
        if(lastSeen==SIDE_LEFT)  pivotLeft(P.searchSpeed);
        else if(lastSeen==SIDE_RIGHT) pivotRight(P.searchSpeed);
        else pivotLeft(P.searchSpeed);
        readSensors();
        if(centreFound()||anyOn()){recovMode=REC_IDLE;return;}
        if(millis()-recovStart>P.timeoutMs){motorsStop();recovMode=REC_IDLE;Serial.println(F("[REC] timeout"));}
    }
}

// =========================================================================
//  VERBOSE PRINT
// =========================================================================
static unsigned long _lastPrint=0;
static void verbPrint(){
    if(millis()-_lastPrint<150) return; _lastPrint=millis();
    Serial.printf("[T] ");
    for(int i=0;i<7;i++) Serial.print(irOn[i]?"X":".");
    Serial.printf("  pos=%+.2f  rec=%d\n",computePos(),recovMode!=REC_IDLE?1:0);
}

// =========================================================================
//  SETUP
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("\n[BOOT] 7-IR Line Follower Test Firmware"));
    Serial.println(F("  S6=GPIO36  S7=GPIO39  (new outer sensors)"));
    Serial.println(F("  Flash env: test_7ir     Serial: 115200"));

    EEPROM.begin(EEPROM_SZ);
    pinMode(PIN_IN1,OUTPUT); pinMode(PIN_IN2,OUTPUT);
    pinMode(PIN_IN3,OUTPUT); pinMode(PIN_IN4,OUTPUT);
    ledcSetup(LEDC_CH_R,LEDC_FREQ,LEDC_RES); ledcAttachPin(PIN_ENA,LEDC_CH_R);
    ledcSetup(LEDC_CH_L,LEDC_FREQ,LEDC_RES); ledcAttachPin(PIN_ENB,LEDC_CH_L);
    motorsStop();
    for(int i=0;i<7;i++) pinMode(IR_PIN[i],INPUT);
    for(int i=0;i<7;i++){calBlack[i]=3600;calWhite[i]=400;calThresh[i]=2000;}

    if(!loadEEPROM()) Serial.println(F("[BOOT] No EEPROM data. Run CALIBRATE."));
    else printStatus();

    _filt=0; _lastErr=0; recovMode=REC_IDLE; lastSeen=SIDE_UNKNOWN;
    Serial.println(F("[BOOT] Commands: CALIBRATE | STATUS | SAVE | HELP"));
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
    verbPrint();
    if(!allDark()){ if(recovMode!=REC_IDLE) recovMode=REC_IDLE; followLine(); }
    else runRecovery();
    delay(5);
}
