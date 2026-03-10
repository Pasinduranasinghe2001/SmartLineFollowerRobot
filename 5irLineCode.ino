/*
  5 IR SENSOR LINE FOLLOWER
  SPECIAL GEOMETRY VERSION for your robot

  Robot:
    - 3 wheels
    - 2 back motor wheels
    - 1 front normal wheel (caster)
    - 5 IR sensors at front

  REAL SENSOR SPACING:
    S1 --1.5cm-- S2 --1.5cm-- S3 ----4.5cm---- S4 --1.5cm-- S5

  Because S3 and S4 have a large gap due to the front wheel,
  left/right logic is NOT symmetric.

  White line on black surface:
    WHITE => analog value <= WHITE_MAX
    BLACK => analog value >  WHITE_MAX
*/

//////////////////// IR ////////////////////
const int IR_S1 = A0;   // far left
const int IR_S2 = A1;   // left
const int IR_S3 = A2;   // center-left
const int IR_S4 = A3;   // center-right (after big gap)
const int IR_S5 = A4;   // far right

const int WHITE_MAX = 200;
bool isWhite(int v) { return v <= WHITE_MAX; }

//////////////////// L298N ////////////////////
const int ENA = 5;   // LEFT motor PWM
const int IN1 = 22;
const int IN2 = 23;

const int ENB = 6;   // RIGHT motor PWM
const int IN3 = 24;
const int IN4 = 25;

#define LEFT_MOTOR_INVERT  false
#define RIGHT_MOTOR_INVERT false

//////////////////// SPEED ////////////////////
const int BASE_SPEED        = 70;
const int FAST_SPEED        = 95;
const int SLOW_SPEED        = 55;
const int TURN_SPEED        = 85;
const int SHARP_TURN_SPEED  = 95;
const int SEARCH_SPEED      = 70;
const int RECOVER_SPEED     = 75;

//////////////////// MOTOR HELPERS ////////////////////
void leftForward(int spd) {
  spd = constrain(spd, 0, 255);
  if (!LEFT_MOTOR_INVERT) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                    { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  analogWrite(ENA, spd);
}

void rightForward(int spd) {
  spd = constrain(spd, 0, 255);
  if (!RIGHT_MOTOR_INVERT) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else                     { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  analogWrite(ENB, spd);
}

void leftBackward(int spd) {
  spd = constrain(spd, 0, 255);
  if (!LEFT_MOTOR_INVERT) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else                    { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(ENA, spd);
}

void rightBackward(int spd) {
  spd = constrain(spd, 0, 255);
  if (!RIGHT_MOTOR_INVERT) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else                     { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(ENB, spd);
}

void stopLeft() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void stopRight() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  stopLeft();
  stopRight();
}

void bothForward(int spd) {
  leftForward(spd);
  rightForward(spd);
}

// gentle left
void steerLeft(int fastSpd, int slowSpd) {
  leftForward(slowSpd);
  rightForward(fastSpd);
}

// gentle right
void steerRight(int fastSpd, int slowSpd) {
  leftForward(fastSpd);
  rightForward(slowSpd);
}

// hard pivot left
void pivotLeft(int spd) {
  leftBackward(spd);
  rightForward(spd);
}

// hard pivot right
void pivotRight(int spd) {
  leftForward(spd);
  rightBackward(spd);
}

//////////////////// RECOVERY ////////////////////
enum RecMode { REC_NONE, REC_LEFT, REC_RIGHT };
RecMode recMode = REC_NONE;
RecMode lastEdge = REC_NONE;

unsigned long recStartMs = 0;
const unsigned long REC_TIMEOUT_MS = 1600;

// because your right side has big sensor gap,
// remember this as a special pre-right-turn condition
bool rightPreLoss = false;

//////////////////// SETUP ////////////////////
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopMotors();
}

//////////////////// LOOP ////////////////////
void loop() {
  // ---- read sensors ----
  int v1 = analogRead(IR_S1);
  int v2 = analogRead(IR_S2);
  int v3 = analogRead(IR_S3);
  int v4 = analogRead(IR_S4);
  int v5 = analogRead(IR_S5);

  bool S1 = isWhite(v1);
  bool S2 = isWhite(v2);
  bool S3 = isWhite(v3);
  bool S4 = isWhite(v4);
  bool S5 = isWhite(v5);

  // debug
  Serial.print(S1); Serial.print(" ");
  Serial.print(S2); Serial.print(" ");
  Serial.print(S3); Serial.print(" ");
  Serial.print(S4); Serial.print(" ");
  Serial.println(S5);

  bool allDark = (!S1 && !S2 && !S3 && !S4 && !S5);

  // -------------------------------------------------
  // TRACK LAST EDGE
  // -------------------------------------------------
  // Left side remembered when white seen on S1/S2/S3
  if (S1 || S2) lastEdge = REC_LEFT;

  // Right side remembered when white seen strongly on S4/S5
  if (S4 || S5) lastEdge = REC_RIGHT;

  // Center lock
  if (S3 && !S1 && !S2 && !S4 && !S5) {
    lastEdge = REC_NONE;
  }

  // -------------------------------------------------
  // SPECIAL RIGHT PRE-LOSS DETECTION
  // Because of large S3-S4 gap, right turn can happen like:
  //   0 0 1 1 0
  //   0 0 0 1 0
  //   0 0 0 0 0
  // or
  //   0 0 1 0 1
  //   0 0 0 0 1
  //   0 0 0 0 0
  // -------------------------------------------------
  if ((!S1 && !S2 && !S3 && S4) ||
      (!S1 && !S2 && !S3 && S5) ||
      (!S1 && !S2 && !S3 && S4 && S5) ||
      (!S1 && !S2 && S3 && S4) ||
      (!S1 && !S2 && S3 && S5)) {
    rightPreLoss = true;
  }

  // cancel rightPreLoss if center seen again
  if (S3) {
    rightPreLoss = false;
  }

  // -------------------------------------------------
  // ENTER RECOVERY
  // -------------------------------------------------
  if (recMode == REC_NONE && allDark) {
    if (rightPreLoss) {
      recMode = REC_RIGHT;
      recStartMs = millis();
      rightPreLoss = false;
    } else {
      if (lastEdge == REC_LEFT) {
        recMode = REC_LEFT;
        recStartMs = millis();
      } else if (lastEdge == REC_RIGHT) {
        recMode = REC_RIGHT;
        recStartMs = millis();
      }
    }
  }

  // -------------------------------------------------
  // RECOVERY MODE
  // -------------------------------------------------
  if (recMode != REC_NONE) {
    if (S3 || S4) {   // right side has gap, so allow S4 also to end recovery
      recMode = REC_NONE;
    }
    else if (millis() - recStartMs > REC_TIMEOUT_MS) {
      recMode = REC_NONE;
    }
    else {
      if (recMode == REC_LEFT) {
        // left recover
        pivotLeft(RECOVER_SPEED);
      } else if (recMode == REC_RIGHT) {
        // right recover
        pivotRight(RECOVER_SPEED);
      }
      delay(10);
      return;
    }
  }

  // -------------------------------------------------
  // NORMAL LINE FOLLOWING
  // -------------------------------------------------

  // 1) perfect center on S3 only
  if (!S1 && !S2 && S3 && !S4 && !S5) {
    bothForward(BASE_SPEED);
  }

  // 2) slightly left of line -> move right
  else if (!S1 && S2 && S3 && !S4 && !S5) {
    steerRight(FAST_SPEED, SLOW_SPEED);
  }
  else if (!S1 && S2 && !S3 && !S4 && !S5) {
    steerRight(FAST_SPEED, SLOW_SPEED);
  }

  // 3) strongly left -> pivot right
  else if (S1 && !S2 && !S3 && !S4 && !S5) {
    pivotRight(SHARP_TURN_SPEED);
  }
  else if (S1 && S2 && !S3 && !S4 && !S5) {
    pivotRight(SHARP_TURN_SPEED);
  }
  else if (S1 && S2 && S3 && !S4 && !S5) {
    pivotRight(TURN_SPEED);
  }

  // 4) slightly right of line -> move left
  else if (!S1 && !S2 && S3 && S4 && !S5) {
    steerLeft(FAST_SPEED, SLOW_SPEED);
  }
  else if (!S1 && !S2 && !S3 && S4 && !S5) {
    steerLeft(FAST_SPEED, SLOW_SPEED);
  }

  // 5) hard right
  else if (!S1 && !S2 && !S3 && !S4 && S5) {
    pivotLeft(SHARP_TURN_SPEED);
  }
  else if (!S1 && !S2 && !S3 && S4 && S5) {
    pivotLeft(SHARP_TURN_SPEED);
  }

  // 6) because of wide 5cm white line, sometimes broad center
  else if (!S1 && S2 && S3 && S4 && !S5) {
    bothForward(BASE_SPEED);
  }
  else if (S2 && S3 && !S4 && !S5) {
    steerRight(FAST_SPEED, SLOW_SPEED);
  }
  else if (!S1 && !S2 && S3 && S4 && S5) {
    steerLeft(FAST_SPEED, SLOW_SPEED);
  }

  // 7) all white / many white
  else if (S1 && S2 && S3 && S4 && S5) {
    bothForward(BASE_SPEED);
  }

  // 8) right gap-related case
  else if (!S1 && !S2 && S3 && !S4 && S5) {
    pivotLeft(TURN_SPEED);
  }

  // 9) left side extra case
  else if (S1 && !S2 && S3 && !S4 && !S5) {
    pivotRight(TURN_SPEED);
  }

  // 10) all dark search
  else if (allDark) {
    if (lastEdge == REC_LEFT) {
      pivotRight(SEARCH_SPEED);
    } else {
      pivotLeft(SEARCH_SPEED);
    }
  }

  // fallback
  else {
    bothForward(BASE_SPEED);
  }

  delay(10);
}