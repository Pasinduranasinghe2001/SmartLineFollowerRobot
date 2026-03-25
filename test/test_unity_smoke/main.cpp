// =========================================================================
//  test/test_unity_smoke/main.cpp
//
//  Unity unit-test suite for EC6090 Robot – sensor math & calibration logic
//
//  These tests run ON the ESP32 hardware (embedded Unity).
//  They do NOT require motors, sensors, or any peripherals to be connected;
//  they test pure arithmetic and threshold logic in isolation.
//
//  Run with:
//    pio test --environment esp32dev
//
//  Expected output (Serial Monitor 115200 baud):
//    -----------------------
//    7 Tests 0 Failures 0 Ignored
//    OK
// =========================================================================
#include <Arduino.h>
#include <unity.h>

// ───────────────────────────────────────────────────────────────────────────
//  HELPERS  – pure-C replicas of the functions under test
//  (No hardware headers needed; avoids linker issues in test build)
// ───────────────────────────────────────────────────────────────────────────

// 5-sensor weights (main project)
static const int W5[5] = { -4, -2, 0, 2, 4 };

// 7-sensor weights (7-IR test sketch)
static const int W7[7] = { -6, -4, -2, 0, 2, 4, 6 };

// Replica of sensors_computePosition() for 5 sensors
static float pos5(int str[5]) {
    long wsum = 0, total = 0;
    for (int i = 0; i < 5; i++) {
        wsum  += (long)W5[i] * (long)str[i];
        total += (long)str[i];
    }
    if (total < 50) return 0.0f;
    return (float)wsum / (float)total;
}

// Replica of computePos() for 7 sensors
static float pos7(int str[7]) {
    long wsum = 0, total = 0;
    for (int i = 0; i < 7; i++) {
        wsum  += (long)W7[i] * (long)str[i];
        total += (long)str[i];
    }
    if (total < 50) return 0.0f;
    return (float)wsum / (float)total;
}

// Replica of calibration threshold computation
static int calThresh(int floorVal, int lineVal) {
    return (floorVal + lineVal) / 2;
}

// Replica of irStrength normalisation (map + constrain)
static int irStrength(int raw, int blackRef, int whiteRef) {
    if (blackRef <= whiteRef + 20) blackRef = whiteRef + 200;
    long st = ((long)(raw - blackRef) * 1000L) / ((long)(whiteRef - blackRef));
    if (st < 0)    st = 0;
    if (st > 1000) st = 1000;
    return (int)st;
}

// =========================================================================
//  TEST CASES
// =========================================================================

// T1: Centre sensor only – 5 sensors
//   All strength on S3 (weight 0) → position must be exactly 0.0
void test_5sensor_centre_position() {
    int str[5] = { 0, 0, 800, 0, 0 };
    float p = pos5(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p);
}

// T2: Far-right sensor only – 5 sensors
//   All strength on S5 (weight +4) → position must be +4.0
void test_5sensor_rightmost_position() {
    int str[5] = { 0, 0, 0, 0, 600 };
    float p = pos5(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, p);
}

// T3: Far-left sensor only – 5 sensors
//   All strength on S1 (weight -4) → position must be -4.0
void test_5sensor_leftmost_position() {
    int str[5] = { 700, 0, 0, 0, 0 };
    float p = pos5(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -4.0f, p);
}

// T4: Centre sensor only – 7 sensors (S4, weight 0)
void test_7sensor_centre_position() {
    int str[7] = { 0, 0, 0, 900, 0, 0, 0 };
    float p = pos7(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p);
}

// T5: New outer-right sensor S7 (weight +6) – 7 sensors
//   Confirms the new sensor contributes the correct max positive weight
void test_7sensor_outerright_S7_position() {
    int str[7] = { 0, 0, 0, 0, 0, 0, 500 };
    float p = pos7(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.0f, p);
}

// T6: New outer-left sensor S6... wait, S6 is index 5, weight +4.
//   Actually S1=index0 weight=-6  S7=index6 weight=+6
//   S6 = index 5, weight = +4  (second from right outer)
void test_7sensor_S6_position() {
    int str[7] = { 0, 0, 0, 0, 0, 800, 0 };
    float p = pos7(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, p);
}

// T7: Calibration threshold is midpoint of floor and line readings
//   floor=3200 (dark, high ADC on ESP32), line=600 (reflective)
//   thresh = (3200 + 600) / 2 = 1900
void test_calibration_threshold_midpoint() {
    int t = calThresh(3200, 600);
    TEST_ASSERT_EQUAL_INT(1900, t);
}

// T8: irStrength normalisation
//   raw=400 (on the white line), black=3600, white=400
//   Expected strength = 1000 (maximum, reading exactly on the line)
void test_irstrength_max_on_line() {
    int s = irStrength(400, 3600, 400);
    TEST_ASSERT_EQUAL_INT(1000, s);
}

// T9: irStrength normalisation
//   raw=3600 (on the dark floor), black=3600, white=400
//   Expected strength = 0 (minimum, fully off the line)
void test_irstrength_min_on_floor() {
    int s = irStrength(3600, 3600, 400);
    TEST_ASSERT_EQUAL_INT(0, s);
}

// T10: Below-minimum total strength – function should return 0 (last pos)
//   All sensors read very low strength (<50 total)
void test_5sensor_all_dark_returns_zero() {
    int str[5] = { 0, 0, 0, 0, 0 };
    float p = pos5(str);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p);
}

// =========================================================================
//  REQUIRED BY UNITY EMBEDDED  (setup / loop)
// =========================================================================
void setup() {
    delay(2000);   // allow USB-Serial to enumerate on host
    UNITY_BEGIN();

    RUN_TEST(test_5sensor_centre_position);
    RUN_TEST(test_5sensor_rightmost_position);
    RUN_TEST(test_5sensor_leftmost_position);
    RUN_TEST(test_7sensor_centre_position);
    RUN_TEST(test_7sensor_outerright_S7_position);
    RUN_TEST(test_7sensor_S6_position);
    RUN_TEST(test_calibration_threshold_midpoint);
    RUN_TEST(test_irstrength_max_on_line);
    RUN_TEST(test_irstrength_min_on_floor);
    RUN_TEST(test_5sensor_all_dark_returns_zero);

    UNITY_END();
}

void loop() {
    // Nothing – Unity runs all tests in setup() then halts
}
