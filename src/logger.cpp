// =========================================================================
//  logger.cpp  -  Offline LittleFS CSV logger
//
//  CSV columns (matches robot_debugLog CSV format exactly):
//  t_ms, rawPos, filtPos, error, derivative, correction,
//  dynBase, effectiveBase, leftSpd, rightSpd, curveMode,
//  curveAboveCount, dist_cm
// =========================================================================
#include <Arduino.h>
#include "logger.h"
#include "config.h"

#if DBG_LOG_TO_FILE

#include <LittleFS.h>

#define LOG_FILE   "/pidlog.csv"

static bool _fsReady    = false;
static bool _hdrWritten = false;

// ── Init ─────────────────────────────────────────────────────────────────
void logger_init() {
    if (!LittleFS.begin(true)) {   // true = format on first use
        Serial.println(F("[LOG] LittleFS mount FAILED - file logging disabled"));
        _fsReady = false;
        return;
    }
    _fsReady = true;

    // Check if an existing log is present (continued run)
    if (LittleFS.exists(LOG_FILE)) {
        File f = LittleFS.open(LOG_FILE, "r");
        size_t sz = f ? f.size() : 0;
        if (f) f.close();
        Serial.printf("[LOG] LittleFS OK. Existing log: %u bytes.\n", sz);
        Serial.println(F("[LOG] Send  CLEARLOG  to wipe before new run."));
        _hdrWritten = true;   // header already in file
    } else {
        Serial.println(F("[LOG] LittleFS OK. No existing log - ready for new run."));
        _hdrWritten = false;
    }
    logger_printInfo();
}

// ── Append one CSV row ────────────────────────────────────────────────────
void logger_appendRow(const PidDebugSnapshot& s) {
    if (!_fsReady) return;

    File f = LittleFS.open(LOG_FILE, "a");
    if (!f) {
        Serial.println(F("[LOG] Cannot open log file for append!"));
        return;
    }

    // Write header row on first write of this session
    if (!_hdrWritten) {
        f.println(F("t_ms,rawPos,filtPos,error,derivative,correction,"
                    "dynBase,effectiveBase,leftSpd,rightSpd,"
                    "curveMode,curveAboveCount,dist_cm"));
        _hdrWritten = true;
    }

    // Write data row  (printf-style via char buffer for speed)
    char buf[128];
    snprintf(buf, sizeof(buf),
             "%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%d,%d,%d,%.1f",
             s.loopMs,
             s.rawPos,
             s.filteredPos,
             s.error,
             s.derivative,
             s.correction,
             s.dynBase,
             s.effectiveBase,
             s.leftSpd,
             s.rightSpd,
             (int)s.inCurveMode,
             s.curveAbove,
             s.dist);
    f.println(buf);
    f.close();
}

// ── Dump entire file to Serial ────────────────────────────────────────────
void logger_dump() {
    if (!_fsReady) {
        Serial.println(F("[LOG] Filesystem not ready."));
        return;
    }
    if (!LittleFS.exists(LOG_FILE)) {
        Serial.println(F("[LOG] No log file found. Run the robot first."));
        return;
    }

    File f = LittleFS.open(LOG_FILE, "r");
    if (!f) {
        Serial.println(F("[LOG] Cannot open log file for reading!"));
        return;
    }

    size_t sz = f.size();
    Serial.printf("[LOG] Dumping %u bytes from %s ...\n", sz, LOG_FILE);
    Serial.println(F("[LOG] ===== BEGIN CSV ====="));

    // Stream in 256-byte chunks to avoid watchdog reset on large files
    uint8_t buf[256];
    while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n > 0) Serial.write(buf, n);
    }
    f.close();

    Serial.println(F("\n[LOG] ===== END CSV ====="));
    Serial.printf("[LOG] Done. %u bytes dumped.\n", sz);
}

// ── Clear log file ────────────────────────────────────────────────────────
void logger_clear() {
    if (!_fsReady) {
        Serial.println(F("[LOG] Filesystem not ready."));
        return;
    }
    if (LittleFS.exists(LOG_FILE)) {
        LittleFS.remove(LOG_FILE);
        Serial.println(F("[LOG] Log file deleted. Ready for new run."));
    } else {
        Serial.println(F("[LOG] No log file to clear."));
    }
    _hdrWritten = false;
}

// ── Print filesystem info ─────────────────────────────────────────────────
void logger_printInfo() {
    if (!_fsReady) {
        Serial.println(F("[LOG] Filesystem not ready."));
        return;
    }
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    size_t free_ = total - used;

    Serial.printf("[LOG] LittleFS: total=%u KB  used=%u KB  free=%u KB\n",
                  total / 1024, used / 1024, free_ / 1024);

    if (LittleFS.exists(LOG_FILE)) {
        File f = LittleFS.open(LOG_FILE, "r");
        if (f) {
            Serial.printf("[LOG] /pidlog.csv: %u bytes\n", f.size());
            f.close();
        }
    } else {
        Serial.println(F("[LOG] /pidlog.csv: (no file)"));
    }
}

#else
// Stubs when DBG_LOG_TO_FILE=0 - zero overhead
void logger_init()       {}
void logger_appendRow(const PidDebugSnapshot&) {}
void logger_dump()       { Serial.println(F("[LOG] File logging disabled (DBG_LOG_TO_FILE=0)")); }
void logger_clear()      { Serial.println(F("[LOG] File logging disabled (DBG_LOG_TO_FILE=0)")); }
void logger_printInfo()  { Serial.println(F("[LOG] File logging disabled (DBG_LOG_TO_FILE=0)")); }
#endif
