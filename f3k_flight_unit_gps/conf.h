#pragma once
#include <Arduino.h>
// ── Debug / bench test mode ──────────────────────────────────
// DEBUG_TILT_MODE 0 — normal flight operation (default)
// DEBUG_TILT_MODE 1 — altitude simulated from physical tilt angle:
//     0–15°  → 0 ft (GROUND)   15–45° → 10 ft (LAUNCH_WIN)
//     45–90° → tilt_deg as ft (FLIGHT)   back to 0–15° → LANDED
// DEBUG_TILT_MODE 2 — fully autonomous parabola simulation, no IMU needed:
//     Simulates repeated flights with sinusoidal altitude profile.
//     Flight duration  = unit_id × 10 seconds  (e.g. unit 4 → 40s)
//     Peak altitude    = unit_id × 10 feet      (e.g. unit 4 → 40 ft)
//     Ground pause     = 5 seconds between flights
//     Repeats until window elapsed. Use with auto-window or manual /pstart.
#define DEBUG_TILT_MODE  2   // ← 0=normal  1=tilt  2=parabola sim

// ── I2C port selection ───────────────────────────────────────
// QT Py ESP32-S3 has two I2C buses:
//   USE_STEMMA_QT 1 → Wire1 — STEMMA QT JST connector (SDA=GPIO41, SCL=GPIO40)
//   USE_STEMMA_QT 0 → Wire  — Standard pads           (SDA=GPIO8,  SCL=GPIO9)
// Use STEMMA QT (1) for the JST-SH daisy-chain cable to DPS310 + LSM6DSO32.
#define USE_STEMMA_QT  1   // ← set to 0 for standard I2C pads

// ── GPS (PA1010D #4415 — always Wire1 / STEMMA QT) ───────────
// I2C address 0x10 — no conflict with DPS310 (0x77) or LSM6DSO32 (0x6A).
// GPS is always on Wire1 regardless of USE_STEMMA_QT, because the
// STEMMA QT JST connector is the only practical daisy-chain point.
// NOTE: The PA1010D can clock-stretch Wire1 for 30-65ms while processing
// NMEA data. This affects loop-max. On a custom board, move GPS to a
// dedicated I2C bus to isolate it from the scoring sensors.
#define GPS_I2C_ADDR  0x10

// ── WiFi AP mode (direct phone access without scorer network) ─
// FORCE_AP_MODE 1 → always start as hotspot regardless of F3KBase availability
// FORCE_AP_MODE 0 → try F3KBase first, fall back to AP if not found within 15s
// AP network: open (no password), SSID = "F3K-Unit-XX" where XX = unit_id
// Connect phone to AP, then browse to http://192.168.4.1/logs
#define FORCE_AP_MODE  0   // ← set to 1 to always run as hotspot



// ── LittleFS config ──────────────────────────────────────────
#define CONFIG_PATH     "/f3k_config.json"
#define DEFAULT_UNIT_ID 1



