#pragma once
#include <Arduino.h>
#include "conf.h"



// ── State machine ────────────────────────────────────────────
// State values per F3K ICD v1.7 — must match scorer exactly
enum FlightState : uint8_t {
  STATE_CALIBRATING = 0,
  STATE_GROUND      = 1,
  STATE_LAUNCH_WIN  = 2,
  STATE_FLIGHT      = 3,
  STATE_LANDED      = 4
};




struct Config {
  uint8_t  unit_id;        // 1–200
  bool     web_enabled;    // true = run AsyncWebServer
  uint16_t cpu_mhz;        // 80, 160, or 240
  uint16_t window_number;  // increments each working window, persisted to flash
};

// ── Flight data ──────────────────────────────────────────────
struct FlightData {
  uint8_t      unit_id;
  FlightState  state;
  float        altitude_ft;
  float        launch_height_ft;   // absolute alt at launch trigger
  float        throw_height_ft;    // peak alt gain during LAUNCH_WIN, frozen at FLIGHT
  float        peak_alt_ft;
  uint32_t     flight_duration_ms;
  uint8_t      battery_pct;
} ;


// ── IMU data (LSM6DSO32 — updated at 26 Hz) ─────────────────
struct ImuData {
  float accel_x, accel_y, accel_z;   // m/s²
  float gyro_x,  gyro_y,  gyro_z;    // rad/s
  float g_force;                      // magnitude in G (√(x²+y²+z²) / 9.81)
  float tilt_deg;                     // tilt from vertical in degrees
  bool  valid;
} ;

// ── Pilot session (AP mode data collection) ──────────────────
struct FlightRecord {
  uint16_t      flight_num;
  float         duration_s;
  float         throw_height_ft;
  float         peak_alt_ft;
  float         score;          // mode 0: Secs-Ft — duration_s - throw_height_ft (window-independent)
                                // mode 1: JoeD V1 — (duration_s/180)^0.425*1000 ± height component
  unsigned long start_time_ms;  // millis() at GROUND→LAUNCH_WIN (window-relative)
  unsigned long end_time_ms;    // millis() at landing detection (window-relative)
};