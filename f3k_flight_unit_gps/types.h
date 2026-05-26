#pragma once
#include <Arduino.h>
#include "conf.h"



// ── State machine ────────────────────────────────────────────
// State values per F3K ICD v1.0 — must match scorer exactly
enum FlightState : uint8_t {
  STATE_GROUND      = 0,
  STATE_LAUNCH_WIN  = 1,
  STATE_FLIGHT      = 2,
  STATE_LANDED      = 3,
  STATE_CALIBRATING = 4
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

