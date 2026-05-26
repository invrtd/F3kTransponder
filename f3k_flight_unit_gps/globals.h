#pragma once
#include <Arduino.h>
#include "types.h"
#include "conf.h"

Config cfg = { DEFAULT_UNIT_ID, true, 240, 0 };  // safe defaults

FlightData flight = {DEFAULT_UNIT_ID, STATE_CALIBRATING, 0, 0, 0, 0, 0, 100};

ImuData imu = {0, 0, 0, 0, 0, 0, 0, 0, false};

// Altitude at last landing — baseline for throw height calculation
float last_landed_alt_ft = 0.0f;

// ── Sensor buffer (for 1 Hz web display batch) ───────────────
float buf_pressure[BUF_SIZE];
float buf_temperature[BUF_SIZE];
int   buf_count = 0;

// Index matches FlightState enum values per ICD v1.7
const char* stateNames[] = {
  "CALIBRATING", "GROUND", "LAUNCH_WIN", "FLIGHT", "LANDED"
};


bool imu_present = false;             // set false if IMU not found at boot
bool dps_present = false;             // set false if DPS310 not found after retries
float tare_baseline_ft       = 0.0;
volatile bool tare_requested = false;



// ── GPS state (PA1010D — updated ~1 Hz when fix valid) ───────
// gps object uses Wire1 unconditionally — see GPS_I2C_ADDR define above.
Adafruit_GPS  gps_sensor(&Wire1);
bool          gps_present    = false;  // false if not found at boot
bool          gps_fix        = false;  // true when fix_quality > 0
uint8_t       gps_fix_quality = 0;     // 0=none 1=GPS 2=DGPS 6=estimated
uint8_t       gps_sats       = 0;      // satellites used in fix
float         gps_hdop       = 99.9f;  // horizontal dilution of precision
float         gps_lat        = 0.0f;   // decimal degrees, positive=N
float         gps_lon        = 0.0f;   // decimal degrees, positive=E
float         gps_alt_m      = 0.0f;   // MSL altitude, metres
unsigned long last_gps_tx_ms = 0;      // last Packet 2 transmit timestamp



// ── Network credentials ──────────────────────────────────────


// ── Scorer server (where UDP packets are sent) ───────────────
// Update SERVER_IP to match your laptop's IP on the field network
const char*    SERVER_IP    = "192.168.8.101";
const uint16_t UDP_PORT     = 5005;           // Packet 1 — scoring
const uint16_t UDP_DBG_PORT = 4213;           // Packet 4 — debug/health
const uint16_t UDP_ANN_PORT = 4214;           // Packet 5 — log announcement
const uint16_t UDP_WIN_PORT = 5006;           // Window command (scorer→unit)
const uint16_t UDP_HZ         = 5;
const uint32_t UDP_INTERVAL   = 1000 / UDP_HZ;        // 200ms — active flight
const uint32_t UDP_INTERVAL_SLOW = 1000;               // 1 Hz  — ground/standby

// ── Sensor / display timing ──────────────────────────────────
const float    SEA_LEVEL_HPA     = 1013.25;
const float    M_TO_FT           = 3.28084;
const uint32_t SENSOR_INTERVAL   = 125;   // ms — 8 Hz
const uint32_t IMU_INTERVAL      = 38;    // ms — 26 Hz (1000/26 ≈ 38ms)
const uint32_t DISPLAY_INTERVAL  = 200;   // ms — 5 Hz web update
const uint32_t DISPLAY_OFFSET_MS = 100;   // ms stagger from scorer packet
