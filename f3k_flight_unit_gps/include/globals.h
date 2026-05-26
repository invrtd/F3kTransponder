#pragma once

#include <Arduino.h>
#include <FS.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>

#include "../conf.h"
#include "../types.h"

// ============================================================
// globals.h
//
// Declarations only.
// Actual storage/initializers belong in globals.cpp.
// ============================================================

// ── Core runtime state ───────────────────────────────────────
extern Config cfg;
extern FlightData flight;
extern ImuData imu;

// Altitude at last landing — baseline for throw height calculation
extern float last_landed_alt_ft;

// ── Sensor buffer for 1 Hz web display batch ─────────────────
extern float buf_pressure[BUF_SIZE];
extern float buf_temperature[BUF_SIZE];
extern int   buf_count;

// Index matches FlightState enum values per ICD v1.7
extern const char* stateNames[];

// Tare / baseline
extern float tare_baseline_ft;
extern volatile bool tare_requested;

// ── Scorer server ────────────────────────────────────────────
// Update SERVER_IP in globals.cpp to match your laptop's IP
extern const char* SERVER_IP;

// These are constants, safe to keep in the header.
static const uint16_t UDP_PORT          = 5005;  // Packet 1 — scoring
static const uint16_t UDP_DBG_PORT      = 4213;  // Packet 4 — debug/health
static const uint16_t UDP_ANN_PORT      = 4214;  // Packet 5 — log announcement
static const uint16_t UDP_WIN_PORT      = 5006;  // Window command
static const uint16_t UDP_HZ            = 5;
static const uint32_t UDP_INTERVAL      = 1000 / UDP_HZ;
static const uint32_t UDP_INTERVAL_SLOW = 1000;

// ── Sensor / display timing ──────────────────────────────────
static const float    SEA_LEVEL_HPA     = 1013.25f;
static const float    M_TO_FT           = 3.28084f;
static const uint32_t SENSOR_INTERVAL   = 125;  // ms — 8 Hz
static const uint32_t IMU_INTERVAL      = 38;   // ms — 26 Hz
static const uint32_t DISPLAY_INTERVAL  = 200;  // ms — 5 Hz web update
static const uint32_t DISPLAY_OFFSET_MS = 100;  // ms stagger from scorer packet

// ── Deferred window prep/open ────────────────────────────────
extern bool          prep_active;
extern unsigned long prep_fire_ms;
extern uint16_t      prep_window_secs;
extern uint32_t      prep_window_id;

// ── Window runtime state ─────────────────────────────────────
extern bool          window_active;
extern uint16_t      window_secs;
extern uint32_t      window_id;
extern unsigned long window_start_ms;
extern unsigned long window_close_ms;
extern volatile bool window_open_pending;
extern volatile bool window_close_pending;

// ── Deferred WiFi shutdown ───────────────────────────────────
extern bool          wifi_shutdown_pending;
extern unsigned long wifi_shutdown_after_ms;

// ── Logging state ────────────────────────────────────────────
extern unsigned long log_epoch_ms;
extern unsigned long flight_start_epoch_ms;
extern bool          log_open;
extern volatile int  littlefs_streaming;
extern char          log_path[32];
extern uint16_t      flight_counter;
extern File          log_file;

// ── Logging / pruning flags ──────────────────────────────────
extern bool prune_pending;
extern bool log_preopen_done;

// ── Announce timer — Packet 5 after window closes ────────────
extern bool          announce_pending;
extern uint8_t       announce_count;
extern unsigned long last_announce_ms;
extern uint32_t      announce_file_size;

static const uint8_t  ANNOUNCE_REPEATS     = 5;
static const uint32_t ANNOUNCE_INTERVAL_MS = 2000;

// ── Diagnostics ──────────────────────────────────────────────
extern unsigned long loop_count;
extern unsigned long busy_us_total;
extern unsigned long loop_us_total;
extern unsigned long loop_max_us;

struct DiagSnap {
  int      rssi_dbm;
  float    cpu_load_pct;
  float    loop_avg_us;
  float    loop_max_us;
  uint32_t free_heap;
  int      sample_count;
};

extern DiagSnap diag;

// ── Timing ───────────────────────────────────────────────────
extern unsigned long last_sensor_ms;
extern unsigned long last_imu_ms;
extern unsigned long last_gps_ms;
extern unsigned long last_gps_tx_ms;
extern unsigned long last_display_ms;
extern unsigned long last_udp_ms;
extern unsigned long last_dbg_ms;
extern unsigned long last_diag_ms;

// Live temperature from DPS310
extern float live_temperature_c;

// ── Hardware / network objects ───────────────────────────────
// Actual constructors belong in globals.cpp.
extern Adafruit_NeoPixel pixel;
extern AsyncWebServer server;
extern WiFiUDP udp;
extern WiFiUDP udp_win;
extern WiFiUDP udp_gps;

// ── Flight records ───────────────────────────────────────────
constexpr int MAX_FLIGHT_RECORDS = 10;
extern FlightRecord flight_records[MAX_FLIGHT_RECORDS];
extern int flight_record_count;

// ── Calibration state ────────────────────────────────────────
extern float         cal_buf[CAL_BUF_SIZE];
extern int           cal_count;
extern unsigned long cal_start_ms;
extern bool          calibration_done;

// ── Web display double-buffer JSON ───────────────────────────
extern String        batch_json_a;
extern String        batch_json_b;
extern volatile int  active_buf;

// ── Runtime modes ────────────────────────────────────────────
extern bool          tilt_mode;
extern uint8_t       sim_mode;
extern uint8_t       score_mode;

// ── WiFi / AP runtime state ──────────────────────────────────
extern bool          ap_mode;
extern bool          wifi_active;
extern FlightState   prev_flight_state;

// ── Contest context (ICD v1.7) ────────────────────────────────
// Received from scorer in 0x20 bytes [8–10] and 0x21 bytes [10–12].
// Written into CSV comment headers for human-readable context.
extern uint8_t  contest_task_id;   // F3XVault flight_type_id; 0 = unknown/LL
extern uint8_t  contest_round_num; // 1-based; 0 = unknown
extern uint8_t  contest_group_num; // 1-based; 0 = unknown

// Task name lookup table keyed by task_id (F3XVault flight_type_id)
// Returns human-readable string for CSV headers.
const char* taskName(uint8_t tid);

// ── Pilot AP countdown state ─────────────────────────────────
extern bool          window_countdown_active;
extern unsigned long window_countdown_start;
extern uint16_t      window_countdown_secs;

// ── Pilot completed-log download state ───────────────────────
extern String        pilot_download_path;