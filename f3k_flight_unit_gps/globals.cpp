#include "globals.h"

Config cfg = { DEFAULT_UNIT_ID, true, 240, 0 };

FlightData flight = {
  DEFAULT_UNIT_ID,
  STATE_CALIBRATING,
  0, 0, 0, 0, 0,
  100
};

ImuData imu = {0, 0, 0, 0, 0, 0, 0, 0, false};

const char* stateNames[] = {
  "CALIBRATING", "GROUND", "LAUNCH_WIN", "FLIGHT", "LANDED"
};

float last_landed_alt_ft = 0.0f;

float buf_pressure[BUF_SIZE];
float buf_temperature[BUF_SIZE];
int buf_count = 0;

float tare_baseline_ft = 0.0f;
volatile bool tare_requested = false;

const char* SERVER_IP = "192.168.8.101";

bool prep_active = false;
unsigned long prep_fire_ms = 0;
uint16_t prep_window_secs = 0;
uint32_t prep_window_id = 0;

// ── Window runtime state ─────────────────────────────────────
bool          window_active = false;
uint16_t      window_secs = 0;
uint32_t      window_id = 0;
unsigned long window_start_ms = 0;
unsigned long window_close_ms = 0;
volatile bool window_open_pending = false;
volatile bool window_close_pending = false;

bool wifi_shutdown_pending = false;
unsigned long wifi_shutdown_after_ms = 0;

unsigned long log_epoch_ms = 0;
unsigned long flight_start_epoch_ms = 0;
bool log_open = false;
volatile int littlefs_streaming = 0;
char log_path[32];
uint16_t flight_counter = 0;

// ── Logging / pruning flags ──────────────────────────────────
bool prune_pending = false;
bool log_preopen_done = false;

bool announce_pending = false;
uint8_t announce_count = 0;
unsigned long last_announce_ms = 0;
uint32_t announce_file_size = 0;

unsigned long loop_count = 0;
unsigned long busy_us_total = 0;
unsigned long loop_us_total = 0;
unsigned long loop_max_us = 0;

DiagSnap diag = {0, 0, 0, 0, 0, 0};

unsigned long last_sensor_ms = 0;
unsigned long last_imu_ms = 0;
unsigned long last_gps_ms = 0;
unsigned long last_gps_tx_ms = 0;
unsigned long last_display_ms = 0;
unsigned long last_udp_ms = 0;
unsigned long last_dbg_ms = 0;
unsigned long last_diag_ms = 0;

float live_temperature_c = 0.0f;

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
WiFiUDP udp;
WiFiUDP udp_win;
WiFiUDP udp_gps;
File log_file;

// ── Calibration state ────────────────────────────────────────
float         cal_buf[CAL_BUF_SIZE];
int           cal_count = 0;
unsigned long cal_start_ms = 0;
bool          calibration_done = false;

// ── Web display double-buffer JSON ───────────────────────────
String        batch_json_a = "{\"ready\":false}";
String        batch_json_b = "{\"ready\":false}";
volatile int  active_buf = 0;

// ── Runtime modes ────────────────────────────────────────────
#if DEBUG_TILT_MODE
bool    tilt_mode = true;
uint8_t sim_mode  = DEBUG_TILT_MODE;
#else
bool    tilt_mode = false;
uint8_t sim_mode  = 0;
#endif

uint8_t score_mode = 0;

// ── WiFi / AP runtime state ──────────────────────────────────
bool        ap_mode = false;
bool        wifi_active = true;
FlightState prev_flight_state = STATE_CALIBRATING;

// ── Contest context (ICD v1.7) ────────────────────────────────
// Received from scorer in 0x20 bytes [8–10] and 0x21 bytes [10–12].
// Written into CSV comment headers for human-readable context.
uint8_t  contest_task_id   = 0;  // F3XVault flight_type_id; 0 = unknown/LL
uint8_t  contest_round_num = 0;  // 1-based; 0 = unknown
uint8_t  contest_group_num = 0;  // 1-based; 0 = unknown

// ── Pilot AP countdown state ─────────────────────────────────
bool          window_countdown_active = false;
unsigned long window_countdown_start = 0;
uint16_t      window_countdown_secs = 0;

// ── Pilot completed-log download state ───────────────────────
String        pilot_download_path = "";

// ── Flight record history ────────────────────────────────────
FlightRecord flight_records[MAX_FLIGHT_RECORDS];
int flight_record_count = 0;
// Task name lookup table keyed by task_id (F3XVault flight_type_id)
// Returns human-readable string for CSV headers.
const char* taskName(uint8_t tid) {
  switch (tid) {
    case  6: return "Task A - Last Flight (5:00 Max, 7 Min)";
    case  7: return "Task B - Last Two Flights (4:00 Max)";
    case  8: return "Task C - All Up Last Down x3";
    case  9: return "Task D - Ladder (7 rungs: 0:30-2:00)";
    case 10: return "Task E - Poker (5 targets, 10 Min)";
    case 11: return "Task F - Three of Six (3:00 Max)";
    case 12: return "Task G - Five Longest (2:00 Max)";
    case 13: return "Task H - 1,2,3,4 Minute";
    case 14: return "Task I - Three Longest (3:20 Max)";
    case 15: return "Task J - Last Three (3:00 Max)";
    case 17: return "Task C - All Up Last Down x4";
    case 18: return "Task C - All Up Last Down x5";
    case 19: return "Task A - Last Flight (5:00 Max, 10 Min)";
    case 20: return "Task B - Last Two Flights (3:00 Max, 7 Min)";
    case 21: return "Task K - Big Ladder";
    case 26: return "Task D (2020) - Two Flights (5:00 Max)";
    case 27: return "Task E (2020) - Poker 3 Flights, 10 Min";
    case 28: return "Task E (2020) - Poker 3 Flights, 15 Min";
    case 29: return "Task L - Single Flight (9:59 Max)";
    case 30: return "Task M - Huge Ladder (3, 5, 7 Min)";
    case 33: return "Task N - Best Flight (9:59 Max)";
    case 34: return "Task L - Single Flight (6:59 Max)";
    default: return "Low Launch (custom)";
  }
}
