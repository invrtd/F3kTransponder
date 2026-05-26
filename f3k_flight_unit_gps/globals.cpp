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
  "GROUND", "LAUNCH_WIN", "FLIGHT", "LANDED", "CALIBRATING"
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

bool wifi_shutdown_pending = false;
unsigned long wifi_shutdown_after_ms = 0;

unsigned long log_epoch_ms = 0;
unsigned long flight_start_epoch_ms = 0;
bool log_open = false;
volatile int littlefs_streaming = 0;
char log_path[32];
uint16_t flight_counter = 0;

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

// ── Pilot AP countdown state ─────────────────────────────────
bool          window_countdown_active = false;
unsigned long window_countdown_start = 0;
uint16_t      window_countdown_secs = 0;

// ── Pilot completed-log download state ───────────────────────
String        pilot_download_path = "";

// ── Flight record history ────────────────────────────────────
const int MAX_FLIGHT_RECORDS = 10;
FlightRecord flight_records[MAX_FLIGHT_RECORDS];
int flight_record_count = 0;