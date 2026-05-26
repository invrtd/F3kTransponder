// ============================================================
//  F3K Flight Unit — Main Firmware
//  Version:  1.3  |  May 2026
//
//  Board:    Adafruit QT Py ESP32-S3 (#5426, No PSRAM)
//  Sensors:  DPS310 altimeter (#4494) + LSM6DSO32 IMU (#4692)
//            via STEMMA QT I2C (Wire1, SDA=GPIO41, SCL=GPIO40)
//  GPS:      Adafruit PA1010D (#4415) — I2C 0x10, also Wire1 STEMMA QT
//            daisy-chained after DPS310 + LSM6DSO32
//  Battery:  150 mAh LiPo (#1317) — ~2 hrs at 80 MHz
//
//  Partition scheme: Default (3MB APP / 1.5MB SPIFFS)
//
//  Network:
//    STA mode  — connects to F3KBase, static IP 192.168.8.unit_id
//                scorer at 192.168.8.101, gateway 192.168.8.1
//                WiFi off during window (restarts after to retrieve log)
//                UDP 1 Hz on ground, 5 Hz during LAUNCH_WIN/FLIGHT/LANDED
//    AP mode   — fallback hotspot at 192.168.unit_id.1
//                WiFi off during window to save power (~100-150 mA)
//
//  State machine:
//    CALIBRATING → GROUND → LAUNCH_WIN → FLIGHT → LANDED
//
//  Detection thresholds:
//    LAUNCH_G_THRESHOLD  3.5 G     IMU G-spike (primary launch trigger)
//    LAUNCH_ALT_FT       8 ft      Barometric rise (backup launch trigger)
//    NEAR_GROUND         15 ft     Altitude ceiling for launch + tilt landing
//    MIN_FLIGHT_MS       10000 ms  Minimum flight before landing fires
//    LAND_IMPACT_G       4.0 G     Impact landing G-spike threshold
//    LAUNCH_WIN_MS       5000 ms   Confirmation window before FLIGHT
//
//  AP mode practice feature:
//    AUTO_WINDOW_SECS    595       Auto-open 9:55 window on first confirmed
//                                  throw when no window is active. Backdated
//                                  5s so LAUNCH_WIN counts as flight time.
//                                  Set to 0 to disable.
//
//  Scoring formulas (selectable via /setscore?m=0|1):
//    Mode 0 — Secs-Ft:  score = duration_s - throw_height_ft
//                        window score = sum of all flights
//    Mode 1 — JoeD V1:  score = (t/180)^0.425*1000 ± height component
//                        h<=100ft: bonus = (100-h)^1.6 * 0.113
//                        h>100ft:  penalty = (h-100)^2.3 * 0.09
//                        cap 1100 / floor 0
//                        window score = average of all flights
//
//  Log files (LittleFS /logs/):
//    window_NNN.csv   — sensor log: 8 Hz for windows ≤600s (~51 KB/10min)
//                       4 Hz for windows >600s (~86 KB/15min)
//    summary_NNN.csv  — score summary, auto-generated at window close
//
//  HTTP endpoints (port 80):
//    /pilot           — pilot data collection UI (AP mode)
//    /wstatus         — window status page (AP mode, during active window only)
//    /pstatus         — JSON telemetry for pilot page polling
//    /pgps            — JSON GPS state for GPS tab polling
//    /pstart?secs=N   — start window (5s countdown, then WiFi off)
//    /pstop           — stop window or cancel countdown
//    /log?n=N&del=1   — serve sensor log CSV (del=1 deletes, omit to keep)
//    /summary?n=N     — serve score summary CSV (kept on device)
//    /logs            — log browser with download and delete
//    /delete?f=name   — delete window_NNN.csv or summary_NNN.csv
//    /wipe-logs?confirm=YES&extra=SURE — delete ALL window_*.csv + summary_*.csv (UI escape hatch)
//    /setscore?m=0|1  — select scoring formula at runtime
//    /settilt?v=0|1   — toggle tilt mode (triggers recalibration)
//    /debug           — full telemetry overlay (telemetry tab iframe)
//
//  UDP (port 5005 → scorer):
//    Packet 1 — scoring data       5 Hz  port 5005
//    Packet 2 — GPS fix            1 Hz  port 4211  (when fix valid)
//    Packet 4 — debug/health       5 Hz  port 4213
//    Packet 5 — log announcement   on window close  port 4214
//    Listener — window command     port 5006 (scorer → unit)
//               0x20 = window start (immediate open)
//               0x21 = prep countdown (auto-fires when timer expires)
//
//  Libraries required (Tools → Manage Libraries):
//    Adafruit DPS310
//    Adafruit LSM6DS
//    Adafruit BusIO
//    Adafruit Unified Sensor
//    Adafruit GPS Library               ← add for PA1010D
//    Adafruit NeoPixel                  ← for boot status LED
//    ESP Async WebServer	By ESP32Async  ← use this fork
//    Async TCP By ESP32Async            ← use this fork
//
//  ICD reference: F3K_ICD_v1_7.docx
//
//  Source file CRC32: 67A96578  (computed over all lines except this one)
//  To verify: python3 -c "import binascii; d=open('f3k_flight_unit_gps.ino','rb').read(); lines=[l for l in d.split(b'\n') if b'Source file CRC32' not in l]; print(f'{binascii.crc32(b\"\\n\".join(lines))&0xFFFFFFFF:08X}')"
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LittleFS.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include "esp_wifi.h"     // for esp_wifi_set_ps() modem sleep
#include "secrets.h"      // passwords don't belong here 
#include "conf.h"      
#include "html.h"      // embedded HTML for pilot UI and window status pages
#include "types.h"     // data structures for sensor state and flight state machine
#include "globals.h"
//#include "webserver.h" // AsyncWebServer setup and endpoint handlers

// ── Log timestamp helper ──────────────────────────────────────
// Prints a timestamp prefix before every Serial log line.
// Format: [HH:MM:SS.mmm] from GPS UTC when fix valid,
//         [+NNNNNNNms]   from millis() always (shown alongside GPS).
// Usage: replace Serial.printf(...) with logts(); Serial.printf(...)
void logts() {
  unsigned long ms = millis();
  if (gps_fix) {
    // GPS milliseconds aren't always populated — use seconds only
    Serial.printf("[%02u:%02u:%02u +%07lums] ",
                  gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds, ms);
  } else {
    Serial.printf("[+%07lums] ", ms);
  }
}
const uint32_t CALIBRATION_MS  = 5000;   // duration of baseline averaging
const int      CAL_BUF_SIZE    = 50;     // max samples (8 Hz × 5s ≈ 40)
float          cal_buf[CAL_BUF_SIZE];
int            cal_count        = 0;
unsigned long  cal_start_ms     = 0;
bool           calibration_done = false;

// ── Batch JSON (double-buffered for async safety) ────────────
String batch_json_a = "{\"ready\":false}";
String batch_json_b = "{\"ready\":false}";
volatile int active_buf = 0;

// ── Runtime mode flags ───────────────────────────────────────
// tilt_mode can be toggled at runtime via /settilt endpoint
// regardless of the compile-time DEBUG_TILT_MODE setting.
// Both mode 1 (tilt angle) and mode 2 (parabola sim) set tilt_mode=true
// so they share the same calibration-skip and state-machine path.
// sim_mode tracks the specific mode: 0=normal, 1=tilt angle, 2=parabola sim.
#if DEBUG_TILT_MODE
bool    tilt_mode = true;              // start in sim mode if compiled with DEBUG_TILT_MODE 1 or 2
uint8_t sim_mode  = DEBUG_TILT_MODE;  // matches the compile-time selection
#else
bool    tilt_mode = false;
uint8_t sim_mode  = 0;
#endif

// ── Scoring formula selection ─────────────────────────────────
// SCORE_MODE 0 = Secs-Ft: duration_s - throw_height_ft  (window-independent)
// SCORE_MODE 1 = JoeD V1 formula:
//   time_score = (duration_s / window_secs)^0.425 * 1000
//   h <= 100ft: score = time_score + (100 - h)^1.6 * 0.113   (bonus, cap 1100)
//   h >  100ft: score = time_score - (h - 100)^2.3 * 0.09    (penalty, floor 0)
uint8_t score_mode = 0;   // default: Secs-Ft — change via /setscore?m=0|1
// ── WiFi mode ────────────────────────────────────────────────
bool ap_mode     = false;   // true when running as AP hotspot (no scorer network)
bool wifi_active = true;    // false while WiFi is off during an active window (AP mode only)
FlightState prev_flight_state = STATE_CALIBRATING;  // tracks state transitions for AP management

// ── Window countdown ─────────────────────────────────────────
const uint32_t WINDOW_COUNTDOWN_MS = 5000;  // 5s delay before window opens

// ── Auto-window (AP mode practice feature) ───────────────────
// When ap_mode is true and a flight is confirmed (LAUNCH_WIN→FLIGHT)
// with no window open, a window is auto-opened backdated by LAUNCH_WIN_MS
// so the 5-second launch confirmation period counts as flight time.
// Duration is set to AUTO_WINDOW_SECS (9:55 = 595s by default).
// Set to 0 to disable.
const uint16_t AUTO_WINDOW_SECS = 595;   // 9:55 — change or set 0 to disable
bool          window_countdown_active = false;
unsigned long window_countdown_start  = 0;
uint16_t      window_countdown_secs   = 0;   // pending window duration

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
const int MAX_FLIGHT_RECORDS = 20;
FlightRecord flight_records[MAX_FLIGHT_RECORDS];
int          flight_record_count = 0;
bool         pilot_window_active = false;   // window started from pilot UI
uint16_t     pilot_window_secs   = 0;
unsigned long pilot_window_start_ms = 0;
String        pilot_download_path = "";     // path of completed sensor log, cleared after download
#define LOG_DIR          "/logs"
#define LITTLEFS_FULL_PCT 60         // auto-delete threshold — prune early to avoid pressure

bool          window_active    = false;
uint16_t      window_secs      = 0;   // duration from scorer command
uint32_t      window_id        = 0;   // unique ID from scorer command
unsigned long window_start_ms  = 0;
unsigned long window_close_ms  = 0;   // captured atomically at window-end decision point
bool          prune_pending    = false;
bool          log_preopen_done = false; // log file pre-opened during prep, ready at window start

// ── Contest context (ICD v1.7) ────────────────────────────────
// Received from scorer in 0x20 bytes [8–10] and 0x21 bytes [10–12].
// Written into CSV comment headers for human-readable context.
uint8_t  contest_task_id   = 0;  // F3XVault flight_type_id; 0 = unknown/LL
uint8_t  contest_round_num = 0;  // 1-based; 0 = unknown
uint8_t  contest_group_num = 0;  // 1-based; 0 = unknown

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

// ── Hardware timers — window timing ──────────────────────────
// Two independent ESP32 hardware timers replace millis()-based polling
// for the two most timing-critical moments: window open and window close.
// The ISR fires at the exact microsecond regardless of main loop blocking.
//
// Timer 2 — Window Open:  arms when prep_fire_ms is set, fires to latch
//            window_start_ms and raise window_open_pending flag.
// Timer 3 — Window Close: arms when window opens, fires to latch
//            window_close_ms and raise window_close_pending flag.
//
// Both timers use prescaler 80 → 1 µs tick resolution.
// ISRs run in IRAM and touch only volatile flags and the latch variables.

static hw_timer_t* _timer_open  = nullptr;
static hw_timer_t* _timer_close = nullptr;

volatile bool          window_open_pending  = false;
volatile bool          window_close_pending = false;
volatile unsigned long window_open_latch_ms  = 0;  // millis() at ISR fire
volatile unsigned long window_close_latch_ms = 0;  // millis() at ISR fire

void IRAM_ATTR onWindowOpenTimer() {
  // Do NOT call millis() here — unsafe in ISR on ESP32 Arduino v3.x.
  // Main loop handler captures millis() when it sees the flag.
  window_open_pending = true;
}

void IRAM_ATTR onWindowCloseTimer() {
  window_close_pending = true;
}

// v3.x API: timerBegin(freq_hz) — 1000 Hz gives 1 ms tick resolution.
// timerAlarm(timer, ticks, autoreload, reload_count): ticks = delay_ms.
void armWindowOpenTimer(uint32_t delay_ms) {
  if (!_timer_open) return;
  timerStop(_timer_open);
  timerRestart(_timer_open);  // reset counter to 0
  timerAlarm(_timer_open, (uint64_t)delay_ms, false, 0);
  timerStart(_timer_open);
}

void armWindowCloseTimer(uint32_t delay_ms) {
  if (!_timer_close) return;
  timerStop(_timer_close);
  timerRestart(_timer_close);  // reset counter to 0
  timerAlarm(_timer_close, (uint64_t)delay_ms, false, 0);
  timerStart(_timer_close);
}

void disarmWindowOpenTimer()  { if (_timer_open)  timerStop(_timer_open);  }
void disarmWindowCloseTimer() { if (_timer_close) timerStop(_timer_close); }
// Scorer sends 0x21 during prep time so units can auto-open the
// window even if out of WiFi range when 0x20 would normally fire.
bool          prep_active      = false;   // countdown running
unsigned long prep_fire_ms     = 0;       // millis() at which to open window
uint16_t      prep_window_secs = 0;       // window duration to use when it fires
uint32_t      prep_window_id   = 0;       // window_id expected with the 0x20

// ── Deferred WiFi shutdown ────────────────────────────────────
// WiFi is shut down after window open, but not synchronously inside
// openWindowLog() — that is called from the UDP receive path and
// server.end() + WiFi.mode(OFF) need the async TCP task on Core 0
// to fully drain first. A flag defers the actual shutdown to the
// main loop where a proper yield/delay can be used safely.
bool          wifi_shutdown_pending = false;
unsigned long wifi_shutdown_after_ms = 0;  // fire shutdown at this millis()
unsigned long log_epoch_ms        = 0;   // t_ms=0 reference — set at window open
unsigned long flight_start_epoch_ms = 0; // millis() at each GROUND→LAUNCH_WIN
File          log_file;
bool          log_open           = false;
volatile int  littlefs_streaming = 0;  // count of active HTTP file streams; LittleFS.end() deferred while > 0
char          log_path[32];           // e.g. /logs/window_042.csv
uint16_t      flight_counter   = 0;   // increments each LAUNCH_WIN→FLIGHT

// Announce timer — sends Packet 5 a few times after window closes
bool          announce_pending   = false;
uint8_t       announce_count     = 0;
unsigned long last_announce_ms   = 0;
uint32_t      announce_file_size = 0;   // cached at window close — avoids LittleFS in sendAnnouncement
const uint8_t ANNOUNCE_REPEATS   = 5;   // send announcement N times
const uint32_t ANNOUNCE_INTERVAL_MS = 2000;

// ── Diagnostics ──────────────────────────────────────────────
unsigned long loop_count    = 0;
unsigned long busy_us_total = 0;
unsigned long loop_us_total = 0;
unsigned long loop_max_us   = 0;

struct DiagSnap {
  int      rssi_dbm;
  float    cpu_load_pct;
  float    loop_avg_us;
  float    loop_max_us;
  uint32_t free_heap;
  int      sample_count;
} diag = {0, 0, 0, 0, 0, 0};

// ── Timing ───────────────────────────────────────────────────
unsigned long last_sensor_ms  = 0;
unsigned long last_imu_ms     = 0;
unsigned long last_gps_ms     = 0;   // GPS NMEA drain cadence
unsigned long last_display_ms = 0;
unsigned long last_udp_ms     = 0;
unsigned long last_dbg_ms     = 0;
unsigned long last_diag_ms    = 0;   // 1 Hz diagnostic snapshot — always runs

// Live temperature from DPS310 (updated each sensor read)
float live_temperature_c = 0.0f;

// ── Globals ──────────────────────────────────────────────────
Adafruit_DPS310    dps;
Adafruit_LSM6DSO32 lsm;
// QT Py ESP32-S3 onboard NeoPixel: GPIO39 (data), GPIO38 (power enable)
Adafruit_NeoPixel  pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
AsyncWebServer     server(80);
WiFiUDP            udp;
WiFiUDP            udp_win;   // listens on port 5006 for window commands
WiFiUDP            udp_gps;   // Packet 2 — GPS fix data (port 4211)

// ============================================================
//  LittleFS — load full config
// ============================================================
void loadConfig() {
  if (!LittleFS.begin(true)) {
    logts(); Serial.println("LittleFS: mount failed — using defaults.");
    return;
  }
  logts(); Serial.println("LittleFS: mounted OK");

  if (!LittleFS.exists(CONFIG_PATH)) {
    logts(); Serial.println("LittleFS: config not found — using defaults.");
    LittleFS.end(); return;
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) { Serial.println("LittleFS: open failed."); LittleFS.end(); return; }
  String s = f.readString();
  f.close();
  LittleFS.end();

  // Simple key:value parser — no external JSON library needed
  auto parseInt = [&](const char* key, int fallback) -> int {
    int idx = s.indexOf(key);
    if (idx < 0) return fallback;
    int colon = s.indexOf(':', idx);
    if (colon < 0) return fallback;
    int v = s.substring(colon + 1).toInt();
    return v;
  };

  int id  = parseInt("\"unit_id\"",      DEFAULT_UNIT_ID);
  int web = parseInt("\"web_enabled\"",   1);
  int cpu = parseInt("\"cpu_mhz\"",      240);
  int wn  = parseInt("\"window_number\"", 0);

  cfg.unit_id       = (id >= 1 && id <= 200) ? (uint8_t)id : DEFAULT_UNIT_ID;
  cfg.web_enabled   = (web == 1);
  cfg.cpu_mhz       = (cpu == 80 || cpu == 160 || cpu == 240) ? cpu : 240;
  cfg.window_number = (wn >= 0 && wn <= 9999) ? (uint16_t)wn : 0;

  logts(); Serial.printf("Config loaded: unit_id=%d  web=%s  cpu=%dMHz  window=%d\n",
                cfg.unit_id,
                cfg.web_enabled ? "ON" : "OFF",
                cfg.cpu_mhz,
                cfg.window_number);
}

// ============================================================
//  Helpers
// ============================================================
float pressureToAltitudeFeet(float p) {
  return 44330.0f * (1.0f - powf(p / SEA_LEVEL_HPA, 0.1903f)) * M_TO_FT;
}

// ============================================================
//  IMU read — called at 26 Hz
// ============================================================
void readImu() {
  if (!imu_present) return;

  sensors_event_t accel_event, gyro_event, temp_event;
  lsm.getEvent(&accel_event, &gyro_event, &temp_event);

  imu.accel_x = accel_event.acceleration.x;
  imu.accel_y = accel_event.acceleration.y;
  imu.accel_z = accel_event.acceleration.z;
  imu.gyro_x  = gyro_event.gyro.x;
  imu.gyro_y  = gyro_event.gyro.y;
  imu.gyro_z  = gyro_event.gyro.z;

  // G-force magnitude (total acceleration vector in G)
  float mag = sqrtf(imu.accel_x * imu.accel_x +
                    imu.accel_y * imu.accel_y +
                    imu.accel_z * imu.accel_z);
  imu.g_force = mag / 9.80665f;

  // Tilt angle from vertical — angle between accel vector and gravity vector
  // When flat/level, accel_z ≈ -9.8 m/s², tilt ≈ 0°
  imu.tilt_deg = acosf(fabsf(imu.accel_z) / max(mag, 0.001f)) * 180.0f / PI;

  imu.valid = true;
}

// ============================================================
//  UDP packet — 14 bytes, big-endian
//  Byte 0:    unit_id
//  Byte 1:    state
//  Bytes 2-3: altitude × 10  (uint16)
//  Bytes 4-7: timestamp_ms   (uint32)
//  Bytes 8-9: duration × 10  (uint16)
//  Byte 10:   peak_alt_ft    (uint8)
//  Bytes 11-12: launch_ht×10 (uint16)
//  Byte 13:   battery_pct    (uint8)
// ============================================================
void sendUdpPacket() {
  uint8_t pkt[14] = {0};

  uint16_t alt10      = (uint16_t)constrain(flight.altitude_ft    * 10.0f, 0, 65535);
  uint16_t dur10      = (uint16_t)constrain(flight.flight_duration_ms / 100, 0, 65535);
  uint8_t  peak       = (uint8_t) constrain(flight.peak_alt_ft,   0, 255);
  uint16_t launch10   = (uint16_t)constrain(flight.launch_height_ft * 10.0f, 0, 65535);
  uint32_t ts         = millis();

  pkt[0]  = flight.unit_id;
  pkt[1]  = (uint8_t)flight.state;

  // Big-endian multi-byte fields
  pkt[2]  = (alt10    >> 8) & 0xFF;
  pkt[3]  =  alt10          & 0xFF;

  pkt[4]  = (ts       >> 24) & 0xFF;
  pkt[5]  = (ts       >> 16) & 0xFF;
  pkt[6]  = (ts       >>  8) & 0xFF;
  pkt[7]  =  ts              & 0xFF;

  pkt[8]  = (dur10    >> 8) & 0xFF;
  pkt[9]  =  dur10          & 0xFF;

  pkt[10] = peak;

  pkt[11] = (launch10 >> 8) & 0xFF;
  pkt[12] =  launch10       & 0xFF;

  pkt[13] = flight.battery_pct;

  udp.beginPacket(SERVER_IP, UDP_PORT);
  udp.write(pkt, 14);
  udp.endPacket();
}

// ============================================================
//  UDP Packet 4 — Debug/Health (Port 4213)
//  14 bytes, big-endian, 5 Hz
//
//  Byte 0:    unit_id
//  Byte 1:    rssi_dbm      (int8,   signed)
//  Byte 2:    cpu_load_pct  (uint8,  0-100)
//  Bytes 3-4: free_heap_kb  (uint16, ÷10 → kB)
//  Bytes 5-6: loop_avg_us   (uint16, µs)
//  Bytes 7-8: loop_max_us   (uint16, µs)
//  Bytes 9-10: temperature  (int16,  ÷100 → °C)
//  Byte 11:   state         (uint8,  mirrors Packet 1)
//  Bytes 12-13: spare       (0x00)
// ============================================================
void sendDebugPacket() {
  uint8_t pkt[14] = {0};

  // Clamp diagnostics to fit field widths
  int8_t   rssi    = (int8_t)constrain(diag.rssi_dbm, -128, 127);
  uint8_t  cpu     = (uint8_t)constrain((int)diag.cpu_load_pct, 0, 100);
  uint16_t heap10  = (uint16_t)constrain((diag.free_heap / 100), 0, 65535);
  uint16_t lavg    = (uint16_t)constrain((int)diag.loop_avg_us, 0, 65535);
  uint16_t lmax    = (uint16_t)constrain((int)diag.loop_max_us, 0, 65535);
  int16_t  temp100 = (int16_t)constrain((int)(live_temperature_c * 100.0f), -32768, 32767);

  pkt[0]  = flight.unit_id;
  pkt[1]  = (uint8_t)rssi;           // cast preserves sign bits
  pkt[2]  = cpu;
  pkt[3]  = (heap10  >> 8) & 0xFF;
  pkt[4]  =  heap10        & 0xFF;
  pkt[5]  = (lavg    >> 8) & 0xFF;
  pkt[6]  =  lavg          & 0xFF;
  pkt[7]  = (lmax    >> 8) & 0xFF;
  pkt[8]  =  lmax          & 0xFF;
  pkt[9]  = (temp100 >> 8) & 0xFF;
  pkt[10] =  temp100       & 0xFF;
  pkt[11] = (uint8_t)flight.state;
  pkt[12] = 0x00;                    // spare
  pkt[13] = 0x00;                    // spare

  udp.beginPacket(SERVER_IP, UDP_DBG_PORT);
  udp.write(pkt, 14);
  udp.endPacket();
}

// ============================================================
//  UDP Packet 2 — GPS Fix (Port 4211)
//  16 bytes, big-endian, transmitted only when fix_quality > 0
//  Per ICD v1.5 Section 3
//
//  Byte 0:    unit_id        (uint8)
//  Byte 1:    fix_quality    (uint8)   0=none 1=GPS 2=DGPS 6=est
//  Byte 2:    satellites     (uint8)   0–32
//  Byte 3:    hdop_x10       (uint8)   HDOP × 10, capped 255
//  Bytes 4–7: latitude_e5   (int32)   decimal degrees × 100000
//  Bytes 8–11:longitude_e5  (int32)   decimal degrees × 100000
//  Bytes 12–13:altitude_m_x10 (int16) MSL decimetres, signed
//  Bytes 14–15:spare         (0x00)
// ============================================================
void sendGpsPacket() {
  if (!gps_fix) return;   // ICD: only transmit when fix_quality > 0

  uint8_t pkt[16] = {0};

  int32_t lat_e5 = (int32_t)(gps_lat * 100000.0f);
  int32_t lon_e5 = (int32_t)(gps_lon * 100000.0f);
  int16_t alt_dm = (int16_t)constrain(gps_alt_m * 10.0f, -32768.0f, 32767.0f);
  uint8_t hdop10 = (uint8_t)constrain(gps_hdop  * 10.0f, 0.0f, 255.0f);

  pkt[0]  = cfg.unit_id;
  pkt[1]  = gps_fix_quality;
  pkt[2]  = gps_sats;
  pkt[3]  = hdop10;

  pkt[4]  = (lat_e5 >> 24) & 0xFF;
  pkt[5]  = (lat_e5 >> 16) & 0xFF;
  pkt[6]  = (lat_e5 >>  8) & 0xFF;
  pkt[7]  =  lat_e5        & 0xFF;

  pkt[8]  = (lon_e5 >> 24) & 0xFF;
  pkt[9]  = (lon_e5 >> 16) & 0xFF;
  pkt[10] = (lon_e5 >>  8) & 0xFF;
  pkt[11] =  lon_e5        & 0xFF;

  pkt[12] = (alt_dm >> 8) & 0xFF;
  pkt[13] =  alt_dm       & 0xFF;
  pkt[14] = 0x00;
  pkt[15] = 0x00;

  udp_gps.beginPacket(SERVER_IP, 4211);
  udp_gps.write(pkt, 16);
  udp_gps.endPacket();
}

// ============================================================
//  saveConfig — persists window_number back to LittleFS
// ============================================================
void saveConfig() {
  if (!LittleFS.begin(false)) return;
  File f = LittleFS.open(CONFIG_PATH, "w");
  if (!f) {
    if (littlefs_streaming <= 0 && !log_open) LittleFS.end();
    return;
  }
  f.printf("{\n  \"unit_id\": %d,\n  \"web_enabled\": %d,\n"
           "  \"cpu_mhz\": %d,\n  \"window_number\": %d\n}\n",
           cfg.unit_id, cfg.web_enabled ? 1 : 0,
           cfg.cpu_mhz, cfg.window_number);
  f.close();
  if (littlefs_streaming <= 0 && !log_open) LittleFS.end();
  logts(); Serial.printf("[CFG] Saved window_number=%d\n", cfg.window_number);
}

// ============================================================
//  pruneLogsIfNeeded — called from main loop during idle ground state
//  Deletes AT MOST ONE FILE per call, then sets prune_pending=true if
//  more work remains. This yields control back to the main loop between
//  deletions so UDP packets (prep countdown) can be processed.
//  Blocking for an entire multi-file prune cycle was causing the unit to
//  miss early prep countdown packets, creating window timing errors.
//
//  Priority order:
//  1. Summary cap: delete oldest summary (+ matching window log) if >16
//  2. Space threshold: delete oldest window log if usage > LITTLEFS_FULL_PCT
// ============================================================
#define MAX_SUMMARIES  16   // keep at most 16 summary files (≈ 2-day contest)

void pruneLogsIfNeeded() {
  if (!LittleFS.begin(false)) return;

  // ── Check summary count ────────────────────────────────────────
  {
    uint16_t nums[64];
    uint8_t  count = 0;
    File dir = LittleFS.open(LOG_DIR);
    if (dir) {
      File entry = dir.openNextFile();
      while (entry && count < 64) {
        String name = String(entry.name());
        if (name.startsWith("summary_") && name.endsWith(".csv"))
          nums[count++] = (uint16_t)name.substring(8, 11).toInt();
        entry = dir.openNextFile();
      }
    }

    if (count > MAX_SUMMARIES) {
      // Find the single lowest-numbered summary to delete this pass
      uint16_t lowest = 65535;
      for (int i = 0; i < count; i++)
        if (nums[i] < lowest) lowest = nums[i];

      char path[48];
      snprintf(path, sizeof(path), "%s/summary_%03d.csv", LOG_DIR, lowest);
      if (LittleFS.exists(path)) {
        LittleFS.remove(path);
        logts(); Serial.printf("[FS] Pruned old summary: %s (cap=%d)\n", path, MAX_SUMMARIES);
      }
      snprintf(path, sizeof(path), "%s/window_%03d.csv", LOG_DIR, lowest);
      if (LittleFS.exists(path)) {
        LittleFS.remove(path);
        logts(); Serial.printf("[FS] Pruned matching window log: %s\n", path);
      }
      // More summaries may still exceed cap — reschedule for next loop pass
      LittleFS.end();
      prune_pending = true;
      return;
    }
  }

  // ── Check space threshold ──────────────────────────────────────
  {
    size_t total    = LittleFS.totalBytes();
    size_t used     = LittleFS.usedBytes();
    uint8_t used_pct = (total > 0) ? (uint8_t)(100ULL * used / total) : 0;
    logts(); Serial.printf("[FS] %d%% used (%u / %u bytes)\n", used_pct, used, total);

    if (used_pct >= LITTLEFS_FULL_PCT) {
      // Find the single oldest window log to delete this pass
      File dir = LittleFS.open(LOG_DIR);
      if (dir) {
        String   oldest = "";
        uint16_t lowest = 65535;
        File entry = dir.openNextFile();
        while (entry) {
          String name = String(entry.name());
          if (name.startsWith("window_") && name.endsWith(".csv")) {
            uint16_t num = (uint16_t)name.substring(7, 10).toInt();
            if (num < lowest) { lowest = num; oldest = name; }
          }
          entry = dir.openNextFile();
        }
        if (oldest.length() > 0) {
          String full = String(LOG_DIR) + "/" + oldest;
          LittleFS.remove(full);
          logts(); Serial.printf("[FS] Deleted %s (space prune)\n", full.c_str());
          // May still be over threshold — reschedule for next loop pass
          LittleFS.end();
          prune_pending = true;
          return;
        }
      }
    }
  }

  LittleFS.end();
  // prune_pending stays false — all done
}

// ============================================================
//  calculateScore — unified scoring with selectable formula
//
//  Mode 0 — Secs-Ft:
//    duration_s - throw_height_ft  (window-independent, no normalization)
//    Window score = sum of all flight scores
//
//  Mode 1 — JoeD V1:
//    time_score = (duration_s / 180)^0.425 * 1000   ← t_max ALWAYS 180s
//    h <= 100ft: score = time_score + (100-h)^1.6 * 0.113  (cap 1100)
//    h >  100ft: score = time_score - (h-100)^2.3 * 0.09   (floor 0)
//    Window score = AVERAGE of all flight scores (including airborne-at-close)
// ============================================================
float calculateScore(float dur_s, float throw_height_ft) {
  if (score_mode == 1) {
    // JoeD V1 — t_max is always 3 minutes (180s), independent of window length
    const float JOED_TMAX = 180.0f;
    float time_ratio = constrain(dur_s / JOED_TMAX, 0.0f, 1.0f);
    float score_t    = powf(time_ratio, 0.425f) * 1000.0f;
    float h          = throw_height_ft;
    float final_score;
    if (h <= 100.0f) {
      float bonus = powf(100.0f - h, 1.6f) * 0.113f;
      final_score = score_t + bonus;
    } else {
      float penalty = powf(h - 100.0f, 2.3f) * 0.09f;
      final_score = score_t - penalty;
    }
    return constrain(final_score, 0.0f, 1100.0f);
  }

  // Mode 0 — Secs-Ft: duration_s - throw_height_ft (window-independent)
  if (window_secs == 0) return 0.0f;
  return dur_s - throw_height_ft;
}

// ============================================================
//  openWindowLog — called when scorer sends window start
// ============================================================
void openWindowLog() {
  if (littlefs_streaming > 0) {
    logts(); Serial.printf("[LOG] openWindowLog deferred — %d stream(s) still active\n",
                  littlefs_streaming);
  }

  cfg.window_number++;
  // saveConfig() deferred — written after window is stable to avoid blocking here
  // It will be called from the main loop once wifi_active is confirmed.
  // For now just update the in-memory value so log_path is correct.
  snprintf(log_path, sizeof(log_path), "%s/window_%03d.csv", LOG_DIR, cfg.window_number);

  if (log_preopen_done && log_open) {
    // ── Fast path: log was pre-opened during prep ──────────────
    // File is already open and headers written. Just reset counters
    // and anchor the epoch. No filesystem I/O needed here.
    log_epoch_ms        = window_start_ms;
    flight_counter      = 0;
    flight_record_count = 0;
    log_preopen_done    = false;
    logts(); Serial.printf("[LOG] Opened %s (pre-opened, fast path)  id=%u  %ds\n",
                  log_path, window_id, window_secs);
    if (gps_fix) {
      logts(); Serial.printf("[TOD] Window open: %02u:%02u:%02u UTC (GPS)\n",
                    gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds);
    }
    disarmWindowOpenTimer();
    armWindowCloseTimer((uint32_t)window_secs * 1000UL);
    logts(); Serial.printf("[HW] Close timer armed: %ds\n", window_secs);
    if (!ap_mode) {
      wifi_shutdown_pending  = true;
      wifi_shutdown_after_ms = millis() + 200;
    }
    // Defer saveConfig() to after WiFi is stable
    return;
  }

  // ── Slow path: open log file now (no pre-open available) ─────
  if (!LittleFS.begin(false)) {
    logts(); Serial.println("[LOG] LittleFS mount failed — logging disabled.");
    window_active = false;  // undo caller's set — openWindowLog failed
    return;
  }

  size_t free_bytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  float  log_rate   = (window_secs > 600) ? 1200.0f : 2400.0f;
  size_t needed     = max((size_t)204800, (size_t)(window_secs * log_rate));
  if (free_bytes < needed) {
    logts(); Serial.printf("[LOG] !!! Insufficient space: %u free, need ~%u — logging disabled !!!\n",
                  free_bytes, needed);
    LittleFS.end();
    window_active = false;  // undo caller's set — openWindowLog failed
    return;
  }

  if (!LittleFS.exists(LOG_DIR)) LittleFS.mkdir(LOG_DIR);

  log_file = LittleFS.open(log_path, "w");
  if (!log_file) {
    logts(); Serial.printf("[LOG] Failed to open %s\n", log_path);
    LittleFS.end();
    window_active = false;  // undo caller's set — openWindowLog failed
    return;
  }

  // Write context and column headers
  {
    char ctx[160];
    if (contest_round_num > 0) {
      snprintf(ctx, sizeof(ctx), "# Round %u, Group %u | Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
               contest_round_num, contest_group_num,
               taskName(contest_task_id), contest_task_id,
               window_secs, flight.unit_id);
    } else {
      snprintf(ctx, sizeof(ctx), "# Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
               taskName(contest_task_id), contest_task_id,
               window_secs, flight.unit_id);
    }
    log_file.print(ctx);
  }
  log_file.print("t_ms,flight,flight_t_s,state,throw_height_ft,alt_ft,alt_tared_ft,pressure_hpa,temp_c,"
                 "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,g_total,tilt_deg,"
                 "gps_lat,gps_lon,gps_alt_m,gps_sats\n");

  log_open            = true;
  log_preopen_done    = false;
  log_epoch_ms        = window_start_ms;
  flight_counter      = 0;
  flight_record_count = 0;
  logts(); Serial.printf("[LOG] Opened %s  id=%u  %ds window  free=%u bytes\n",
                log_path, window_id, window_secs,
                (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()));
  if (gps_fix) {
    logts(); Serial.printf("[TOD] Window open: %02u:%02u:%02u UTC (GPS)\n",
                  gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds);
  }
  disarmWindowOpenTimer();
  armWindowCloseTimer((uint32_t)window_secs * 1000UL);
  logts(); Serial.printf("[HW] Close timer armed: %ds\n", window_secs);

  if (!ap_mode) {
    wifi_shutdown_pending  = true;
    wifi_shutdown_after_ms = millis() + 200;
  }
}

// ============================================================
//  logSample — writes one CSV row (called at 8 Hz when window active)
// ============================================================
void logSample(float alt_ft, float pressure_hpa, float temp_c) {
  if (!log_open) return;

  // t_ms relative to window open (log_epoch_ms)
  unsigned long t_rel = millis() - log_epoch_ms;

  // Periodic flush every 8 seconds (64 samples) to ensure data reaches flash.
  // Without this, a full filesystem causes silent write failures that leave
  // the file at 0 bytes — the buffer is never committed.
  static uint16_t flush_counter = 0;
  if (++flush_counter >= 64) {
    flush_counter = 0;
    log_file.flush();
    // Check if filesystem has run out of space by comparing file size to
    // what we expect — if size stopped growing, disable logging gracefully.
    if (t_rel > 10000 && log_file.size() < 1000) {
      logts(); Serial.println("[LOG] !!! Write failure detected — filesystem likely full. Disabling logging.");
      log_file.close();
      LittleFS.end();
      log_open = false;
      return;
    }
  }

  // Flight elapsed time in seconds (000.0 format)
  // Only counts when in LAUNCH_WIN or FLIGHT — 0.0 otherwise
  float flight_t_s = 0.0f;
  if (flight.state == STATE_LAUNCH_WIN || flight.state == STATE_FLIGHT) {
    flight_t_s = flight.flight_duration_ms / 1000.0f;
  }

  char row[290];   // enlarged from 220 to fit GPS columns
  snprintf(row, sizeof(row),
           "%lu,%d,%07.1f,%d,%.3f,%.3f,%.3f,%.4f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.2f",
           t_rel,
           flight_counter,
           flight_t_s,
           (int)flight.state,
           flight.throw_height_ft,
           alt_ft,
           alt_ft - tare_baseline_ft,
           pressure_hpa,
           temp_c,
           imu.valid ? imu.accel_x / 9.80665f : 0.0f,
           imu.valid ? imu.accel_y / 9.80665f : 0.0f,
           imu.valid ? imu.accel_z / 9.80665f : 0.0f,
           imu.valid ? imu.gyro_x * 180.0f / PI : 0.0f,
           imu.valid ? imu.gyro_y * 180.0f / PI : 0.0f,
           imu.valid ? imu.gyro_z * 180.0f / PI : 0.0f,
           imu.valid ? imu.g_force : 0.0f,
           imu.valid ? imu.tilt_deg : 0.0f);
  log_file.print(row);

  // GPS columns — only written when a valid fix exists on this sample.
  // Empty fields (,,,) when no fix or no module: correct CSV null semantics
  // and avoids ~96 KB of 0,0,0,0 zeros per 10-min window with no GPS.
  // lat=0/lon=0 would be a real coordinate (Gulf of Guinea) so we never
  // write numeric zeros here.
  if (gps_fix) {
    char gps_suffix[64];
    snprintf(gps_suffix, sizeof(gps_suffix),
             ",%.6f,%.6f,%.1f,%u",
             gps_lat, gps_lon, gps_alt_m, gps_sats);
    log_file.print(gps_suffix);
  } else {
    log_file.print(",,,");   // empty lat, lon, alt_m, sats — 4 columns, 3 commas
  }
  log_file.print('\n');
}

// ============================================================
//  writeSummaryLog — writes score summary CSV after window ends
//  Columns: Flight#, Start(ms), End(ms), Duration(s),
//            LaunchHeight(ft), JoeD_Score, SecsMinusFt_Score
//  Last row: totals
// ============================================================
void writeSummaryLog() {
  if (!LittleFS.begin(false)) return;

  // Path mirrors sensor log: summary_NNN.csv
  char sum_path[36];
  snprintf(sum_path, sizeof(sum_path), "%s/summary_%03d.csv", LOG_DIR, cfg.window_number);

  File f = LittleFS.open(sum_path, "w");
  if (!f) {
    logts(); Serial.printf("[SUMMARY] Failed to open %s\n", sum_path);
    LittleFS.end();
    return;
  }

  // Context comment header — matches window CSV format for easy cross-reference
  {
    char ctx[160];
    if (contest_round_num > 0) {
      snprintf(ctx, sizeof(ctx), "# Round %u, Group %u | Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
               contest_round_num, contest_group_num,
               taskName(contest_task_id), contest_task_id,
               window_secs, flight.unit_id);
    } else {
      snprintf(ctx, sizeof(ctx), "# Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
               taskName(contest_task_id), contest_task_id,
               window_secs, flight.unit_id);
    }
    f.print(ctx);
  }

  // Column header
  f.print("flight_num,start_ms,end_ms,duration_s,launch_height_ft,joed_score,secsft_score\n");

  float total_dur     = 0;
  float total_height  = 0;
  float total_joed    = 0;
  float total_secsft  = 0;

  for (int i = 0; i < flight_record_count; i++) {
    FlightRecord& r = flight_records[i];

    // Always compute both scores regardless of current mode
    float joed_score = 0;
    {
      const float JOED_TMAX = 180.0f;
      float tr = constrain(r.duration_s / JOED_TMAX, 0.0f, 1.0f);
      float st = powf(tr, 0.425f) * 1000.0f;
      float h  = r.throw_height_ft;
      float fs;
      if (h <= 100.0f) fs = st + powf(100.0f - h, 1.6f) * 0.113f;
      else             fs = st - powf(h - 100.0f, 2.3f) * 0.09f;
      joed_score = constrain(fs, 0.0f, 1100.0f);
    }
    float secsft_score = r.duration_s - r.throw_height_ft;

    char row[120];
    snprintf(row, sizeof(row), "%u,%lu,%lu,%.1f,%.1f,%.1f,%.1f\n",
             r.flight_num,
             r.start_time_ms,
             r.end_time_ms,
             r.duration_s,
             r.throw_height_ft,
             joed_score,
             secsft_score);
    f.print(row);

    total_dur    += r.duration_s;
    total_height += r.throw_height_ft;
    total_joed   += joed_score;
    total_secsft += secsft_score;
  }

  // Totals row — "T" in flight_num column
  // joed total = average (window score); secsft total = sum
  // Guard against divide-by-zero when no flights were scored
  float avg_height = flight_record_count > 0 ? total_height / flight_record_count : 0.0f;
  float avg_joed   = flight_record_count > 0 ? total_joed   / flight_record_count : 0.0f;
  char totals[120];
  snprintf(totals, sizeof(totals), "T,,,%.1f,%.1f,%.1f,%.1f\n",
           total_dur,
           avg_height,
           avg_joed,
           total_secsft);
  f.print(totals);

  f.flush();
  uint32_t sz = f.size();
  f.close();
  LittleFS.end();

  logts(); Serial.printf("[SUMMARY] Written %s  %u bytes  %d flights\n",
                sum_path, sz, flight_record_count);
}

// ============================================================
//  closeWindowLog — flush and close, trigger announcement
// ============================================================
void closeWindowLog() {
  // Always clear window state — even if no log file was open. The Timeout
  // fallback in loop() re-calls this every iteration while window_active
  // remains true; an earlier early-return-on-!log_open bypassed these
  // three assignments and produced infinite [WIN] Timeout fallback spam,
  // and blocked all subsequent 0x21 prep packets (0x21 is ignored while
  // window_active is true). The "already set false at top" comment below
  // (line ~1181) documents the original intent.
  window_active        = false;
  window_close_pending = false;  // clear ISR flag — we're handling it now
  disarmWindowCloseTimer();      // stop the window timeout check immediately

  if (!log_open) return;

  // ── Capture close timestamp FIRST — before any blocking I/O ──
  // This is the authoritative window-end time used for scoring.
  // Everything after this point is housekeeping.
  window_close_ms = millis();

  // Record in-progress flight at exact close time
  if ((flight.state == STATE_FLIGHT || flight.state == STATE_LAUNCH_WIN) &&
      flight_record_count < MAX_FLIGHT_RECORDS) {
    float dur = flight.flight_duration_ms / 1000.0f;
    float score = calculateScore(dur, flight.throw_height_ft);
    flight_records[flight_record_count++] = {
      (uint16_t)flight_counter,
      dur,
      flight.throw_height_ft,
      flight.peak_alt_ft,
      score,
      (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
      window_close_ms - log_epoch_ms
    };
    logts(); Serial.printf("[SCORE] Window close — flight #%d still airborne: dur=%.1fs score=%.1f\n",
                  flight_counter, dur, score);
  }

  log_file.flush();
  uint32_t file_size = log_file.size();
  announce_file_size = file_size;   // cache for sendAnnouncement() — avoids LittleFS reopens
  log_file.close();
  LittleFS.end();
  log_open = false;
  // window_active already set false at top of closeWindowLog()

  // Cancel any pending deferred WiFi shutdown — it was scheduled for an
  // in-flight AP-off transition. Now that the window is closing we are
  // about to restart the AP ourselves, and we must not let the deferred
  // executor fire afterwards and silently kill it again.
  if (wifi_shutdown_pending) {
    wifi_shutdown_pending = false;
    logts(); Serial.println("[PWR] Cancelled pending WiFi shutdown at window close");
  }

  // Make sensor log available for pilot download
  pilot_download_path = String(log_path);

  logts(); Serial.printf("[LOG] Closed %s  %u bytes\n", log_path, file_size);
  if (gps_fix) {
    logts(); Serial.printf("[TOD] Window close: %02u:%02u:%02u UTC (GPS)\n",
                  gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds);
  }

  if (file_size == 0) {
    logts(); Serial.println("!!! WARNING: Log file is empty — possible LittleFS error !!!");
  }

  // Write score summary CSV
  writeSummaryLog();

  // Schedule deferred filesystem cleanup. Running pruneLogsIfNeeded() here
  // would block closeWindowLog() for hundreds of milliseconds (directory
  // scans + multiple file deletes) while WiFi is restarting — risks WDT.
  // The main loop services this flag only when in idle GROUND state.
  prune_pending = true;

  // Restart WiFi after window close so scorer can retrieve the log.
  // AP mode: restart the hotspot.
  // STA mode: reconnect to F3KBase so announcements and log retrieval work.
  if (ap_mode) {
    char ap_ssid[24];
    snprintf(ap_ssid, sizeof(ap_ssid), "F3K-Unit-%02d", cfg.unit_id);
    IPAddress ap_ip(192, 168, cfg.unit_id, 1);
    IPAddress ap_gateway(192, 168, cfg.unit_id, 1);
    IPAddress ap_subnet(255, 255, 255, 0);
    logts(); Serial.println("[PWR] closeWindowLog: calling WiFi.mode(WIFI_AP)...");
    WiFi.mode(WIFI_AP);
    logts(); Serial.println("[PWR] closeWindowLog: calling softAPConfig...");
    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    logts(); Serial.printf("[PWR] closeWindowLog: calling softAP(%s)...\n", ap_ssid);
    bool ap_ok = WiFi.softAP(ap_ssid);
    logts(); Serial.printf("[PWR] closeWindowLog: softAP returned %s\n", ap_ok ? "OK" : "FAILED");
    delay(500);
    logts(); Serial.printf("[PWR] closeWindowLog: AP IP = %s  clients = %d\n",
                  WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum());
    logts(); Serial.println("[PWR] closeWindowLog: calling server.begin()...");
    server.begin();
    wifi_active = true;
    logts(); Serial.printf("[PWR] WiFi ON — AP ready: %s  http://%s/pilot\n",
                  ap_ssid, WiFi.softAPIP().toString().c_str());
  } else {
    // STA mode — reconnect to scorer network with static IP
    WiFi.mode(WIFI_STA);
    IPAddress sta_ip(192, 168, 8, cfg.unit_id);
    IPAddress sta_gateway(192, 168, 8, 1);
    IPAddress sta_subnet(255, 255, 255, 0);
    IPAddress sta_dns(192, 168, 8, 1);
    WiFi.config(sta_ip, sta_gateway, sta_subnet, sta_dns);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // Don't block here — announcements will fire once WiFi.status()==WL_CONNECTED
    // The loop's announce handler checks wifi_active before sending
    wifi_active = false;  // will be set true when connection is confirmed in loop
    logts(); Serial.printf("[PWR] WiFi reconnecting to %s as 192.168.8.%d...\n",
                  WIFI_SSID, cfg.unit_id);
  }

  // Trigger announcement packets.
  // In STA mode: if WiFi is currently active AND not mid-shutdown, send all
  // ANNOUNCE_REPEATS announcements immediately (100ms apart) before this
  // window's state is overwritten by a possible back-to-back openWindowLog()
  // call (e.g. when 0x20 arrives immediately after 0x21 fires).
  // If WiFi is down or a shutdown is already pending (radio about to go off),
  // fall back to the deferred loop path which fires once WiFi reconnects.
  if (!ap_mode && wifi_active && !wifi_shutdown_pending) {
    logts(); Serial.printf("[LOG] Sending %d immediate announcements for window_%03d\n",
                  ANNOUNCE_REPEATS, cfg.window_number);
    for (uint8_t i = 0; i < ANNOUNCE_REPEATS; i++) {
      announce_count = i;
      sendAnnouncement();
      if (i < ANNOUNCE_REPEATS - 1) delay(100);
    }
    announce_pending = false;
    announce_count   = 0;
  } else {
    // WiFi not yet up or shutdown pending — defer to loop as before
    logts(); Serial.printf("[LOG] Deferring announcements for window_%03d (wifi_active=%s pending=%s)\n",
                  cfg.window_number,
                  wifi_active           ? "Y" : "N",
                  wifi_shutdown_pending ? "Y" : "N");
    announce_pending  = true;
    announce_count    = 0;
    last_announce_ms  = 0;  // fire immediately on next loop pass
  }
}

// ============================================================
//  UDP Packet 5 — Log Announcement (Port 4214)
//  14 bytes, big-endian
//
//  Byte 0:     unit_id
//  Byte 1:     packet_type  0x10
//  Bytes 2-3:  window_number  (uint16)
//  Bytes 4-7:  window_id      (uint32)
//  Bytes 8-11: file_size      (uint32, bytes)
//  Bytes 12-13: spare
// ============================================================
void sendAnnouncement() {
  // Use the file size cached at window close — avoids concurrent LittleFS
  // access while AsyncWebServer may be serving other files, which can panic.
  uint32_t file_size = announce_file_size;

  uint8_t pkt[14] = {0};
  pkt[0]  = flight.unit_id;
  pkt[1]  = 0x10;  // log available
  pkt[2]  = (cfg.window_number >> 8) & 0xFF;
  pkt[3]  =  cfg.window_number       & 0xFF;
  pkt[4]  = (window_id >> 24) & 0xFF;
  pkt[5]  = (window_id >> 16) & 0xFF;
  pkt[6]  = (window_id >>  8) & 0xFF;
  pkt[7]  =  window_id        & 0xFF;
  pkt[8]  = (file_size >> 24) & 0xFF;
  pkt[9]  = (file_size >> 16) & 0xFF;
  pkt[10] = (file_size >>  8) & 0xFF;
  pkt[11] =  file_size        & 0xFF;
  pkt[12] = 0x00;
  pkt[13] = 0x00;

  udp.beginPacket(SERVER_IP, UDP_ANN_PORT);
  udp.write(pkt, 14);
  udp.endPacket();
  logts(); Serial.printf("[LOG] Announce window_%03d  %u bytes  (%d/%d)\n",
                cfg.window_number, file_size, announce_count + 1, ANNOUNCE_REPEATS);
}

// ============================================================
//  checkWindowCommand — poll udp_win for scorer broadcasts (ICD v1.7)
//
//  Packet 0x20 — Window Start (14 bytes, big-endian):
//    Byte 0:     0x20
//    Byte 1:     0xFF (broadcast marker)
//    Bytes 2-3:  window_secs  uint16
//    Bytes 4-7:  window_id    uint32
//    Byte 8:     task_id      uint8   (F3XVault flight_type_id; 0=unknown)  NEW v1.7
//    Byte 9:     round_num    uint8   (1-based; 0=unknown)                  NEW v1.7
//    Byte 10:    group_num    uint8   (1-based; 0=unknown)                  NEW v1.7
//    Bytes 11-13: spare
//
//  Packet 0x21 — Prep Countdown (14 bytes, big-endian):
//    Byte 0:     0x21
//    Byte 1:     0xFF (broadcast marker)
//    Bytes 2-3:  countdown_secs  uint16  — seconds until window opens
//    Bytes 4-5:  window_secs     uint16  — window duration when it fires
//    Bytes 6-9:  window_id       uint32  — will match the 0x20 that follows
//    Byte 10:    task_id         uint8   NEW v1.7
//    Byte 11:    round_num       uint8   NEW v1.7
//    Byte 12:    group_num       uint8   NEW v1.7
//    Byte 13:    spare
//  Packet 0x22 — TOD Sync (6 bytes, big-endian):
//    Byte 0:     0x22
//    Byte 1:     0xFF (broadcast marker)
//    Bytes 2-5:  scorer_ms_since_midnight  (uint32, UTC milliseconds since 00:00:00)
//    Sent every 5s during prep and window. Unit compares vs GPS UTC and prints delta.
//    Warnings printed at >2s delta, ERROR at >20s delta.
// ============================================================
void checkWindowCommand(unsigned long now) {
  int pkt_size = udp_win.parsePacket();
  if (pkt_size < 14) return;

  uint8_t buf[14] = {0};
  udp_win.read(buf, 14);

  uint8_t ptype = buf[0];

  // Drain buffer before any early return
  while (udp_win.parsePacket() > 0) udp_win.flush();

  // ── 0x22 — TOD sync ──────────────────────────────────────────
  if (ptype == 0x22) {
    uint32_t scorer_ms = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16)
                       | ((uint32_t)buf[4] <<  8) |  buf[5];

    if (gps_fix) {
      // Compute GPS ms since midnight UTC
      uint32_t gps_ms = ((uint32_t)gps_sensor.hour   * 3600000UL)
                      + ((uint32_t)gps_sensor.minute  *   60000UL)
                      + ((uint32_t)gps_sensor.seconds *    1000UL)
                      + ((uint32_t)gps_sensor.milliseconds);
      int32_t  delta_ms = (int32_t)scorer_ms - (int32_t)gps_ms;

      // Format both as HH:MM:SS.mmm
      auto fmtTOD = [](uint32_t ms) -> String {
        uint32_t h   = ms / 3600000;  ms %= 3600000;
        uint32_t m   = ms /   60000;  ms %=   60000;
        uint32_t s   = ms /    1000;  ms %=    1000;
        char buf[16];
        snprintf(buf, sizeof(buf), "%02u:%02u:%02u.%03u", h, m, s, ms);
        return String(buf);
      };

      const char* status = "OK";
      if      (abs(delta_ms) > 20000) status = "ERROR — BROKEN SYNC";
      else if (abs(delta_ms) >  2000) status = "WARNING — drift > 2s";

      logts(); Serial.printf("[TOD] Scorer: %s  GPS: %s  delta=%+dms  %s\n",
                    fmtTOD(scorer_ms).c_str(), fmtTOD(gps_ms).c_str(),
                    delta_ms, status);
    } else {
      // No GPS fix — print scorer time only
      uint32_t ms = scorer_ms;
      uint32_t h  = ms / 3600000; ms %= 3600000;
      uint32_t m  = ms /   60000; ms %=   60000;
      uint32_t s  = ms /    1000; ms %=    1000;
      logts(); Serial.printf("[TOD] Scorer: %02u:%02u:%02u.%03u  (no GPS fix)\n", h, m, s, ms);
    }
    return;
  }

  // ── 0x21 — Prep countdown ────────────────────────────────────
  if (ptype == 0x21) {
    // Rule 6: ignore if window already active
    if (window_active) {
      logts(); Serial.println("[WIN] 0x21 ignored — window already active");
      return;
    }

    uint16_t countdown_secs = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t win_secs       = ((uint16_t)buf[4] << 8) | buf[5];
    uint32_t wid            = ((uint32_t)buf[6] << 24) | ((uint32_t)buf[7] << 16)
                            | ((uint32_t)buf[8] <<  8) |  buf[9];
    // v1.7: task context bytes [10–12]
    contest_task_id   = buf[10];
    contest_round_num = buf[11];
    contest_group_num = buf[12];

    logts(); Serial.printf("[WIN] 0x21 prep: countdown=%ds  win=%ds  id=%u  task=%u  R%u G%u\n",
                  countdown_secs, win_secs, wid, contest_task_id, contest_round_num, contest_group_num);

    if (prep_active && prep_window_id != wid) {
      // Rule 3: different window_id — discard old countdown, start fresh
      logts(); Serial.printf("[WIN] 0x21 new id=%u (was %u) — resetting countdown\n", wid, prep_window_id);
      prep_active = false;
      // Discard any pre-opened log for the old window
      if (log_preopen_done && log_open && !window_active) {
        log_file.close();
        LittleFS.end();
        log_open         = false;
        log_preopen_done = false;
        logts(); Serial.println("[LOG] Pre-opened log discarded (prep reset)");
      }
    }

    if (!prep_active) {
      // Rule 1: no countdown running — start from countdown_secs
      prep_fire_ms     = now + (uint32_t)countdown_secs * 1000UL;
      prep_window_secs = win_secs;
      prep_window_id   = wid;
      prep_active      = true;
      armWindowOpenTimer((uint32_t)countdown_secs * 1000UL);
      logts(); Serial.printf("[WIN] 0x21 countdown started: fires in %ds  win=%ds  id=%u\n",
                    countdown_secs, win_secs, wid);
    } else {
      // Rule 2: countdown already running for same window_id — adjust remaining time
      unsigned long new_fire = now + (uint32_t)countdown_secs * 1000UL;
      long drift = (long)new_fire - (long)prep_fire_ms;
      prep_fire_ms     = new_fire;
      prep_window_secs = win_secs;
      armWindowOpenTimer((uint32_t)countdown_secs * 1000UL);  // re-arm with corrected value
      logts(); Serial.printf("[WIN] 0x21 countdown adjusted: drift=%ldms  fires in %ds\n",
                    drift, countdown_secs);
    }
    return;
  }

  // ── 0x20 — Window start ──────────────────────────────────────
  if (ptype != 0x20) return;  // unknown type

  uint16_t secs = ((uint16_t)buf[2] << 8) | buf[3];
  uint32_t wid  = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16)
                | ((uint32_t)buf[6] <<  8) |  buf[7];
  // v1.7: task context bytes [8–10]
  contest_task_id   = buf[8];
  contest_round_num = buf[9];
  contest_group_num = buf[10];

  logts(); Serial.printf("[WIN] 0x20 start: secs=%d  id=%u  task=%u  R%u G%u\n",
                secs, wid, contest_task_id, contest_round_num, contest_group_num);

  // Rule 5: 0x20 while prep countdown running → cancel countdown, open immediately
  if (prep_active) {
    logts(); Serial.println("[WIN] 0x20 cancels prep countdown — opening immediately");
    prep_active = false;
  }

  if (window_active) {
    // Grace period: if the current window just opened within the last
    // WINDOW_OPEN_GRACE_MS, treat this 0x20 as the authoritative confirmation
    // rather than a new window. This absorbs the race where the prep countdown
    // fires ~500ms before the scorer's 0x20 arrives, which would otherwise
    // produce a tiny orphan log followed by a duplicate window open.
    // In this case: update window_id and window_secs in place, no close/reopen.
    const uint32_t WINDOW_OPEN_GRACE_MS = 3000;
    if ((now - window_start_ms) < WINDOW_OPEN_GRACE_MS) {
      logts(); Serial.printf("[WIN] 0x20 within grace period (%lums after open) — adopting id=%u secs=%d  task=%u R%u G%u\n",
                    now - window_start_ms, wid, secs, contest_task_id, contest_round_num, contest_group_num);
      window_id   = wid;
      window_secs = secs;
      // contest_task_id / round / group already set above from buf[8–10]
      // Don't adjust window_start_ms — keep the time the window actually opened
      return;
    }
    logts(); Serial.println("[WIN] 0x20 while window active — closing current");
    closeWindowLog();
  }

  window_secs     = secs;
  window_id       = wid;
  window_start_ms = now;
  window_active   = true;
  logts(); Serial.printf("[WIN] Window open  id=%u  duration=%ds\n", wid, secs);
  openWindowLog();
}
// ============================================================
void updateStateMachine(float alt_ft) {
  // ── Thresholds — tuned from 3-session real flight data ───────
  // Data: gyro median during FLIGHT=34dps, G p5-p95=0.73-1.48G
  // Ground handling: G reaches 2.0-2.5G — IMU launch threshold raised above this
  // Soaring glider appears "at rest" on IMU — rest window duration raised to 3s
  // and gyro threshold raised above the flight median of 34dps
  const float    LAUNCH_ALT_FT      =  8.0f;  // ft above ground → altitude launch trigger
  const float    LAUNCH_G_THRESHOLD =  3.5f;  // G — IMU launch (raised: ground handling reaches 2-2.5G)
  const float    LANDED_ALT_FT      =  5.0f;  // ft — landing altitude threshold
  const float    LAND_IMPACT_G      =  4.0f;  // G — hard impact (raised: avoid turbulence false triggers)
  const float    LAND_REST_G_MIN    =  0.8f;  // G — rest lower bound
  const float    LAND_REST_G_MAX    =  1.2f;  // G — rest upper bound
  const float    LAND_GYRO_DPS      = 45.0f;  // °/s — raised: flight median gyro is 34dps
  const uint32_t LAUNCH_WIN_MS      = 5000;   // ms — confirmation window duration
  const uint32_t LAND_REST_MS       = 3000;   // ms — raised: 3s steady rest is definitive
  const uint32_t MIN_FLIGHT_MS      = 10000;  // ms — minimum time in FLIGHT before alt landing fires

  static uint32_t launch_win_opened_ms  = 0;
  static uint32_t flight_started_ms     = 0;
  static uint32_t land_rest_start_ms    = 0;   // when rest condition first met
  static uint32_t impact_pending_ms     = 0;   // when impact G spike was detected
  static float    impact_pending_alt    = 0;   // altitude at impact spike
  static bool     was_high_this_flight  = false; // hit >10ft during this flight (catch guard)
  static bool     tilt_was_low          = false;  // tilt<12° confirmed during flight (tilt land guard)
  static uint8_t  tilt_low_consec       = 0;   // consecutive samples with tilt<12°
  static uint8_t  tilt_high_consec      = 0;   // consecutive samples with tilt>20°

  // ── Precompute IMU values used across cases ───────────────
  bool imu_ok        = imu_present && imu.valid;
  float gyro_max_dps = 0;
  if (imu_ok) {
    gyro_max_dps = max(fabsf(imu.gyro_x * 180.0f / PI),
                   max(fabsf(imu.gyro_y * 180.0f / PI),
                       fabsf(imu.gyro_z * 180.0f / PI)));
  }
  bool imu_launch_spike = imu_ok && (imu.g_force >= LAUNCH_G_THRESHOLD);
  bool imu_impact       = imu_ok && (imu.g_force >= LAND_IMPACT_G);
  bool imu_at_rest      = imu_ok &&
                          (imu.g_force >= LAND_REST_G_MIN) &&
                          (imu.g_force <= LAND_REST_G_MAX) &&
                          (gyro_max_dps < LAND_GYRO_DPS);

  switch (flight.state) {

    case STATE_CALIBRATING:
      if (calibration_done) {
        flight.state = STATE_GROUND;
        logts(); Serial.printf("[STATE] CALIBRATING → GROUND  baseline=%.2fft\n",
                      tare_baseline_ft);
      }
      break;

    case STATE_GROUND:
      flight.flight_duration_ms = 0;
      flight.throw_height_ft    = 0.0f;  // clear for next flight
      land_rest_start_ms        = 0;
      was_high_this_flight      = false;
      tilt_was_low              = false;
      tilt_low_consec           = 0;
      tilt_high_consec          = 0;
      {
        bool alt_trigger    = (alt_ft - tare_baseline_ft >= LAUNCH_ALT_FT);
        bool imu_trigger    = imu_launch_spike;
        // Altitude ceiling: a glider on the ground cannot be above 15ft.
        // If a false LANDED fires mid-air, this prevents immediately re-launching
        // at altitude when the state resets to GROUND.
        bool near_ground    = (alt_ft - tare_baseline_ft < 15.0f);
        if ((alt_trigger || imu_trigger) && near_ground) {
          flight.state            = STATE_LAUNCH_WIN;
          flight.launch_height_ft = alt_ft;
          flight.throw_height_ft  = alt_ft - last_landed_alt_ft;
          launch_win_opened_ms    = millis();
          flight_started_ms       = millis();
          flight_start_epoch_ms   = millis();   // window-relative start for summary CSV
          flight.peak_alt_ft      = alt_ft;
          logts(); Serial.printf("[STATE] GROUND → LAUNCH_WIN  alt=%.1fft  G=%.2f  trigger=%s\n",
                        alt_ft, imu_ok ? imu.g_force : 0.0f,
                        (alt_trigger && imu_trigger) ? "ALT+IMU" :
                        alt_trigger ? "ALT" : "IMU");
        }
      }
      break;

    case STATE_LAUNCH_WIN:
      flight.flight_duration_ms = millis() - flight_started_ms;
      if (alt_ft > flight.peak_alt_ft)      flight.peak_alt_ft      = alt_ft;
      if (alt_ft > flight.launch_height_ft) flight.launch_height_ft = alt_ft;
      // Update throw height every sample — peak gain above last landing
      flight.throw_height_ft = max(flight.throw_height_ft,
                                   alt_ft - last_landed_alt_ft);

      if (millis() - launch_win_opened_ms >= LAUNCH_WIN_MS) {
        flight.state = STATE_FLIGHT;
        // throw_height_ft is now frozen for the rest of this flight
        flight_counter++;
        logts(); Serial.printf("[STATE] LAUNCH_WIN → FLIGHT  lh=%.1fft  throw=%.1fft  t=%.1fs  flight#%d\n",
                      flight.launch_height_ft, flight.throw_height_ft,
                      flight.flight_duration_ms / 1000.0f, flight_counter);
        if (gps_fix) {
          logts(); Serial.printf("[TOD] Flight #%d launch: %02u:%02u:%02u UTC (GPS)\n",
                        flight_counter, gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds);
        }

        // ── Auto-window (AP mode practice feature) ────────────
        // If no window is open and AUTO_WINDOW_SECS > 0, open one now.
        // Backdate window_start_ms by LAUNCH_WIN_MS so t_ms=0 in the CSV
        // aligns with the throw and the 5-second confirmation period is
        // credited as flight time. The window then expires naturally.
        if (ap_mode && !window_active && AUTO_WINDOW_SECS > 0) {
          window_secs     = AUTO_WINDOW_SECS;
          window_id       = millis();                    // unique local ID
          window_start_ms = millis() - LAUNCH_WIN_MS;   // backdate 5s
          window_active   = true;
          flight_record_count = 0;
          pilot_download_path = "";
          logts(); Serial.printf("[WIN] Auto-window opened: %ds  backdated 5s  id=%u\n",
                        window_secs, window_id);
          openWindowLog();
          // openWindowLog() resets flight_counter to 0, but flight #1 is
          // already confirmed and in progress — restore it so the landing
          // record shows flight_num=1, not flight_num=0.
          flight_counter = 1;
        }
      }
      // False trigger: altitude dropped AND no significant motion
      if ((alt_ft - tare_baseline_ft < LANDED_ALT_FT) && !imu_launch_spike) {
        flight.state              = STATE_GROUND;
        flight.flight_duration_ms = 0;
        flight.throw_height_ft    = 0.0f;
        logts(); Serial.println("[STATE] LAUNCH_WIN → GROUND (false trigger)");
      }
      break;

    case STATE_FLIGHT:
      flight.flight_duration_ms = millis() - flight_started_ms;
      if (alt_ft > flight.peak_alt_ft) flight.peak_alt_ft = alt_ft;

      // ── Track flight state flags for landing guards ────────
      // Catch guard: must have climbed above 10ft at some point
      if (alt_ft - tare_baseline_ft > 10.0f) was_high_this_flight = true;

      // Tilt guard: must have seen sustained low tilt (in-flight bias) before
      // tilt landing can fire. Prevents false trigger when unit never flew.
      if (imu_ok) {
        if (imu.tilt_deg < 12.0f) {
          tilt_low_consec = min((int)tilt_low_consec + 1, 255);
          if (tilt_low_consec >= 5) tilt_was_low = true;
        } else {
          tilt_low_consec = 0;
        }
        // Count consecutive high-tilt samples for tilt landing detector
        if (imu.tilt_deg > 20.0f) {
          tilt_high_consec = min((int)tilt_high_consec + 1, 255);
        } else {
          tilt_high_consec = 0;
        }
      }

      // ── Landing detection — whichever fires first ─────────

      // 0. Catch/grab — primary detector
      //    Guards: (a) must have climbed above 10ft — rules out barometric
      //    drift keeping alt negative throughout a low/failed flight
      //    (b) minimum flight time elapsed
      if (was_high_this_flight &&
          (alt_ft - tare_baseline_ft < -2.0f) &&
          (imu.g_force > 1.5f) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS)) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        // Re-zero baseline only if landing detected near ground level —
        // guards against re-zeroing when tilt fires at altitude or
        // large accumulated drift pushes reading well below baseline.
        // Suppressed in sim mode — sim controls altitude explicitly and
        // tare updates would corrupt the arc math.
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at catch: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (catch alt=%.1fft G=%.2f)  dur=%.1fs  peak=%.1fft\n",
                      alt_ft - tare_baseline_ft, imu.g_force,
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }

      // 1. Tilt landing detector
      //    In coordinated flight tilt stays 3-8°. On landing tilt rises to >20°
      //    as pilot holds glider at arbitrary angle. Require 1s sustained (8 samples).
      //    Guards: tilt must have been low during flight, min flight time elapsed,
      //    AND altitude must be near ground — aerobatics at altitude can also
      //    produce sustained high tilt (e.g. inverted, knife-edge maneuvers).
      if (tilt_was_low &&
          imu_ok &&
          (tilt_high_consec >= 8) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS) &&
          (alt_ft - tare_baseline_ft < 15.0f)) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at tilt: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (tilt %.1f° x%d samples)  dur=%.1fs  peak=%.1fft\n",
                      imu.tilt_deg, tilt_high_consec,
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }

      // 1. Hard impact spike near ground
      if (imu_impact && (alt_ft - tare_baseline_ft < 15.0f) &&
          flight.flight_duration_ms >= MIN_FLIGHT_MS) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at impact: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (impact G=%.2f)  dur=%.1fs  peak=%.1fft\n",
                      imu.g_force, flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }
      // 2. Altitude + gyro — alt below threshold AND low rotation (not just a low pass)
      // Pure altitude alone cannot distinguish low flying from landing.
      // Requiring gyro < 10 dps ensures the glider is actually stationary, not skimming.
      if ((alt_ft - tare_baseline_ft < LANDED_ALT_FT) &&
          (gyro_max_dps < 10.0f) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS)) {
        impact_pending_ms = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
          logts(); Serial.printf("[SCORE] Flight #%d recorded: dur=%.1fs throw=%.1fft score=%.1f (record %d/%d)\n",
                        flight_counter, flight.flight_duration_ms/1000.0f,
                        flight.throw_height_ft, score,
                        flight_record_count, MAX_FLIGHT_RECORDS);
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (alt)  dur=%.1fs  peak=%.1fft\n",
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        if (gps_fix) {
          logts(); Serial.printf("[TOD] Landing: %02u:%02u:%02u UTC (GPS)\n",
                        gps_sensor.hour, gps_sensor.minute, gps_sensor.seconds);
        }
        break;
      }
      // 3. Sustained rest (gentle landing / catch)
      // Disabled in tilt mode — stationary tilted unit falsely triggers
      // Also requires minimum flight time to avoid false triggers at launch
      if (!tilt_mode && flight.flight_duration_ms >= MIN_FLIGHT_MS) {
        if (imu_at_rest) {
          if (land_rest_start_ms == 0) land_rest_start_ms = millis();
          if (millis() - land_rest_start_ms >= LAND_REST_MS) {
          impact_pending_ms = 0;
            if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
            if (!window_active) {
              // Window already closed and scored this flight at timeout — don't record again
            } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
              float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
              flight_records[flight_record_count++] = {
                (uint16_t)flight_counter,
                flight.flight_duration_ms / 1000.0f,
                flight.throw_height_ft,
                flight.peak_alt_ft,
                score
              };
            }
            flight.state = STATE_LANDED;
            logts(); Serial.printf("[STATE] FLIGHT → LANDED (rest %.0fms)  dur=%.1fs  peak=%.1fft\n",
                          (float)(millis() - land_rest_start_ms),
                          flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
          }
        } else {
          land_rest_start_ms = 0;
        }
      }
      break;

    case STATE_LANDED:
      // Auto-reset to GROUND quickly to support Quick Turn (QT) maneuver.
      // FAI: glider is "landed" the instant pilot touches it, "launched" the
      // instant pilot releases it. A good QT takes 1-3 seconds, so we must
      // be back in GROUND well within that window.
      // LANDED_RESET_MS is intentionally short — the scorer handles scoring
      // of the completed flight; the unit just needs to be ready for the next launch.
      {
        static uint32_t landed_ms = 0;
        const uint32_t  LANDED_RESET_MS = 500;  // ms — back to GROUND, ready for QT
        if (landed_ms == 0) landed_ms = millis();
        if (millis() - landed_ms >= LANDED_RESET_MS) {
          landed_ms                 = 0;
          flight.state              = STATE_GROUND;
          flight.peak_alt_ft        = 0;
          flight.flight_duration_ms = 0;
          flight.throw_height_ft    = 0.0f;
          land_rest_start_ms        = 0;
          impact_pending_ms         = 0;
          impact_pending_alt        = 0;
          was_high_this_flight      = false;
          tilt_was_low              = false;
          tilt_low_consec           = 0;
          tilt_high_consec          = 0;
          logts(); Serial.println("[STATE] LANDED → GROUND");
        }
      }
      break;
  }
}

// ============================================================
//  snapDiagnostics — 1 Hz, always runs regardless of web state
//  Populates diag struct for Packet 4. Serial output is silent
//  after calibration — only events are printed.
// ============================================================
void snapDiagnostics() {
  diag.rssi_dbm     = WiFi.RSSI();
  diag.free_heap    = ESP.getFreeHeap();
  diag.loop_max_us  = loop_max_us;
  diag.sample_count = buf_count;

  if (loop_count > 0) {
    diag.loop_avg_us  = (float)loop_us_total / loop_count;
    diag.cpu_load_pct = loop_us_total > 0
                        ? 100.0f * (float)busy_us_total / (float)loop_us_total
                        : 0.0f;
  }
  loop_count = busy_us_total = loop_us_total = loop_max_us = 0;

  // When web is disabled, flushDisplayBatch() never runs so buf_count
  // never resets. Reset it here so the display buffer doesn't saturate.
  if (!cfg.web_enabled) buf_count = 0;

  // Silent after calibration — no periodic output, events only
}

// ============================================================
//  5 Hz web display batch flush (staggered 100ms from scorer packet)
// ============================================================
void flushDisplayBatch() {
  // diag struct already populated by snapDiagnostics() — no need to re-read here

  if (tare_requested && buf_count > 0) {
    float sum = 0;
    for (int i = 0; i < buf_count; i++) sum += pressureToAltitudeFeet(buf_pressure[i]);
    tare_baseline_ft = sum / buf_count;
    tare_requested   = false;
    logts(); Serial.printf("[TARE] Baseline set: %.3f ft\n", tare_baseline_ft);
  }

  if (buf_count == 0) { buf_count = 0; return; }

  String arr_alt_ft = "[", arr_alt_tare = "[", arr_pressure = "[", arr_temp = "[";
  // Reserve ~12 chars per sample per array to avoid repeated reallocation
  arr_alt_ft.reserve(buf_count * 12);
  arr_alt_tare.reserve(buf_count * 12);
  arr_pressure.reserve(buf_count * 12);
  arr_temp.reserve(buf_count * 10);
  float sum_alt = 0;

  for (int i = 0; i < buf_count; i++) {
    float alt_ft   = pressureToAltitudeFeet(buf_pressure[i]);
    float alt_tare = alt_ft - tare_baseline_ft;
    sum_alt += alt_ft;
    arr_alt_ft   += String(alt_ft,             3);
    arr_alt_tare += String(alt_tare,           3);
    arr_pressure += String(buf_pressure[i],    4);
    arr_temp     += String(buf_temperature[i], 3);
    if (i < buf_count - 1) {
      arr_alt_ft += ","; arr_alt_tare += ",";
      arr_pressure += ","; arr_temp += ",";
    }
  }
  arr_alt_ft += "]"; arr_alt_tare += "]";
  arr_pressure += "]"; arr_temp += "]";

  float mean_alt_ft = sum_alt / buf_count;

  String& wb = (active_buf == 0) ? batch_json_b : batch_json_a;
  wb.reserve(600);   // pre-size to avoid reallocation during concatenation
  wb  = "{";
  wb += "\"ready\":true,";
  wb += "\"unit_id\":"           + String(flight.unit_id)                      + ",";
  wb += "\"state\":"             + String((int)flight.state)                   + ",";
  wb += "\"state_name\":\""      + String(stateNames[flight.state])            + "\",";
  wb += "\"sample_count\":"      + String(buf_count)                           + ",";
  wb += "\"tare_baseline_ft\":"  + String(tare_baseline_ft, 3)                 + ",";
  wb += "\"mean_alt_ft\":"       + String(mean_alt_ft, 3)                      + ",";
  wb += "\"mean_alt_tared_ft\":" + String(mean_alt_ft - tare_baseline_ft, 3)   + ",";
  wb += "\"peak_alt_ft\":"       + String(flight.peak_alt_ft, 1)               + ",";
  wb += "\"launch_height_ft\":"  + String(flight.launch_height_ft, 1)          + ",";
  wb += "\"flight_duration_s\":" + String(flight.flight_duration_ms / 1000.0f, 1) + ",";
  wb += "\"alt_ft\":"            + arr_alt_ft                                  + ",";
  wb += "\"alt_tared\":"         + arr_alt_tare                                + ",";
  wb += "\"pressure\":"          + arr_pressure                                + ",";
  wb += "\"temp\":"              + arr_temp                                    + ",";
  wb += "\"rssi_dbm\":"          + String(diag.rssi_dbm)                       + ",";
  wb += "\"cpu_load_pct\":"      + String(diag.cpu_load_pct, 1)                + ",";
  wb += "\"loop_avg_us\":"       + String(diag.loop_avg_us, 0)                 + ",";
  wb += "\"loop_max_us\":"       + String(diag.loop_max_us, 0)                 + ",";
  wb += "\"free_heap_b\":"       + String(diag.free_heap)                      + ",";
  // IMU — LSM6DSO32
  wb += "\"imu_present\":"       + String(imu_present ? "true" : "false")      + ",";
  wb += "\"accel_x\":"           + String(imu.accel_x, 3)                      + ",";
  wb += "\"accel_y\":"           + String(imu.accel_y, 3)                      + ",";
  wb += "\"accel_z\":"           + String(imu.accel_z, 3)                      + ",";
  wb += "\"gyro_x\":"            + String(imu.gyro_x,  4)                      + ",";
  wb += "\"gyro_y\":"            + String(imu.gyro_y,  4)                      + ",";
  wb += "\"gyro_z\":"            + String(imu.gyro_z,  4)                      + ",";
  wb += "\"g_force\":"           + String(imu.g_force,  3)                     + ",";
  wb += "\"tilt_deg\":"          + String(imu.tilt_deg, 2);
  wb += "}";

  active_buf = (active_buf == 0) ? 1 : 0;
  buf_count  = 0;
}



// ============================================================
//  setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);
  delay(500);
  logts(); Serial.println("\n=== F3K Flight Unit ===");

  // Print reset reason to help diagnose unexpected reboots
  esp_reset_reason_t reason = esp_reset_reason();
  const char* reason_str[] = {"UNKNOWN","POWERON","EXT","SW","PANIC",
                               "INT_WDT","TASK_WDT","WDT","DEEPSLEEP","BROWNOUT","SDIO"};
  logts(); Serial.printf("[BOOT] Reset reason: %s\n",
                reason < 11 ? reason_str[reason] : "OTHER");
  logts(); Serial.printf("[BOOT] Build: %s %s\n", __DATE__, __TIME__);
#if DEBUG_TILT_MODE == 1
  logts(); Serial.println("*** DEBUG TILT MODE 1 — altitude simulated from tilt angle ***");
  logts(); Serial.println("  Flat(0-15°)=GROUND(0ft)  Tilted(15-45°)=LAUNCH(10ft)  Side(45-90°)=FLIGHT(tilt°=ft)");
#elif DEBUG_TILT_MODE == 2
  logts(); Serial.println("*** DEBUG TILT MODE 2 — autonomous parabola simulation ***");
  logts(); Serial.printf("  Flight: %ds duration  %dft peak  5s ground pause  repeats until window ends\n",
                cfg.unit_id * 10, cfg.unit_id * 10);
#endif

  // -- Load config from LittleFS --
  loadConfig();
  flight.unit_id = cfg.unit_id;

  // -- CPU frequency --
  setCpuFrequencyMhz(cfg.cpu_mhz);
  logts(); Serial.printf("CPU: %d MHz\n", getCpuFrequencyMhz());

  // -- I2C port selection (controlled by USE_STEMMA_QT define) --
#if USE_STEMMA_QT
  Wire1.begin();                // STEMMA QT JST bus (SDA=41, SCL=40)
  Wire1.setClock(400000);       // DPS310 + LSM6DSO32 + PA1010D all on Wire1
  logts(); Serial.println("I2C: STEMMA QT port  Wire1 (SDA=41, SCL=40)  400kHz");
#else
  Wire.begin();                 // Standard pads (SDA=8, SCL=9)
  Wire.setClock(400000);
  logts(); Serial.println("I2C: Standard port  Wire (SDA=8, SCL=9)  400kHz");
#endif
#if USE_STEMMA_QT
  if (!dps.begin_I2C(0x77, &Wire1)) {
#else
  if (!dps.begin_I2C(0x77, &Wire)) {
#endif
    // DPS310 not found on first try — retry up to 10 times with 500ms gap.
    // Occasional I2C glitch at power-on can cause a false miss.
    logts(); Serial.print("DPS310 not found — retrying");
    // Enable NeoPixel power and init for status indication
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
    pixel.begin();
    pixel.setBrightness(40);

    bool dps_ok = false;
    for (int attempt = 1; attempt <= 10 && !dps_ok; attempt++) {
      logts(); Serial.printf(" %d", attempt);
      delay(500);
      pixel.setPixelColor(0, pixel.Color(50, 20, 0));  // amber — retrying
      pixel.show();
#if USE_STEMMA_QT
      dps_ok = dps.begin_I2C(0x77, &Wire1);
#else
      dps_ok = dps.begin_I2C(0x77, &Wire);
#endif
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
      pixel.show();
    }
    logts(); Serial.println();

    if (!dps_ok) {
      // All retries exhausted — flash red 5 times then continue without DPS310.
      // Unit will boot, connect to WiFi, and send altitude=0 in all packets.
      // Scoring is disabled (no altitude data) but logging infrastructure works.
      logts(); Serial.println("ERROR: DPS310 not found after 10 attempts — continuing without altimeter.");
      logts(); Serial.println("  Check STEMMA QT cable. Unit will operate with altitude=0.");
      for (int i = 0; i < 5; i++) {
        pixel.setPixelColor(0, pixel.Color(80, 0, 0));  // red
        pixel.show(); delay(200);
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));
        pixel.show(); delay(200);
      }
      dps_present      = false;
      calibration_done = true;    // skip calibration — no sensor
      tare_baseline_ft = 0.0f;
      flight.state     = STATE_GROUND;
    } else {
      dps_present = true;
      logts(); Serial.println("DPS310 OK (on retry)");
      dps.configurePressure(DPS310_8HZ, DPS310_8SAMPLES);
      dps.configureTemperature(DPS310_8HZ, DPS310_8SAMPLES);
      // Brief green flash to confirm recovery
      pixel.setPixelColor(0, pixel.Color(0, 60, 0));
      pixel.show(); delay(300);
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
      pixel.show();
    }
  } else {
    dps_present = true;
    logts(); Serial.println("DPS310 OK");
    dps.configurePressure(DPS310_8HZ, DPS310_8SAMPLES);
    dps.configureTemperature(DPS310_8HZ, DPS310_8SAMPLES);
  }

  // -- LSM6DSO32 IMU --
#if USE_STEMMA_QT
  if (!lsm.begin_I2C(0x6A, &Wire1)) {
#else
  if (!lsm.begin_I2C(0x6A, &Wire)) {
#endif
    logts(); Serial.println("WARNING: LSM6DSO32 not found — IMU data unavailable.");
    logts(); Serial.println("  Check STEMMA QT chain. Unit will continue without IMU.");
    imu_present = false;
  } else {
    imu_present = true;
    logts(); Serial.println("LSM6DSO32 OK");
    // ±32G accelerometer range — suits high-G launch events
    lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    // 26 Hz output data rate
    lsm.setAccelDataRate(LSM6DS_RATE_26_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_26_HZ);
    // ±2000 dps gyro range
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    logts(); Serial.printf("  Accel: ±32G  Gyro: ±2000dps  Rate: 26Hz\n");
  }

  // -- PA1010D GPS — Wire1 (STEMMA QT), address 0x10 --
  // GPS is non-fatal: unit continues without it, matching IMU behaviour.
  // Wire1 is already begun above; gps_sensor holds a reference to it.
  // NOTE: PA1010D can clock-stretch Wire1 for 30-65ms while processing
  // NMEA data, causing loop-max spikes. A future custom board should put
  // GPS on a dedicated I2C bus to isolate it from the scoring sensors.
  gps_sensor.begin(GPS_I2C_ADDR);
  // RMC + GGA gives us: fix quality, sats, HDOP, lat, lon, MSL altitude
  gps_sensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 1 Hz update — GPS is background telemetry, not timing-critical
  gps_sensor.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Probe: begin() on I2C always succeeds, so check for readable bytes
  delay(200);
  if (gps_sensor.available() > 0) {
    gps_present = true;
    logts(); Serial.println("PA1010D GPS OK");
  } else {
    // Second chance — module may need an extra moment after power-on
    delay(800);
    if (gps_sensor.available() > 0) {
      gps_present = true;
      logts(); Serial.println("PA1010D GPS OK (delayed)");
    } else {
      gps_present = false;
      logts(); Serial.println("WARNING: PA1010D GPS not found — unit will continue without GPS.");
      logts(); Serial.println("  Check STEMMA QT chain after LSM6DSO32.");
    }
  }

  // -- WiFi — STA mode (scorer network) or AP mode (direct phone access) --
  delay(1000);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  char ap_ssid[24];
  snprintf(ap_ssid, sizeof(ap_ssid), "F3K-Unit-%02d", cfg.unit_id);

#if FORCE_AP_MODE
  ap_mode = true;
  logts(); Serial.println("WiFi: FORCE_AP_MODE — starting hotspot");
#else
  // Try to connect to scorer network
  WiFi.mode(WIFI_STA);
  // Static IP: 192.168.8.unit_id — dedicated subnet, no DHCP needed
  IPAddress sta_ip(192, 168, 8, cfg.unit_id);
  IPAddress sta_gateway(192, 168, 8, 1);
  IPAddress sta_subnet(255, 255, 255, 0);
  IPAddress sta_dns(192, 168, 8, 1);
  WiFi.config(sta_ip, sta_gateway, sta_subnet, sta_dns);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  logts(); Serial.printf("Connecting to %s as 192.168.8.%d", WIFI_SSID, cfg.unit_id);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 15000) {
      logts(); Serial.println("\nNot found — falling back to AP mode");
      ap_mode = true;
      break;
    }
    delay(500); Serial.print(".");
  }
  logts(); Serial.println();
#endif

  if (ap_mode) {
    // ── AP mode — unit is the hotspot ──────────────────────────
    WiFi.mode(WIFI_AP);
    // Custom subnet: 192.168.XX.0 where XX = unit_id
    // e.g. unit 42 → gateway 192.168.42.1, unit 7 → 192.168.7.1
    IPAddress ap_ip(192, 168, cfg.unit_id, 1);
    IPAddress ap_gateway(192, 168, cfg.unit_id, 1);
    IPAddress ap_subnet(255, 255, 255, 0);
    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid);   // open network, no password
    delay(500);
    logts(); Serial.printf("AP mode: SSID=%s  IP=%s\n", ap_ssid, WiFi.softAPIP().toString().c_str());
    logts(); Serial.printf("Browse to: http://%s/pilot\n", WiFi.softAPIP().toString().c_str());
    logts(); Serial.println("Scorer UDP: DISABLED (AP mode)");
    // No modem sleep in AP mode — radio must stay active for connections
  } else {
    // ── STA mode — connected to scorer network ─────────────────
    logts(); Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    logts(); Serial.printf("UDP scorer → %s:%d\n", SERVER_IP, UDP_PORT);
    logts(); Serial.printf("UDP debug  → %s:%d\n", SERVER_IP, UDP_DBG_PORT);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    logts(); Serial.println("Modem sleep: ENABLED (WIFI_PS_MIN_MODEM)");
    udp.begin(UDP_PORT);
    udp_gps.begin(4211);   // Packet 2 — GPS fix
    udp_win.begin(WiFi.broadcastIP(), UDP_WIN_PORT);
    logts(); Serial.printf("UDP window listener on port %d (broadcast: %s)\n",
                  UDP_WIN_PORT, WiFi.broadcastIP().toString().c_str());
  }

  // -- Ensure log directory exists --
  if (!LittleFS.begin(false)) {
    logts(); Serial.println("LittleFS: mount failed in setup — logs unavailable.");
  } else {
    if (!LittleFS.exists(LOG_DIR)) {
      LittleFS.mkdir(LOG_DIR);
      logts(); Serial.println("LittleFS: created /logs directory");
    }
    LittleFS.end();
  }

  if (ap_mode) cfg.web_enabled = true;  // pilot UI always needs the web server

  // -- Async web server (optional — controlled by web_enabled config flag) --
  // Log endpoints always registered — scorer needs them regardless of web_enabled
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest* req){
    String ip = ap_mode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                  "<title>F3K Unit " + String(flight.unit_id) + " Logs</title>"
                  "<style>body{font-family:monospace;background:#1a1a1a;color:#e0e0e0;padding:20px;}"
                  "h2{color:#4db8ff;}table{border-collapse:collapse;width:100%;max-width:600px;}"
                  "th{background:#2a2a2a;color:#4db8ff;padding:8px 12px;text-align:left;border-bottom:2px solid #4db8ff;}"
                  "td{padding:7px 12px;border-bottom:1px solid #333;}"
                  "tr:hover td{background:#2a2a2a;}"
                  "a{color:#66ffaa;text-decoration:none;}a:hover{text-decoration:underline;}"
                  ".empty{color:#ff6666;}.size{color:#ffcc44;}</style></head><body>"
                  "<h2>F3K Unit " + String(flight.unit_id) + " — Flight Logs</h2>";

    if (LittleFS.begin(false)) {
      File dir = LittleFS.open(LOG_DIR);
      // Collect entries
      struct LogEntry { String name; size_t size; };
      LogEntry entries[50];
      int count = 0;
      if (dir) {
        File entry = dir.openNextFile();
        while (entry && count < 50) {
          String name = String(entry.name());
          if (name.endsWith(".csv")) {
            entries[count++] = { name, entry.size() };
          }
          entry = dir.openNextFile();
        }
      }
      LittleFS.end();

      if (count == 0) {
        html += "<p>No log files found.</p>";
      } else {
        // ── Sensor logs ──────────────────────────────────────────
        html += "<h3 style='color:#4db8ff;margin:0 0 0.5rem;'>Sensor Logs</h3>"
                "<table><tr><th>#</th><th>File</th><th>Size</th><th>Download</th><th>Delete</th></tr>";
        int row = 0;
        for (int i = 0; i < count; i++) {
          String name = entries[i].name;
          if (!name.startsWith("window_")) continue;
          String num = name.substring(7, 10);
          String url = "http://" + ip + "/log?n=" + num;
          String sizeStr = entries[i].size == 0
            ? "<span class='empty'>0 bytes (empty)</span>"
            : "<span class='size'>" + String(entries[i].size) + " bytes</span>";
          html += "<tr><td>" + String(++row) + "</td>"
                  "<td>" + name + "</td>"
                  "<td>" + sizeStr + "</td>"
                  "<td><a href='" + url + "' download='" +
                  "unit" + String(flight.unit_id) + "_" + name +
                  "'>⬇</a></td>"
                  "<td><a href='#' onclick=\"delFile('" + name + "',this)\" "
                  "style='color:#ff6666;'>✕</a></td></tr>";
        }
        if (row == 0) html += "<tr><td colspan='5'>No sensor logs found.</td></tr>";
        html += "</table>";

        // ── Score summaries ───────────────────────────────────────
        html += "<h3 style='color:#66ffaa;margin:1.2rem 0 0.5rem;'>Score Summaries</h3>"
                "<table><tr><th>#</th><th>File</th><th>Size</th><th>Download</th><th>Delete</th></tr>";
        int srow = 0;
        for (int i = 0; i < count; i++) {
          String name = entries[i].name;
          if (!name.startsWith("summary_")) continue;
          String num = name.substring(8, 11);
          String url = "http://" + ip + "/summary?n=" + num;
          String sizeStr = entries[i].size == 0
            ? "<span class='empty'>0 bytes (empty)</span>"
            : "<span class='size'>" + String(entries[i].size) + " bytes</span>";
          html += "<tr><td>" + String(++srow) + "</td>"
                  "<td>" + name + "</td>"
                  "<td>" + sizeStr + "</td>"
                  "<td><a href='" + url + "' download='" +
                  "unit" + String(flight.unit_id) + "_" + name +
                  "'>⬇</a></td>"
                  "<td><a href='#' onclick=\"delFile('" + name + "',this)\" "
                  "style='color:#ff6666;'>✕</a></td></tr>";
        }
        if (srow == 0) html += "<tr><td colspan='5'>No summaries found.</td></tr>";
        html += "</table>"
                "<p style='color:#666;font-size:0.8em;margin-top:0.8rem;'>"
                "Tap ✕ to delete a file &mdash; tap again to confirm.</p>"
                "<p style='margin-top:1.5rem;'>"
                "<a href='#' onclick=\"wipeAll(this);return false;\" "
                "style='display:inline-block;padding:8px 16px;background:#3a1a1a;"
                "color:#ff6666;border:1px solid #ff6666;border-radius:4px;"
                "text-decoration:none;font-weight:bold;'>"
                "Delete All Logs</a></p>";
      }
    } else {
      html += "<p style='color:#ff6666'>LittleFS unavailable.</p>";
    }

    html += "<p style='color:#666;margin-top:20px;font-size:0.85em'>"
            "Unit " + String(flight.unit_id) + " &nbsp;|&nbsp; "
            "Window " + String(cfg.window_number) + " &nbsp;|&nbsp; "
            "Free heap: " + String(ESP.getFreeHeap()) + " bytes</p>"
            "<script>"
            "function delFile(name,el){"
            "  if(el.dataset.confirm!='1'){"
            "    el.dataset.confirm='1';"
            "    el.textContent='Sure?';"
            "    el.style.color='#ffaa00';"
            "    setTimeout(()=>{if(el.dataset.confirm==='1'){"
            "      el.dataset.confirm='0';el.textContent='✕';el.style.color='#ff6666';"
            "    }},3000);"
            "    return;"
            "  }"
            "  fetch('/delete?f='+encodeURIComponent(name))"
            "  .then(r=>{"
            "    if(r.status===503){"
            "      el.dataset.confirm='0';el.textContent='Busy';el.style.color='#ffaa00';"
            "      setTimeout(()=>{el.textContent='✕';el.style.color='#ff6666';},2000);"
            "      return;"
            "    }"
            "    if(r.ok){"
            "      el.closest('tr').style.opacity='0.3';"
            "      el.textContent='✓';el.style.color='#3fb950';"
            "    } else { el.textContent='ERR'; }"
            "  }).catch(()=>{el.textContent='ERR';});"
            "}"
            "function wipeAll(el){"
            "  var s=el.dataset.stage||'0';"
            "  if(s==='0'){"
            "    el.dataset.stage='1';"
            "    el.textContent='Are you sure?';"
            "    el.style.background='#5a2a2a';"
            "    setTimeout(()=>{if(el.dataset.stage==='1'){"
            "      el.dataset.stage='0';el.textContent='Delete All Logs';el.style.background='#3a1a1a';"
            "    }},3000);"
            "    return;"
            "  }"
            "  if(s==='1'){"
            "    el.dataset.stage='2';"
            "    el.textContent='Really wipe ALL?';"
            "    el.style.background='#7a2a2a';"
            "    setTimeout(()=>{if(el.dataset.stage==='2'){"
            "      el.dataset.stage='0';el.textContent='Delete All Logs';el.style.background='#3a1a1a';"
            "    }},3000);"
            "    return;"
            "  }"
            "  fetch('/wipe-logs?confirm=YES&extra=SURE')"
            "  .then(r=>r.text())"
            "  .then(t=>{"
            "    el.textContent=t;el.style.background='#1a3a1a';el.style.color='#3fb950';"
            "    setTimeout(()=>location.reload(),1500);"
            "  }).catch(()=>{el.textContent='ERR';el.style.color='#ff6666';});"
            "}"
            "</script>"
            "</body></html>";

    req->send(200, "text/html", html);
  });

  // /delete?f=filename — delete a log or summary file by name
  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!req->hasParam("f")) {
      req->send(400, "text/plain", "Missing parameter: f");
      return;
    }
    // Refuse to delete while any file stream is active — deleting a file
    // mid-stream corrupts the transfer and can panic LittleFS.
    if (littlefs_streaming > 0) {
      req->send(503, "text/plain", "File transfer in progress — retry shortly");
      return;
    }
    String name = req->getParam("f")->value();
    // Safety: only allow deletion of window_ and summary_ csv files
    if ((!name.startsWith("window_") && !name.startsWith("summary_")) ||
        !name.endsWith(".csv")) {
      req->send(403, "text/plain", "Not permitted");
      return;
    }
    String path = String(LOG_DIR) + "/" + name;
    if (!LittleFS.begin(false)) {
      req->send(503, "text/plain", "LittleFS unavailable");
      return;
    }
    bool removed = false;
    if (LittleFS.exists(path)) {
      LittleFS.remove(path);
      removed = true;
      logts(); Serial.printf("[LOG] Deleted via UI: %s\n", path.c_str());
    }
    if (littlefs_streaming <= 0) LittleFS.end();
    if (removed) {
      req->send(200, "text/plain", "Deleted");
    } else {
      req->send(404, "text/plain", "Not found");
    }
  });

  // /wipe-logs — delete ALL window_*.csv and summary_*.csv files
  // Escape hatch when FS is full and openWindowLog can't start a new log.
  // Requires confirm=YES AND extra=SURE (matched in the UI by a 3-click
  // confirm pattern) so a single fat-fingered request can't wipe in error.
  // Refuses while any file is streaming or a window is currently open.
  server.on("/wipe-logs", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!req->hasParam("confirm") || req->getParam("confirm")->value() != "YES" ||
        !req->hasParam("extra")   || req->getParam("extra")->value()   != "SURE") {
      req->send(400, "text/plain", "Missing safety params (need confirm=YES&extra=SURE)");
      return;
    }
    if (littlefs_streaming > 0) {
      req->send(503, "text/plain", "File transfer in progress — retry shortly");
      return;
    }
    if (window_active) {
      req->send(503, "text/plain", "Window active — wipe refused");
      return;
    }
    if (!LittleFS.begin(false)) {
      req->send(503, "text/plain", "LittleFS unavailable");
      return;
    }
    // Collect names first — deleting mid-iteration is unsafe on LittleFS.
    // Heap-allocated so we don't drop 4-8KB on the AsyncTCP task stack.
    const int MAX_FILES = 256;
    String* victims = new (std::nothrow) String[MAX_FILES];
    if (!victims) {
      if (littlefs_streaming <= 0) LittleFS.end();
      req->send(500, "text/plain", "Out of memory");
      return;
    }
    int n = 0;
    bool overflowed = false;
    File dir = LittleFS.open(LOG_DIR);
    if (dir) {
      File entry = dir.openNextFile();
      while (entry) {
        String name = String(entry.name());
        if ((name.startsWith("window_") || name.startsWith("summary_")) &&
            name.endsWith(".csv")) {
          if (n < MAX_FILES) {
            victims[n++] = name;
          } else {
            overflowed = true;
            break;
          }
        }
        entry = dir.openNextFile();
      }
    }
    uint16_t deleted = 0;
    for (int i = 0; i < n; i++) {
      String path = String(LOG_DIR) + "/" + victims[i];
      if (LittleFS.remove(path)) deleted++;
    }
    delete[] victims;
    if (littlefs_streaming <= 0) LittleFS.end();
    logts(); Serial.printf("[LOG] WIPE: deleted %u files via UI%s\n",
                  deleted, overflowed ? " (more remain — click again)" : "");
    String msg = "Deleted " + String(deleted) + " files";
    if (overflowed) msg += " (more remain — click again)";
    req->send(200, "text/plain", msg);
  });

  // /pilot — pilot data collection page (AP mode)
  server.on("/pilot", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send_P(200, "text/html", PILOT_HTML);
  });

  // /wstatus — window status page (AP mode, served during active window)
  // Redirects to /pilot when no window is active.
  // Data is injected inline as window.__WS__ — no polling endpoint needed.
  server.on("/wstatus", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!window_active) {
      req->redirect("/pilot");
      return;
    }

    // Build window.__WS__ JSON inline — all values already in RAM,
    // no LittleFS reads except log_file.size() which is a metadata call
    // on the already-open file handle (fast, non-blocking).
    uint32_t elapsed_s = (millis() - window_start_ms) / 1000;

    String js = "<script>window.__WS__={";
    js += "unit_id:"     + String(cfg.unit_id)                           + ",";
    js += "elapsed_s:"   + String(elapsed_s)                             + ",";
    js += "win_secs:"    + String(window_secs)                           + ",";
    js += "log_open:"    + String(log_open ? "true" : "false")           + ",";
    js += "log_name:\""  + String(log_open ? log_path : "")              + "\",";
    js += "log_bytes:"   + String(log_open ? log_file.size() : 0)        + ",";
    js += "flight_count:" + String(flight_record_count)                  + ",";
    js += "gps_present:" + String(gps_present ? "true" : "false")        + ",";
    js += "gps_fix:"     + String(gps_fix     ? "true" : "false")        + ",";
    js += "gps_sats:"    + String(gps_sats)                              + ",";
    js += "flights:[";
    for (int i = 0; i < flight_record_count; i++) {
      if (i > 0) js += ",";
      js += "{num:"      + String(flight_records[i].flight_num)          + ",";
      js += "dur:"       + String(flight_records[i].duration_s,    1)    + ",";
      js += "throw_ft:"  + String(flight_records[i].throw_height_ft, 1)  + ",";
      js += "peak_ft:"   + String(flight_records[i].peak_alt_ft,   1)    + ",";
      js += "score:"     + String(flight_records[i].score,         1)    + "}";
    }
    js += "]}</script>";

    // Inject the data script into the PROGMEM HTML by sending in two parts
    String page = String(FPSTR(WSTATUS_HTML));
    page.replace("<script>", js + "<script>");
    req->send(200, "text/html", page);
  });

  // /setscore?m=0|1 — select scoring formula at runtime
  // m=0: Secs-Ft  duration_s - throw_height_ft, window score = sum
  // m=1: JoeD V1 (dur/180)^0.425*1000 ± height component, window score = average
  server.on("/setscore", HTTP_GET, [](AsyncWebServerRequest* req){
    if (req->hasParam("m")) {
      uint8_t m = req->getParam("m")->value().toInt();
      if (m == 0 || m == 1) {
        score_mode = m;
        logts(); Serial.printf("[SCORE] Formula: %s\n",
                      score_mode == 1 ? "JoeD V1 (avg of flights)" : "Secs-Ft (sum of flights)");
      }
    }
    req->send(200, "application/json",
              String("{\"score_mode\":") + score_mode + "}");
  });

  // /settilt?v=0|1|2 — select input mode at runtime
  // v=0: Normal (real sensors, full calibration)
  // v=1: Tilt sim (altitude from IMU tilt angle)
  // v=2: Parabola sim (autonomous flight cycles, no physical input needed)
  server.on("/settilt", HTTP_GET, [](AsyncWebServerRequest* req){
    if (req->hasParam("v")) {
      uint8_t newSim = (uint8_t)constrain(req->getParam("v")->value().toInt(), 0, 2);
      if (newSim != sim_mode) {
        sim_mode  = newSim;
        tilt_mode = (sim_mode > 0);
        // Reset calibration state for any mode change
        calibration_done   = false;
        cal_start_ms       = 0;
        cal_count          = 0;
        last_landed_alt_ft = 0.0f;  // reset so throw height is clean after mode switch
        if (!tilt_mode) {
          // Returning to normal — full barometric recalibration
          flight.state = STATE_CALIBRATING;
        }
        const char* modeNames[] = {"Normal", "Tilt Sim", "Parabola Sim"};
        logts(); Serial.printf("[MODE] Input mode: %s\n", modeNames[sim_mode]);
      }
    }
    req->send(200, "application/json",
              String("{\"sim_mode\":") + sim_mode +
              ",\"tilt_mode\":"       + (tilt_mode ? "true" : "false") + "}");
  });

  // /pstatus — lightweight JSON for pilot page (1 Hz polling)
  server.on("/pstatus", HTTP_GET, [](AsyncWebServerRequest* req){
    // Total score — sum for linear, average for JoeD V1
    float total_score = 0;
    if (flight_record_count > 0) {
      for (int i = 0; i < flight_record_count; i++) total_score += flight_records[i].score;
      if (score_mode == 1) total_score /= flight_record_count;  // JoeD V1: average
    }

    String j = "{";
    j += "\"unit_id\":"    + String(flight.unit_id)                         + ",";
    j += "\"state\":"      + String((int)flight.state)                      + ",";
    j += "\"state_name\":\"" + String(stateNames[flight.state])             + "\",";
    j += "\"flight_num\":" + String(flight_counter)                         + ",";
    j += "\"flight_t\":"   + String(flight.flight_duration_ms/1000.0f, 1)  + ",";
    j += "\"throw_ht\":"   + String(flight.throw_height_ft, 1)              + ",";
    j += "\"alt_tared\":"  + String(flight.altitude_ft - tare_baseline_ft, 1) + ",";
    j += "\"win_active\":" + String(window_active ? "true" : "false")       + ",";
    j += "\"win_secs\":"   + String(window_secs)                            + ",";
    j += "\"prep_active\":" + String(prep_active ? "true" : "false")        + ",";
    j += "\"prep_remain\":" + String(prep_active ? max(0L, (long)(prep_fire_ms - millis()) / 1000 + 1) : 0) + ",";
    j += "\"countdown\":"  + String(window_countdown_active ? "true" : "false") + ",";
    j += "\"countdown_remain\":";
    if (window_countdown_active) {
      uint32_t elapsed = millis() - window_countdown_start;
      uint32_t remain  = elapsed < WINDOW_COUNTDOWN_MS ? (WINDOW_COUNTDOWN_MS - elapsed) / 1000 + 1 : 0;
      j += String(remain);
    } else {
      j += "0";
    }
    j += ",";
    j += "\"log_ready\":"     + String((!window_active && pilot_download_path.length() > 0) ? "true" : "false") + ",";
    j += "\"log_num\":"       + String(cfg.window_number)                      + ",";
    j += "\"tilt_mode\":"  + String(tilt_mode ? "true" : "false")          + ",";
    j += "\"sim_mode\":"   + String(sim_mode)                               + ",";
    j += "\"score_mode\":" + String(score_mode)                              + ",";
    j += "\"wifi_active\":" + String((wifi_active && !wifi_shutdown_pending) ? "true" : "false") + ",";
    j += "\"total_score\":" + String(total_score, 1)                        + ",";
    j += "\"flights\":[";
    for (int i = 0; i < flight_record_count; i++) {
      if (i > 0) j += ",";
      j += "{\"num\":"   + String(flight_records[i].flight_num)       + ",";
      j += "\"dur\":"    + String(flight_records[i].duration_s, 1)    + ",";
      j += "\"throw\":"  + String(flight_records[i].throw_height_ft, 1) + ",";
      j += "\"peak\":"   + String(flight_records[i].peak_alt_ft, 1)   + ",";
      j += "\"score\":"  + String(flight_records[i].score, 1)         + "}";
    }
    j += "]}";
    req->send(200, "application/json", j);
  });

  // /pgps — GPS debug JSON for the GPS tab (1 Hz polling)
  server.on("/pgps", HTTP_GET, [](AsyncWebServerRequest* req){
    String j = "{";
    j += "\"present\":"     + String(gps_present    ? "true" : "false") + ",";
    j += "\"fix\":"         + String(gps_fix         ? "true" : "false") + ",";
    j += "\"fix_quality\":" + String(gps_fix_quality)                   + ",";
    j += "\"satellites\":"  + String(gps_sats)                          + ",";
    j += "\"hdop\":"        + String(gps_hdop, 1)                       + ",";
    j += "\"lat\":"         + String(gps_lat,  6)                       + ",";
    j += "\"lon\":"         + String(gps_lon,  6)                       + ",";
    j += "\"alt_m\":"       + String(gps_alt_m, 1)                      + "}";
    req->send(200, "application/json", j);
  });

  // /pstart?secs=NNN — start a window from pilot UI (5s countdown then opens)
  server.on("/pstart", HTTP_GET, [](AsyncWebServerRequest* req){
    uint16_t secs = 300;
    if (req->hasParam("secs")) secs = req->getParam("secs")->value().toInt();
    secs = constrain(secs, 30, 3600);
    if (window_active) closeWindowLog();
    flight_record_count      = 0;
    pilot_download_path      = "";
    window_countdown_secs    = secs;
    window_countdown_start   = millis();
    window_countdown_active  = true;
    logts(); Serial.printf("[WIN] Countdown started: %ds window in %ds\n",
                  secs, WINDOW_COUNTDOWN_MS / 1000);
    req->send(200, "text/plain", "OK");
  });

  // /pstop — end window early or cancel countdown
  server.on("/pstop", HTTP_GET, [](AsyncWebServerRequest* req){
    if (window_countdown_active) {
      window_countdown_active = false;
      logts(); Serial.println("[WIN] Countdown cancelled");
    } else if (window_active) {
      logts(); Serial.println("[WIN] Pilot stopped window early");
      closeWindowLog();
    }
    req->send(200, "text/plain", "OK");
  });
  // /summary?n=NNN — serve score summary CSV (kept on device for historical reference)
  server.on("/summary", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!LittleFS.begin(false)) {
      req->send(503, "text/plain", "LittleFS unavailable");
      return;
    }
    int num = req->hasParam("n") ? req->getParam("n")->value().toInt() : cfg.window_number;
    char spath[36];
    snprintf(spath, sizeof(spath), "%s/summary_%03d.csv", LOG_DIR, num);
    if (!LittleFS.exists(spath)) {
      LittleFS.end();
      req->send(404, "text/plain", "Summary not found");
      return;
    }
    AsyncWebServerResponse* resp = req->beginResponse(LittleFS, spath, "text/csv");
    char cd[72];
    snprintf(cd, sizeof(cd), "attachment; filename=\"unit%02d_summary_%03d.csv\"",
             flight.unit_id, num);
    resp->addHeader("Content-Disposition", cd);
    resp->addHeader("Connection", "close");
    // Increment stream counter before sending; decrement in onDisconnect.
    // LittleFS stays mounted until the last active stream completes.
    // Also disable modem sleep for the duration — WIFI_PS_MIN_MODEM pauses
    // the radio mid-transfer at ~204KB causing IncompleteRead on the scorer.
    littlefs_streaming++;
    if (littlefs_streaming == 1) esp_wifi_set_ps(WIFI_PS_NONE);
    req->onDisconnect([](){
      if (--littlefs_streaming <= 0) {
        littlefs_streaming = 0;
        LittleFS.end();
        if (!window_active) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
      }
    });
    req->send(resp);
    logts(); Serial.printf("[SUMMARY] Served %s\n", spath);
  });

  // /log?n=NNN&del=1 — serve CSV, optionally delete after download
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest* req){
    if (!req->hasParam("n")) {
      req->send(400, "text/plain", "Missing parameter: n");
      return;
    }
    // Guard: if the log file is still open (window just closed, still flushing)
    // return 503 so the scorer's retry queue handles it gracefully.
    if (log_open) {
      req->send(503, "text/plain", "Log still open — retry shortly");
      return;
    }
    String num = req->getParam("n")->value();
    bool del_after = req->hasParam("del") && req->getParam("del")->value() == "1";
    while (num.length() < 3) num = "0" + num;
    String path = String(LOG_DIR) + "/window_" + num + ".csv";

    if (!LittleFS.begin(false)) {
      logts(); Serial.println("[LOG] HTTP: LittleFS mount failed");
      req->send(503, "text/plain", "LittleFS unavailable");
      return;
    }
    if (!LittleFS.exists(path)) {
      logts(); Serial.printf("[LOG] HTTP: file not found: %s\n", path.c_str());
      LittleFS.end();
      req->send(404, "text/plain", "Log not found: " + path);
      return;
    }
    logts(); Serial.printf("[LOG] Serving %s → %s%s\n",
                  path.c_str(), req->client()->remoteIP().toString().c_str(),
                  del_after ? " (delete after)" : "");
    AsyncWebServerResponse* response = req->beginResponse(LittleFS, path, "text/csv");
    response->addHeader("Content-Disposition",
                        "attachment; filename=\"unit" + String(flight.unit_id) +
                        "_window_" + num + ".csv\"");
    response->addHeader("Connection", "close");
    littlefs_streaming++;
    if (littlefs_streaming == 1) esp_wifi_set_ps(WIFI_PS_NONE);
    if (del_after) {
      String pathCopy = path;
      req->onDisconnect([pathCopy](){
        if (--littlefs_streaming <= 0) {
          littlefs_streaming = 0;
          if (LittleFS.begin(false)) {
            LittleFS.remove(pathCopy);
            logts(); Serial.printf("[LOG] Deleted %s after download\n", pathCopy.c_str());
            LittleFS.end();
          }
          if (!window_active) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        }
        pilot_download_path = "";
      });
    } else {
      req->onDisconnect([](){
        if (--littlefs_streaming <= 0) {
          littlefs_streaming = 0;
          LittleFS.end();
          if (!window_active) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        }
      });
    }
    req->send(response);
  });

  // /debug — full telemetry overlay (used by pilot page telemetry tab)
  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send_P(200, "text/html", PAGE_HTML);
  });

  if (cfg.web_enabled) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
      req->send_P(200, "text/html", PAGE_HTML);
    });
    server.on("/json", HTTP_GET, [](AsyncWebServerRequest* req){
      const String& json = (active_buf == 0) ? batch_json_a : batch_json_b;
      req->send(200, "application/json", json);
    });
    server.on("/tare", HTTP_GET, [](AsyncWebServerRequest* req){
      tare_requested = true;
      req->send(200, "text/plain", "OK");
    });
    server.onNotFound([](AsyncWebServerRequest* req){
      req->send(404, "text/plain", "Not found");
    });
    server.begin();
    if (ap_mode) {
      logts(); Serial.printf("Web (AP): http://%s/\n", WiFi.softAPIP().toString().c_str());
    } else {
      logts(); Serial.printf("Web overlay: http://%s/\n", WiFi.localIP().toString().c_str());
    }
  } else {
    // Start server anyway for log endpoints — just no debug overlay
    server.onNotFound([](AsyncWebServerRequest* req){
      req->send(404, "text/plain", "Not found");
    });
    server.begin();
    if (ap_mode) {
      logts(); Serial.printf("Log server (AP): http://%s/logs\n", WiFi.softAPIP().toString().c_str());
    } else {
      logts(); Serial.printf("Log server: http://%s/logs  (log retrieval: /log?n=NNN)\n", WiFi.localIP().toString().c_str());
      logts(); Serial.println("Web overlay: DISABLED (field mode)");
    }
  }

  last_sensor_ms  = millis();
  last_imu_ms     = millis() + 20;
  last_diag_ms    = millis() + 500;            // first diag after 0.5s
  last_udp_ms     = millis();
  last_dbg_ms     = millis() + DISPLAY_OFFSET_MS;
  last_display_ms = millis() + DISPLAY_OFFSET_MS;

  // ── Hardware timers ──────────────────────────────────────────
  // Timer 2: window open (armed from prep countdown, fires ISR to latch timestamp)
  // Timer 3: window close (armed at window open, fires ISR to latch timestamp)
  // v3.x API: timerBegin(freq_hz) — 1000 Hz = 1 ms resolution.
  // timerAttachInterrupt takes only (timer, ISR) — no edge parameter.
  _timer_open  = timerBegin(1000);
  timerAttachInterrupt(_timer_open,  &onWindowOpenTimer);
  _timer_close = timerBegin(1000);
  timerAttachInterrupt(_timer_close, &onWindowCloseTimer);
  logts(); Serial.println("[HW] Window timers ready (Timer_open  Timer_close  1ms tick)");
}

// ============================================================
//  loop()
// ============================================================
void loop() {
  unsigned long loop_start = micros();
  bool did_work = false;
  unsigned long now = millis();

  // ── Hardware timer ISR handlers ───────────────────────────────
  // These flags are set by the timer ISR at the exact microsecond the
  // window should open or close, independent of main loop blocking.
  // We service them here at the top of the loop for minimum latency.

  if (window_open_pending && !window_active) {
    window_open_pending  = false;
    window_open_latch_ms = millis();  // captured at first loop pass after ISR — <1ms latency
    window_start_ms = (unsigned long)window_open_latch_ms;
    window_secs     = prep_window_secs;
    window_id       = prep_window_id;
    window_active   = true;
    prep_active     = false;
    logts(); Serial.printf("[HW] Window open ISR fired — start_ms=%lu  id=%u  %ds\n",
                  window_start_ms, window_id, window_secs);
    openWindowLog();
    saveConfig();
    did_work = true;
  }

  if (window_close_pending && window_active) {
    window_close_pending  = false;
    window_close_latch_ms = millis();  // captured at first loop pass after ISR — <1ms latency
    logts(); Serial.printf("[HW] Window close ISR fired — elapsed=%lums\n",
                  (unsigned long)window_close_latch_ms - window_start_ms);
    logts(); Serial.println("[WIN] Window elapsed — closing log");
    // Override window_close_ms with the ISR-latched value for accuracy
    window_close_ms = (unsigned long)window_close_latch_ms;
    closeWindowLog();
    did_work = true;
  }

  // ── IMU read at 26 Hz ────────────────────────────────────
  if (now - last_imu_ms >= IMU_INTERVAL) {
    last_imu_ms = now;
    readImu();
    did_work = true;
  }

  // ── GPS — drain NMEA buffer and parse (non-blocking) ─────
  // read() pulls one byte at a time from Wire1. GPS outputs at 1 Hz,
  // so polling every 100ms is more than sufficient to catch every
  // sentence. Polling at 20ms was causing 50 Wire1 requestFrom calls
  // per second — increasing contention with the IMU and DPS310 on
  // the shared bus, and increasing the frequency of GPS clock-stretch
  // events hitting the loop timing.
  if (gps_present && (now - last_gps_ms >= 100)) {
    last_gps_ms = now;
    for (int i = 0; i < 32; i++) gps_sensor.read();
    if (gps_sensor.newNMEAreceived()) {
      if (gps_sensor.parse(gps_sensor.lastNMEA())) {
        gps_fix_quality = gps_sensor.fixquality;
        gps_fix         = (gps_fix_quality > 0);
        if (gps_fix) {
          gps_lat   = gps_sensor.latitudeDegrees  * (gps_sensor.lat  == 'N' ? 1.0f : -1.0f);
          gps_lon   = gps_sensor.longitudeDegrees * (gps_sensor.lon  == 'E' ? 1.0f : -1.0f);
          gps_alt_m = gps_sensor.altitude;
          gps_sats  = gps_sensor.satellites;
          gps_hdop  = gps_sensor.HDOP;
        }
        did_work = true;   // only count a parse as real work, not every drain pass
      }
    }
    // did_work intentionally NOT set here for the common no-sentence case
  }

  // ── Sensor read at 8 Hz ───────────────────────────────────
  if (now - last_sensor_ms >= SENSOR_INTERVAL) {
    last_sensor_ms = now;
    if (!dps_present) {
      // No DPS310 — state machine still needs to run; feed zero altitude
      updateStateMachine(tare_baseline_ft);
      did_work = true;
    } else {
    sensors_event_t temp_event, pressure_event;
    dps.getEvents(&temp_event, &pressure_event);

    if (pressure_event.pressure > 800 && pressure_event.pressure < 1100) {
      float alt_ft = pressureToAltitudeFeet(pressure_event.pressure);

      if (tilt_mode) {
        // ── Simulation modes (tilt_mode=true, sim_mode=1 or 2) ───────

        if (!calibration_done) {
          calibration_done   = true;
          tare_baseline_ft   = 0.0f;
          last_landed_alt_ft = 0.0f;  // reset so throw height computes correctly from sim baseline
          flight.state       = STATE_GROUND;
          if (sim_mode == 2) {
            logts(); Serial.printf("[SIM] Task-aware sim ready — task_id=%u (%s)  R%u G%u\n",
                          contest_task_id, taskName(contest_task_id),
                          contest_round_num, contest_group_num);
            logts(); Serial.println("[SIM]   Flight durations from task plan +/-10s jitter  Launch: 30-60ft");
            logts(); Serial.printf("[SIM]   tare_baseline_ft=%.1f  last_landed_alt_ft=%.1f\n",
                          tare_baseline_ft, last_landed_alt_ft);
          } else {
            logts(); Serial.println("[STATE] TILT MODE — skipping calibration → GROUND");
          }
        }

        if (sim_mode == 2) {
          // ── Task-aware flight simulation — fully autonomous ─────────
          // Generates realistic flight durations from the current contest_task_id.
          // Each flight uses a target drawn from the task plan +/- 10s random jitter.
          // Altitude profile: fast launch climb → soar near peak → gradual descent.
          // Only cycles during an active window; sits at GROUND outside a window.

          // ── Static state ────────────────────────────────────────────
          static unsigned long sim_phase_ms    = 0;
          static bool          sim_in_flight   = false;
          static uint8_t       sim_flight_idx  = 0;
          static uint32_t      sim_flight_ms   = 0;
          static float         sim_launch_ft   = 0.0f;
          static float         sim_peak_ft     = 0.0f;
          static uint8_t       last_sim_mode2  = 255;
          static uint8_t       last_task_id    = 255;
          static bool          last_win_active = false;  // detect window open edge
          static uint8_t       sim_task_snap   = 0;      // task_id snapshotted at window open

          const uint32_t SIM_GROUND_MS = 5000UL;  // 5s between flights

          // ── Task flight plan lookup ──────────────────────────────────
          // Returns list of target durations (seconds) for the active task.
          // Stored in flash via PROGMEM-style const arrays.
          // sim_flight_idx cycles through the list; wraps for unlimited tasks.
          auto getTaskTarget = [&](uint8_t idx) -> uint16_t {
            // Uses sim_task_snap — the task_id snapshotted at window open,
            // not the live contest_task_id which may update mid-window.
            const uint16_t tLL[]  = {60,60,60,60,60};
            const uint16_t tA[]   = {300};
            const uint16_t tB[]   = {240,240};
            const uint16_t tC3[]  = {180,180,180};
            const uint16_t tC4[]  = {180,180,180,180};
            const uint16_t tC5[]  = {180,180,180,180,180};
            const uint16_t tD[]   = {30,45,60,75,90,105,120};
            const uint16_t tE[]   = {200,200,200,200,200};
            const uint16_t tF[]   = {180,180,180,180,180,180};
            const uint16_t tG[]   = {120,120,120,120,120,120,120};
            const uint16_t tH[]   = {60,120,180,240};
            const uint16_t tI[]   = {200,200,200,200,200};
            const uint16_t tJ[]   = {180,180,180};
            const uint16_t tK[]   = {60,90,120,150,180,210,240};
            const uint16_t tD2[]  = {300,300};
            const uint16_t tE2[]  = {200,200,200};
            const uint16_t tL[]   = {599};
            const uint16_t tM[]   = {180,300,420};
            const uint16_t tN[]   = {599};
            const uint16_t tL2[]  = {419};
            const uint16_t tB2[]  = {180,180};
            const uint16_t tA2[]  = {300};

            const uint16_t* list  = tLL;
            uint8_t         count = 5;

            switch (sim_task_snap) {
              case  6: list=tA;   count=1; break;
              case  7: list=tB;   count=2; break;
              case  8: list=tC3;  count=3; break;
              case  9: list=tD;   count=7; break;
              case 10: list=tE;   count=5; break;
              case 11: list=tF;   count=6; break;
              case 12: list=tG;   count=7; break;
              case 13: list=tH;   count=4; break;
              case 14: list=tI;   count=5; break;
              case 15: list=tJ;   count=3; break;
              case 17: list=tC4;  count=4; break;
              case 18: list=tC5;  count=5; break;
              case 19: list=tA2;  count=1; break;
              case 20: list=tB2;  count=2; break;
              case 21: list=tK;   count=7; break;
              case 26: list=tD2;  count=2; break;
              case 27: list=tE2;  count=3; break;
              case 28: list=tE2;  count=3; break;
              case 29: list=tL;   count=1; break;
              case 30: list=tM;   count=3; break;
              case 33: list=tN;   count=1; break;
              case 34: list=tL2;  count=1; break;
              default: list=tLL;  count=5; break;
            }
            return list[idx % count];
          };

          // ── Seed a new flight from task plan ────────────────────────
          auto seedNewFlight = [&]() {
            uint16_t target_s = getTaskTarget(sim_flight_idx);
            // +/- 10s jitter using unit_id + flight_idx as deterministic seed
            // (no rand() to avoid seeding complexity; pseudo-random enough for sim)
            int8_t jitter = (int8_t)(((cfg.unit_id * 7 + sim_flight_idx * 13) % 21) - 10);
            uint16_t dur_s = (uint16_t)constrain((int)target_s + jitter, 5, 599);
            sim_flight_ms  = (uint32_t)dur_s * 1000UL;
            // Launch height: 30–60ft, varied by unit_id + flight index
            sim_launch_ft  = 30.0f + (float)((cfg.unit_id * 11 + sim_flight_idx * 17) % 31);
            // Peak: launch height + 20–40ft additional climb
            sim_peak_ft    = sim_launch_ft + 20.0f + (float)((cfg.unit_id * 5 + sim_flight_idx * 7) % 21);
            sim_flight_idx++;
            logts(); Serial.printf("[SIM] Flight %u: task=%u target=%us jitter=%+ds actual=%us launch=%.0fft peak=%.0fft tare=%.1fft last_landed=%.1fft\n",
                          sim_flight_idx, sim_task_snap, target_s, jitter, dur_s, sim_launch_ft, sim_peak_ft,
                          tare_baseline_ft, last_landed_alt_ft);
          };

          // ── Detect mode change ──────────────────────────────────────
          if (last_sim_mode2 != 2) {
            sim_phase_ms    = millis();
            sim_in_flight   = false;
            sim_flight_idx  = 0;
            last_sim_mode2  = 2;
            last_task_id    = 255;
            last_win_active = false;
            logts(); Serial.println("[SIM] Task sim activated");
          }

          if (!window_active) {
            // Outside window — hold at ground, reset for clean start on next window
            alt_ft          = tare_baseline_ft;
            sim_in_flight   = false;
            sim_flight_idx  = 0;
            sim_phase_ms    = millis();
            last_win_active = false;
          } else {
            // ── Window-open edge and task-change detection ──────────────
            // Snapshot contest_task_id at the moment the window opens or
            // when the 0x20 grace period updates it. This ensures the sim
            // uses the authoritative task even when the scorer's 0x21 prep
            // packets hadn't yet been updated to send ICD v1.7 task bytes.
            bool win_edge     = (window_active && !last_win_active);
            bool task_changed = (contest_task_id != last_task_id);
            last_win_active   = true;

            if (win_edge || task_changed) {
              sim_task_snap  = contest_task_id;
              last_task_id   = contest_task_id;
              sim_flight_idx = 0;
              sim_in_flight  = false;
              sim_phase_ms   = millis();
              logts(); Serial.printf("[SIM] Task snapshot: task_id=%u (%s)  R%u G%u\n",
                            sim_task_snap, taskName(sim_task_snap),
                            contest_round_num, contest_group_num);
            }

            unsigned long phase_ms = millis() - sim_phase_ms;

            if (!sim_in_flight) {
              // Ground pause phase — hold at ground until throw
              // Last 500ms of the pause simulates the throw arc rising to
              // sim_launch_ft so the state machine sees a smooth ascent
              // rather than an instantaneous jump that confuses the tare logic.
              if (phase_ms < SIM_GROUND_MS - 500UL) {
                alt_ft = tare_baseline_ft;
              } else {
                // Rising throw arc: 0 → sim_launch_ft over 500ms
                // We don't have sim_launch_ft yet (seedNewFlight not called),
                // so use a fixed 40ft target for the throw preview.
                float throw_frac = (float)(phase_ms - (SIM_GROUND_MS - 500UL)) / 500.0f;
                alt_ft = tare_baseline_ft + 40.0f * sinf(throw_frac * (PI / 2.0f));
              }
              if (phase_ms >= SIM_GROUND_MS) {
                seedNewFlight();
                sim_in_flight = true;
                sim_phase_ms  = millis();
              }
            } else {
              // ── Flight arc phase ─────────────────────────────────────
              if (phase_ms >= sim_flight_ms) {
                // Flight complete
                alt_ft        = tare_baseline_ft;
                sim_in_flight = false;
                sim_phase_ms  = millis();
                logts(); Serial.println("[SIM] Landed");
              } else {
                // Piecewise realistic altitude profile:
                //   0% – 20%: sinusoidal climb from launch_ft to peak_ft
                //   20% – 80%: power-law gentle sink from peak to ~15ft
                //   80% – 100%: linear descent from ~15ft to 5ft (landing)
                float t = (float)phase_ms / (float)sim_flight_ms;  // 0.0 → 1.0

                float a;
                if (t <= 0.20f) {
                  // Fast climb — sin curve from launch_ft up to peak_ft
                  float frac = t / 0.20f;
                  a = sim_launch_ft + (sim_peak_ft - sim_launch_ft) * sinf(frac * (PI / 2.0f));
                } else if (t <= 0.80f) {
                  // Gradual soar/sink — power curve from peak down to ~15ft
                  float frac = (t - 0.20f) / 0.60f;
                  float sink_to = 15.0f + tare_baseline_ft;
                  a = sim_peak_ft - (sim_peak_ft - sink_to) * powf(frac, 1.5f);
                } else {
                  // Final descent — linear from ~15ft to 5ft
                  float frac = (t - 0.80f) / 0.20f;
                  a = (15.0f + tare_baseline_ft) - 10.0f * frac;
                }
                alt_ft = max(a, tare_baseline_ft + 4.0f); // floor at 4ft — must cross 5ft threshold to trigger landing
              }
            }
          }

        } else {
          // ── Tilt simulation (mode 1) — altitude from IMU tilt angle ─
          // 0–15° → 0ft (GROUND)  15–45° → 10ft (LAUNCH_WIN)  45–90° → tilt°=ft
          if (imu.valid) {
            float t = imu.tilt_deg;
            if      (t < 15.0f) alt_ft = tare_baseline_ft + 0.0f;
            else if (t < 45.0f) alt_ft = tare_baseline_ft + 10.0f;
            else                alt_ft = tare_baseline_ft + t;
          }
        }

        flight.altitude_ft = alt_ft;
        live_temperature_c = temp_event.temperature;
        if (buf_count < BUF_SIZE) {
          buf_pressure[buf_count]    = pressure_event.pressure;
          buf_temperature[buf_count] = temp_event.temperature;
          buf_count++;
        }
        updateStateMachine(alt_ft);

      } else {
        // ── Normal / production operation ────────────────────────
        if (!calibration_done) {
          if (cal_start_ms == 0) {
            cal_start_ms = millis();
            logts(); Serial.print("Calibrating ");
          }
          if (cal_count < CAL_BUF_SIZE) {
            cal_buf[cal_count++] = alt_ft;
            if (cal_count % 8 == 0) Serial.print(".");
          }
          if (millis() - cal_start_ms >= CALIBRATION_MS && cal_count > 0) {
            float sum = 0;
            for (int i = 0; i < cal_count; i++) sum += cal_buf[i];
            tare_baseline_ft = sum / cal_count;
            calibration_done = true;
            logts(); Serial.printf(" done. Baseline: %.2f ft (%d samples)\n",
                          tare_baseline_ft, cal_count);
          }
          flight.altitude_ft = alt_ft;
          live_temperature_c = temp_event.temperature;
          updateStateMachine(alt_ft);
        } else {
          flight.altitude_ft = alt_ft;
          live_temperature_c = temp_event.temperature;
          if (buf_count < BUF_SIZE) {
            buf_pressure[buf_count]    = pressure_event.pressure;
            buf_temperature[buf_count] = temp_event.temperature;
            buf_count++;
          }
          updateStateMachine(alt_ft);
        }
      }

      // Log sample whenever window is active.
      // For windows > 600s, reduce log rate to 4 Hz (every other sample)
      // to halve file size and stay within LittleFS budget.
      // State machine always runs at full 8 Hz regardless.
      if (window_active) {
        static uint8_t log_skip = 0;
        bool do_log = (window_secs <= 600) || ((log_skip++ & 1) == 0);
        if (do_log) {
          logSample(alt_ft, pressure_event.pressure, temp_event.temperature);
        }
      }

    }
    } // end dps_present else
    did_work = true;
  }

  // ── Window countdown — fires window after 5s delay ───────────
  if (window_countdown_active &&
      now - window_countdown_start >= WINDOW_COUNTDOWN_MS) {
    window_countdown_active = false;
    window_secs             = window_countdown_secs;
    window_id               = now;
    window_start_ms         = now;   // use cached now — no underflow risk
    window_active           = true;
    logts(); Serial.printf("[WIN] Countdown complete — opening window: %ds  id=%u\n",
                  window_secs, window_id);
    openWindowLog();
  }

  // ── Window command listener ───────────────────────────────
  if (!ap_mode) checkWindowCommand(now);

  // ── Prep countdown timer — fallback if ISR missed ────────────
  // Normally the hardware timer ISR fires window_open_pending above.
  // This millis() check is a fallback in case the ISR was missed.
  if (prep_active && !window_active && !window_open_pending &&
      (long)(now - prep_fire_ms) >= 0) {
    prep_active = false;
    window_secs     = prep_window_secs;
    window_id       = prep_window_id;
    window_start_ms = now;
    window_active   = true;
    logts(); Serial.printf("[WIN] Prep countdown fired (fallback) — opening window: %ds  id=%u\n",
                  window_secs, window_id);
    openWindowLog();
    saveConfig();
  }

  // ── Log pre-open during prep final 15 seconds ─────────────────
  // Open the log file during prep (10-15s before window) so openWindowLog()
  // can skip all blocking LittleFS I/O at the critical moment. The file
  // is created with headers; openWindowLog() just resets counters and anchors.
  // Only runs once per prep (log_preopen_done flag), and only when healthy.
  if (prep_active && !window_active && !log_preopen_done &&
      littlefs_streaming == 0 && !log_open) {
    long remaining_ms = (long)prep_fire_ms - (long)now;
    if (remaining_ms > 0 && remaining_ms <= 15000) {
      // Pre-increment window number to get the correct log path
      uint16_t next_num = cfg.window_number + 1;
      char pre_path[32];
      snprintf(pre_path, sizeof(pre_path), "%s/window_%03d.csv", LOG_DIR, next_num);

      if (LittleFS.begin(false)) {
        size_t free_bytes = LittleFS.totalBytes() - LittleFS.usedBytes();
        float  log_rate   = (prep_window_secs > 600) ? 1200.0f : 2400.0f;
        size_t needed     = max((size_t)204800, (size_t)(prep_window_secs * log_rate));

        if (free_bytes >= needed) {
          if (!LittleFS.exists(LOG_DIR)) LittleFS.mkdir(LOG_DIR);
          log_file = LittleFS.open(pre_path, "w");
          if (log_file) {
            char ctx[160];
            if (contest_round_num > 0) {
              snprintf(ctx, sizeof(ctx), "# Round %u, Group %u | Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
                       contest_round_num, contest_group_num,
                       taskName(contest_task_id), contest_task_id,
                       prep_window_secs, flight.unit_id);
            } else {
              snprintf(ctx, sizeof(ctx), "# Task: %s (task_id=%u) | Window: %us | Unit: %u\n",
                       taskName(contest_task_id), contest_task_id,
                       prep_window_secs, flight.unit_id);
            }
            log_file.print(ctx);
            log_file.print("t_ms,flight,flight_t_s,state,throw_height_ft,alt_ft,alt_tared_ft,pressure_hpa,temp_c,"
                           "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,g_total,tilt_deg,"
                           "gps_lat,gps_lon,gps_alt_m,gps_sats\n");
            log_file.flush();  // commit headers to flash now
            log_open         = true;
            log_preopen_done = true;
            logts(); Serial.printf("[LOG] Pre-opened %s  (%ldms before window)\n",
                          pre_path, remaining_ms);
          } else {
            LittleFS.end();
          }
        } else {
          logts(); Serial.printf("[LOG] Pre-open skipped — insufficient space (%u free)\n",
                        free_bytes);
          LittleFS.end();
        }
      }
    }
  }

  // ── Window timeout check — fallback if ISR missed ────────────
  // Normally the hardware timer ISR fires window_close_pending above.
  if (window_active && window_secs > 0 && !window_close_pending) {
    uint32_t elapsed = now - window_start_ms;
    uint32_t target  = (uint32_t)window_secs * 1000UL;
    if (elapsed >= target) {
      logts(); Serial.printf("[WIN] Timeout fallback: elapsed=%ums target=%ums\n",
                    elapsed, target);
      logts(); Serial.println("[WIN] Window elapsed — closing log");
      closeWindowLog();
    }
  }

  // ── AP in-flight radio manager ────────────────────────────
  // AP mode only. During a window the AP follows flight state:
  //   STATE_FLIGHT entered  → schedule AP shutdown (deferred 200ms)
  //   STATE_FLIGHT exited   → bring AP back up immediately
  // All other state transitions leave the radio unchanged.
  // This keeps /wstatus reachable whenever the pilot has the glider
  // in hand, and turns the radio off only while the glider is airborne.
  if (ap_mode && window_active) {
    if (flight.state != prev_flight_state) {
      if (flight.state == STATE_FLIGHT) {
        // Glider just launched — schedule AP shutdown via existing mechanism
        if (!wifi_shutdown_pending && wifi_active) {
          wifi_shutdown_pending  = true;
          wifi_shutdown_after_ms = millis() + 200;
          logts(); Serial.printf("[PWR] Flight started — scheduling AP shutdown  clients=%d\n",
                        WiFi.softAPgetStationNum());
        }
      } else if (prev_flight_state == STATE_FLIGHT) {
        // Glider just landed/caught — bring AP back up if it went down
        if (!wifi_active) {
          char ap_ssid[24];
          snprintf(ap_ssid, sizeof(ap_ssid), "F3K-Unit-%02d", cfg.unit_id);
          IPAddress ap_ip(192, 168, cfg.unit_id, 1);
          logts(); Serial.println("[PWR] Landing — calling WiFi.mode(WIFI_AP)...");
          WiFi.mode(WIFI_AP);
          WiFi.softAPConfig(ap_ip, ap_ip, IPAddress(255, 255, 255, 0));
          logts(); Serial.printf("[PWR] Landing — calling softAP(%s)...\n", ap_ssid);
          bool ap_ok = WiFi.softAP(ap_ssid);
          logts(); Serial.printf("[PWR] Landing — softAP returned %s\n", ap_ok ? "OK" : "FAILED");
          delay(100);
          logts(); Serial.printf("[PWR] Landing — AP IP = %s\n", WiFi.softAPIP().toString().c_str());
          logts(); Serial.println("[PWR] Landing — calling server.begin()...");
          server.begin();
          wifi_active = true;
          logts(); Serial.printf("[PWR] Flight ended — AP back up: %s\n", ap_ssid);
        } else {
          logts(); Serial.println("[PWR] Landing — wifi_active already true, skipping AP restart");
        }
      }
      prev_flight_state = flight.state;
    }
  }

  // ── Deferred WiFi shutdown ────────────────────────────────
  // Fires 200ms after window open, giving the async TCP task (Core 0)
  // time to drain before WiFi.mode(OFF) tears down the interface.
  if (wifi_shutdown_pending && (long)(now - wifi_shutdown_after_ms) >= 0) {
    wifi_shutdown_pending = false;
    logts(); Serial.printf("[PWR] Deferred shutdown firing — window_active=%s  wifi_active=%s\n",
                  window_active ? "true" : "false", wifi_active ? "true" : "false");
    server.end();
    vTaskDelay(pdMS_TO_TICKS(50));  // yield to Core 0 TCP task
    if (ap_mode) {
      WiFi.softAPdisconnect(true);
    } else {
      WiFi.disconnect(true);
    }
    WiFi.mode(WIFI_OFF);
    wifi_active = false;
    logts(); Serial.println("[PWR] WiFi OFF — logging in progress");
  }

  // ── STA reconnection after window close ──────────────────
  // When WiFi was shut off during a window in STA mode, wifi_active
  // is false while reconnecting. Detect connection and re-enable.
  // NOTE: window_active is intentionally NOT checked here — a deferred
  // announcement for the previous window must be able to fire even if
  // a new window has already opened (back-to-back 0x21→0x20 scenario).
  if (!ap_mode && !wifi_active &&
      WiFi.status() == WL_CONNECTED) {
    wifi_active = true;
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    // Re-initialize UDP sockets — the network interface was torn down
    // during the window (WiFi.mode(WIFI_OFF)) so sockets must be re-bound.
    udp.begin(UDP_PORT);
    udp_gps.begin(4211);
    udp_win.begin(WiFi.broadcastIP(), UDP_WIN_PORT);
    if (!window_active) {
      server.begin();
    }
    logts(); Serial.printf("[PWR] WiFi ON — reconnected to %s  IP=%s\n",
                  WIFI_SSID, WiFi.localIP().toString().c_str());
  }

  // ── Log announcement — STA mode only ─────────────────────
  if (!ap_mode && wifi_active && announce_pending &&
      (last_announce_ms == 0 || now - last_announce_ms >= ANNOUNCE_INTERVAL_MS)) {
    last_announce_ms = now;
    sendAnnouncement();
    announce_count++;
    if (announce_count >= ANNOUNCE_REPEATS) announce_pending = false;
    did_work = true;
  }

  // ── Deferred filesystem cleanup ───────────────────────────────
  // Prune one file per loop pass (pruneLogsIfNeeded re-sets prune_pending
  // if more work remains, so we return here on the next pass).
  //
  // Allowed during:
  //   A) Idle ground state after window close (original behaviour)
  //   B) Prep countdown with >60s remaining — uses dead prep time productively.
  //      Stopped in the final 60s so filesystem work never overlaps the
  //      window-open critical path. The scorer re-syncs the timer on each
  //      0x21 packet so any brief blocking is automatically corrected.
  //   Rate-limited to one delete every 3s so the loop stays responsive.
  {
    static unsigned long last_prune_ms = 0;
    bool rate_ok = (now - last_prune_ms >= 3000);

    bool idle_ok = (!window_active && !announce_pending &&
                    littlefs_streaming == 0 &&
                    flight.state == STATE_GROUND);

    bool prep_ok = false;
    if (prep_active && !window_active && littlefs_streaming == 0) {
      long remaining_ms = (long)prep_fire_ms - (long)now;
      prep_ok = (remaining_ms > 60000);
    }

    if (prune_pending && rate_ok && (idle_ok || prep_ok)) {
      last_prune_ms = now;
      prune_pending = false;
      pruneLogsIfNeeded();
      did_work = true;
    }
  }

  // ── Diagnostic snapshot at 1 Hz — always runs ─────────────
  if (now - last_diag_ms >= 1000) {
    last_diag_ms = now;
    snapDiagnostics();
    did_work = true;
  }

  // ── UDP scorer + debug packets — STA mode only ────────────
  // Use full 5 Hz during active flight (LAUNCH_WIN, FLIGHT, LANDED).
  // Drop to 1 Hz on ground — scorer only needs a heartbeat.
  if (!ap_mode && wifi_active) {
    uint32_t udp_interval = (flight.state == STATE_LAUNCH_WIN ||
                             flight.state == STATE_FLIGHT     ||
                             flight.state == STATE_LANDED)
                            ? UDP_INTERVAL : UDP_INTERVAL_SLOW;
    if (now - last_udp_ms >= udp_interval) {
      last_udp_ms = now;
      sendUdpPacket();
      did_work = true;
    }
    if (now - last_dbg_ms >= udp_interval) {
      last_dbg_ms = now;
      sendDebugPacket();
      did_work = true;
    }
    // Packet 2 — GPS fix: 1 Hz, only when fix valid (ICD: omit if no fix)
    if (gps_present && gps_fix && (now - last_gps_tx_ms >= 1000)) {
      last_gps_tx_ms = now;
      sendGpsPacket();
      did_work = true;
    }
  }

  // ── Web display batch at 5 Hz (only when web overlay enabled)
  if (cfg.web_enabled && now - last_display_ms >= DISPLAY_INTERVAL) {
    last_display_ms = now;
    flushDisplayBatch();
    did_work = true;
  }

  // ── Loop timing ───────────────────────────────────────────
  unsigned long loop_dur = micros() - loop_start;
  loop_us_total += loop_dur;
  loop_count++;
  if (loop_dur > loop_max_us) loop_max_us = loop_dur;
  if (did_work) busy_us_total += loop_dur;
}
