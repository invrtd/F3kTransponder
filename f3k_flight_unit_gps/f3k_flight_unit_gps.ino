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
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <ESPAsyncWebServer.h>
#include "esp_wifi.h"     // for esp_wifi_set_ps() modem sleep
#include "secrets.h"      // passwords don't belong here 
#include "conf.h"      
#include "html.h"      // embedded HTML for pilot UI and window status pages
#include "types.h"     // data structures for sensor state and flight state machine
#include "globals.h"
#include "sensors.h"    // sensor setup and reading functions
#include "webserver.h" // AsyncWebServer setup and endpoint handlers
#include "logger.h"
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
                  gps_hour, gps_minute, gps_second, gps_milliseconds, ms);
  } else {
    Serial.printf("[+%07lums] ", ms);
  }
}









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
      uint32_t gps_ms = ((uint32_t)gps_hour   * 3600000UL)
                      + ((uint32_t)gps_minute  *   60000UL)
                      + ((uint32_t)gps_second *    1000UL)
                      + ((uint32_t)gps_milliseconds);
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
  const float    LANDED_ALT_FT      =  5.0f;  // ft — landing altitude threshold
  const float    LAND_REST_G_MIN    =  0.8f;  // G — rest lower bound
  const float    LAND_REST_G_MAX    =  1.2f;  // G — rest upper bound
  const float    LAND_GYRO_DPS      = 45.0f;  // °/s — raised: flight median gyro is 34dps
  const uint32_t LAND_REST_MS       = 3000;   // ms — raised: 3s steady rest is definitive

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
                        flight_counter, gps_hour, gps_minute, gps_second);
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
                        gps_hour, gps_minute, gps_second);
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

  // -- Sensors --
  // DPS310 barometer
  sensors_init_barometer();

  // If no barometer, skip calibration and allow the unit to boot.
  // sensors.cpp owns the DPS object and dps_present flag.
  if (!dps_present) {
    calibration_done = true;
    tare_baseline_ft = 0.0f;
    flight.state     = STATE_GROUND;
  }

  // LSM6DSO32 IMU
  sensors_init_imu();

  // PA1010D GPS
  sensors_init_gps();

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
    delay(500);
    Serial.print(".");
  }
  logts(); Serial.println();
#endif

  if (ap_mode) {
    WiFi.mode(WIFI_AP);

    IPAddress ap_ip(192, 168, cfg.unit_id, 1);
    IPAddress ap_gateway(192, 168, cfg.unit_id, 1);
    IPAddress ap_subnet(255, 255, 255, 0);

    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid);   // open network, no password
    delay(500);

    logts(); Serial.printf("AP mode: SSID=%s  IP=%s\n", ap_ssid, WiFi.softAPIP().toString().c_str());
    logts(); Serial.printf("Browse to: http://%s/pilot\n", WiFi.softAPIP().toString().c_str());
    logts(); Serial.println("Scorer UDP: DISABLED (AP mode)");
  } else {
    logts(); Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    logts(); Serial.printf("UDP scorer → %s:%d\n", SERVER_IP, UDP_PORT);
    logts(); Serial.printf("UDP debug  → %s:%d\n", SERVER_IP, UDP_DBG_PORT);

    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    logts(); Serial.println("Modem sleep: ENABLED (WIFI_PS_MIN_MODEM)");

    udp.begin(UDP_PORT);
    udp_gps.begin(4211);
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

  if (ap_mode) cfg.web_enabled = true;



  setupWebServer();

  last_sensor_ms  = millis();
  last_imu_ms     = millis() + 20;
  last_diag_ms    = millis() + 500;
  last_udp_ms     = millis();
  last_dbg_ms     = millis() + DISPLAY_OFFSET_MS;
  last_display_ms = millis() + DISPLAY_OFFSET_MS;

  // ── Hardware timers ──────────────────────────────────────────
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
    sensors_read_imu();
    did_work = true;
  }

  // ── GPS — drain NMEA buffer and parse (non-blocking) ─────
  // Sensor module owns the PA1010D object and updates gps_* globals.
  if (gps_present && (now - last_gps_ms >= 100)) {
    last_gps_ms = now;
    sensors_read_gps();
    // Keep same work-accounting behavior as before: GPS polling is bounded
    // and only considered useful loop work when a fix is currently valid.
    if (gps_fix) did_work = true;
  }

  // ── Sensor read at 8 Hz ───────────────────────────────────
  if (now - last_sensor_ms >= SENSOR_INTERVAL) {
    last_sensor_ms = now;

    float alt_ft       = 0.0f;
    float pressure_hpa = SEA_LEVEL_HPA;
    float temp_c       = dps_temp_c;

    sensors_read_barometer(alt_ft, pressure_hpa, temp_c);

    if (!dps_present) {
      // No DPS310 — state machine still needs to run; feed zero altitude
      updateStateMachine(tare_baseline_ft);
      did_work = true;
    } else {

    if (pressure_hpa > 800 && pressure_hpa < 1100) {

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
          if (imu_present) {
            float t = imu_tilt_deg;
            if      (t < 15.0f) alt_ft = tare_baseline_ft + 0.0f;
            else if (t < 45.0f) alt_ft = tare_baseline_ft + 10.0f;
            else                alt_ft = tare_baseline_ft + t;
          }
        }

        flight.altitude_ft = alt_ft;
        dps_temp_c = temp_c;
        if (buf_count < BUF_SIZE) {
          buf_pressure[buf_count]    = pressure_hpa;
          buf_temperature[buf_count] = temp_c;
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
          dps_temp_c = temp_c;
          updateStateMachine(alt_ft);
        } else {
          flight.altitude_ft = alt_ft;
          dps_temp_c = temp_c;
          if (buf_count < BUF_SIZE) {
            buf_pressure[buf_count]    = pressure_hpa;
            buf_temperature[buf_count] = temp_c;
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
          logSample(alt_ft, pressure_hpa, temp_c);
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