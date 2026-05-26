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
#include "types.h"     // data structures for sensor state and flight state machine
#include "include/html.h"      // embedded HTML for pilot UI and window status pages
#include "include/globals.h"
#include "include/sensors.h"    // sensor setup and reading functions
#include "include/webserver.h" // AsyncWebServer setup and endpoint handlers
#include "include/logger.h"
#include "include/scoring.h"
#include "include/network.h"  // WiFi and UDP setup and helpers
#include "include/store_config.h"
#include "include/runtime.h"
#include "include/window.h"
#include "include/diagnostics.h"
#include "include/state_machine.h"
#include "include/display_batch.h"


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
  windowTimersInit();
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