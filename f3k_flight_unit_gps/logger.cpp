#include "logger.h"
#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

#include "conf.h"
#include "types.h"
#include "globals.h"
#include "sensors.h"
#include "secrets.h"

#ifndef LITTLEFS_FULL_PCT
#define LITTLEFS_FULL_PCT 60
#endif
// Symbols still owned outside logger.cpp during this staged extraction.
extern void logts();
extern float calculateScore(float dur_s, float throw_height_ft);
extern void sendAnnouncement();

extern void armWindowCloseTimer(uint32_t delay_ms);
extern void disarmWindowOpenTimer();
extern void disarmWindowCloseTimer();

#ifndef MAX_SUMMARIES
#define MAX_SUMMARIES  16   // keep at most 16 summary files (≈ 2-day contest)
#endif

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
                    gps_hour, gps_minute, gps_second);
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
                  gps_hour, gps_minute, gps_second);
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
    FlightRecord rec;
    rec.flight_num      = (uint16_t)flight_counter;
    rec.duration_s      = dur;
    rec.throw_height_ft = flight.throw_height_ft;
    rec.peak_alt_ft     = flight.peak_alt_ft;
    rec.score           = score;
    rec.start_time_ms   = (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms));
    rec.end_time_ms     = window_close_ms - log_epoch_ms;

    flight_records[flight_record_count++] = rec;
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
                  gps_hour, gps_minute, gps_second);
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
