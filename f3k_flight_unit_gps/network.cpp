#include "include/network.h"

#include <Arduino.h>
#include <WiFiUdp.h>
#include <LittleFS.h>

#include "conf.h"
#include "types.h"
#include "include/globals.h"
#include "include/sensors.h"
#include "include/logger.h"

// Functions still owned by other modules during staged refactor.
extern void logts();
extern void armWindowOpenTimer(uint32_t delay_ms);

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
