#include "include/webserver.h"
#include "conf.h"
#include "types.h"
#include "include/globals.h"
#include "include/sensors.h"

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include "esp_wifi.h"

// ============================================================
//  External globals owned elsewhere
// ============================================================

// Runtime config / state

extern bool window_active;
extern bool window_countdown_active;


extern uint16_t window_secs;
extern uint32_t window_id;
extern unsigned long window_start_ms;

// HTML pages.
// IMPORTANT: these must have external linkage.
// See note below this file.
extern const char PILOT_HTML[] PROGMEM;
extern const char WSTATUS_HTML[] PROGMEM;
extern const char PAGE_HTML[] PROGMEM;

// Functions owned elsewhere
extern void logts();
extern void closeWindowLog();

// ============================================================
//  Helpers
// ============================================================

static String currentIpString() {
  return ap_mode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

static bool isAllowedLogName(const String& name) {
  return (name.endsWith(".csv") &&
          (name.startsWith("window_") || name.startsWith("summary_")));
}

static String buildLogsPage() {
  String ip = currentIpString();

  String html;
  html.reserve(9000);

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += "<title>F3K Unit " + String(flight.unit_id) + " Logs</title>";
  html += F(
    "<style>"
    "body{font-family:monospace;background:#1a1a1a;color:#e0e0e0;padding:20px;}"
    "h2,h3{color:#4db8ff;}"
    "table{border-collapse:collapse;width:100%;max-width:760px;margin-bottom:1.2rem;}"
    "th{background:#2a2a2a;color:#4db8ff;padding:8px 12px;text-align:left;border-bottom:2px solid #4db8ff;}"
    "td{padding:7px 12px;border-bottom:1px solid #333;}"
    "tr:hover td{background:#2a2a2a;}"
    "a{color:#66ffaa;text-decoration:none;}a:hover{text-decoration:underline;}"
    ".empty{color:#ff6666;}.size{color:#ffcc44;}"
    ".danger{display:inline-block;padding:8px 16px;background:#3a1a1a;color:#ff6666;"
    "border:1px solid #ff6666;border-radius:4px;text-decoration:none;font-weight:bold;}"
    ".muted{color:#666;font-size:0.85em;}"
    "</style></head><body>"
  );

  html += "<h2>F3K Unit " + String(flight.unit_id) + " - Flight Logs</h2>";

  struct LogEntry {
    String name;
    size_t size;
  };

  LogEntry entries[50];
  int count = 0;

  if (LittleFS.begin(false)) {
    File dir = LittleFS.open(LOG_DIR);
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
  } else {
    html += F("<p style='color:#ff6666'>LittleFS unavailable.</p>");
  }

  if (count == 0) {
    html += F("<p>No log files found.</p>");
  } else {
    html += F("<h3>Sensor Logs</h3>");
    html += F("<table><tr><th>#</th><th>File</th><th>Size</th><th>Download</th><th>Delete</th></tr>");

    int row = 0;
    for (int i = 0; i < count; i++) {
      String name = entries[i].name;
      if (!name.startsWith("window_")) continue;

      String num = name.substring(7, 10);
      String url = "http://" + ip + "/log?n=" + num;

      String sizeStr = entries[i].size == 0
        ? String(F("<span class='empty'>0 bytes empty</span>"))
        : String(F("<span class='size'>")) + String(entries[i].size) + F(" bytes</span>");

      html += "<tr><td>" + String(++row) + "</td>";
      html += "<td>" + htmlEscape(name) + "</td>";
      html += "<td>" + sizeStr + "</td>";
      html += "<td><a href='" + url + "' download='unit" +
              String(flight.unit_id) + "_" + htmlEscape(name) + "'>download</a></td>";
      html += "<td><a href='#' onclick=\"delFile('" + htmlEscape(name) + "',this)\" "
              "style='color:#ff6666;'>delete</a></td></tr>";
    }

    if (row == 0) {
      html += F("<tr><td colspan='5'>No sensor logs found.</td></tr>");
    }

    html += F("</table>");

    html += F("<h3 style='color:#66ffaa;'>Score Summaries</h3>");
    html += F("<table><tr><th>#</th><th>File</th><th>Size</th><th>Download</th><th>Delete</th></tr>");

    int srow = 0;
    for (int i = 0; i < count; i++) {
      String name = entries[i].name;
      if (!name.startsWith("summary_")) continue;

      String num = name.substring(8, 11);
      String url = "http://" + ip + "/summary?n=" + num;

      String sizeStr = entries[i].size == 0
        ? String(F("<span class='empty'>0 bytes empty</span>"))
        : String(F("<span class='size'>")) + String(entries[i].size) + F(" bytes</span>");

      html += "<tr><td>" + String(++srow) + "</td>";
      html += "<td>" + htmlEscape(name) + "</td>";
      html += "<td>" + sizeStr + "</td>";
      html += "<td><a href='" + url + "' download='unit" +
              String(flight.unit_id) + "_" + htmlEscape(name) + "'>download</a></td>";
      html += "<td><a href='#' onclick=\"delFile('" + htmlEscape(name) + "',this)\" "
              "style='color:#ff6666;'>delete</a></td></tr>";
    }

    if (srow == 0) {
      html += F("<tr><td colspan='5'>No summaries found.</td></tr>");
    }

    html += F("</table>");

    html += F(
      "<p class='muted'>Tap delete once, then again to confirm.</p>"
      "<p><a href='#' onclick='wipeAll(this);return false;' class='danger'>Delete All Logs</a></p>"
    );
  }

  html += "<p class='muted'>Unit " + String(flight.unit_id) +
          " | Window " + String(cfg.window_number) +
          " | Free heap: " + String(ESP.getFreeHeap()) + " bytes</p>";

  html += F(
    "<script>"
    "function delFile(name,el){"
    " if(el.dataset.confirm!='1'){"
    "   el.dataset.confirm='1';"
    "   el.textContent='Sure?';"
    "   el.style.color='#ffaa00';"
    "   setTimeout(()=>{if(el.dataset.confirm==='1'){"
    "     el.dataset.confirm='0';el.textContent='delete';el.style.color='#ff6666';"
    "   }},3000);"
    "   return;"
    " }"
    " fetch('/delete?f='+encodeURIComponent(name)).then(r=>{"
    "   if(r.status===503){"
    "     el.dataset.confirm='0';el.textContent='Busy';el.style.color='#ffaa00';"
    "     setTimeout(()=>{el.textContent='delete';el.style.color='#ff6666';},2000);"
    "     return;"
    "   }"
    "   if(r.ok){"
    "     el.closest('tr').style.opacity='0.3';"
    "     el.textContent='deleted';el.style.color='#3fb950';"
    "   }else{el.textContent='ERR';}"
    " }).catch(()=>{el.textContent='ERR';});"
    "}"
    "function wipeAll(el){"
    " var s=el.dataset.stage||'0';"
    " if(s==='0'){"
    "   el.dataset.stage='1';el.textContent='Are you sure?';el.style.background='#5a2a2a';"
    "   setTimeout(()=>{if(el.dataset.stage==='1'){"
    "     el.dataset.stage='0';el.textContent='Delete All Logs';el.style.background='#3a1a1a';"
    "   }},3000);return;"
    " }"
    " if(s==='1'){"
    "   el.dataset.stage='2';el.textContent='Really wipe ALL?';el.style.background='#7a2a2a';"
    "   setTimeout(()=>{if(el.dataset.stage==='2'){"
    "     el.dataset.stage='0';el.textContent='Delete All Logs';el.style.background='#3a1a1a';"
    "   }},3000);return;"
    " }"
    " fetch('/wipe-logs?confirm=YES&extra=SURE')"
    " .then(r=>r.text()).then(t=>{"
    "   el.textContent=t;el.style.background='#1a3a1a';el.style.color='#3fb950';"
    "   setTimeout(()=>location.reload(),1500);"
    " }).catch(()=>{el.textContent='ERR';el.style.color='#ff6666';});"
    "}"
    "</script></body></html>"
  );

  return html;
}

static void sendCsvFile(AsyncWebServerRequest* req,
                        const char* path,
                        const char* downloadName,
                        bool deleteAfter,
                        bool isWindowLog) {
  if (!LittleFS.begin(false)) {
    req->send(503, "text/plain", "LittleFS unavailable");
    return;
  }

  if (!LittleFS.exists(path)) {
    LittleFS.end();
    req->send(404, "text/plain", "File not found");
    return;
  }

  AsyncWebServerResponse* resp = req->beginResponse(LittleFS, path, "text/csv");
  String cd = "attachment; filename=\"";
  cd += downloadName;
  cd += "\"";
  resp->addHeader("Content-Disposition", cd);
  resp->addHeader("Connection", "close");

  littlefs_streaming++;
  if (littlefs_streaming == 1) {
    esp_wifi_set_ps(WIFI_PS_NONE);
  }

  if (deleteAfter && isWindowLog) {
    String pathCopy(path);
    req->onDisconnect([pathCopy]() {
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
    req->onDisconnect([]() {
      if (--littlefs_streaming <= 0) {
        littlefs_streaming = 0;
        LittleFS.end();
        if (!window_active) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
      }
    });
  }

  req->send(resp);
}

// ============================================================
//  Route registration
// ============================================================

void setupWebServer() {
  // ------------------------------------------------------------
  // Logs browser
  // ------------------------------------------------------------
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", buildLogsPage());
  });

  // ------------------------------------------------------------
  // Delete one log file
  // ------------------------------------------------------------
  server.on("/delete", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("f")) {
      req->send(400, "text/plain", "Missing parameter: f");
      return;
    }

    if (littlefs_streaming > 0) {
      req->send(503, "text/plain", "File transfer in progress - retry shortly");
      return;
    }

    String name = req->getParam("f")->value();

    if (!isAllowedLogName(name)) {
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
      removed = LittleFS.remove(path);
      if (removed) {
        logts(); Serial.printf("[LOG] Deleted via UI: %s\n", path.c_str());
      }
    }

    if (littlefs_streaming <= 0) {
      LittleFS.end();
    }

    req->send(removed ? 200 : 404, "text/plain", removed ? "Deleted" : "Not found");
  });

  // ------------------------------------------------------------
  // Wipe all logs
  // ------------------------------------------------------------
  server.on("/wipe-logs", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("confirm") || req->getParam("confirm")->value() != "YES" ||
        !req->hasParam("extra")   || req->getParam("extra")->value()   != "SURE") {
      req->send(400, "text/plain", "Missing safety params");
      return;
    }

    if (littlefs_streaming > 0) {
      req->send(503, "text/plain", "File transfer in progress - retry shortly");
      return;
    }

    if (window_active) {
      req->send(503, "text/plain", "Window active - wipe refused");
      return;
    }

    if (!LittleFS.begin(false)) {
      req->send(503, "text/plain", "LittleFS unavailable");
      return;
    }

    const int MAX_FILES = 256;
    String* victims = new (std::nothrow) String[MAX_FILES];

    if (!victims) {
      LittleFS.end();
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
        if (isAllowedLogName(name)) {
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
      if (LittleFS.remove(path)) {
        deleted++;
      }
    }

    delete[] victims;
    LittleFS.end();

    logts(); Serial.printf("[LOG] WIPE: deleted %u files via UI%s\n",
                           deleted, overflowed ? " (more remain)" : "");

    String msg = "Deleted " + String(deleted) + " files";
    if (overflowed) msg += " (more remain - click again)";
    req->send(200, "text/plain", msg);
  });

  // ------------------------------------------------------------
  // Static HTML pages
  // ------------------------------------------------------------
  server.on("/pilot", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PILOT_HTML);
  });

  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", PAGE_HTML);
  });

  // ------------------------------------------------------------
  // Active-window status page
  // ------------------------------------------------------------
  server.on("/wstatus", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!window_active) {
      req->redirect("/pilot");
      return;
    }

    uint32_t elapsed_s = (millis() - window_start_ms) / 1000;

    String js;
    js.reserve(1200);

    js = "<script>window.__WS__={";
    js += "unit_id:"      + String(cfg.unit_id)                    + ",";
    js += "elapsed_s:"    + String(elapsed_s)                      + ",";
    js += "win_secs:"     + String(window_secs)                    + ",";
    js += "log_open:"     + String(log_open ? "true" : "false")    + ",";
    js += "log_name:\""   + String(log_open ? log_path : "")       + "\",";
    js += "log_bytes:"    + String(log_open ? log_file.size() : 0) + ",";
    js += "flight_count:" + String(flight_record_count)            + ",";
    js += "gps_present:"  + String(gps_present ? "true" : "false") + ",";
    js += "gps_fix:"      + String(gps_fix ? "true" : "false")     + ",";
    js += "gps_sats:"     + String(gps_sats)                       + ",";
    js += "flights:[";

    for (int i = 0; i < flight_record_count; i++) {
      if (i > 0) js += ",";
      js += "{num:"      + String(flight_records[i].flight_num)          + ",";
      js += "dur:"       + String(flight_records[i].duration_s, 1)       + ",";
      js += "throw_ft:"  + String(flight_records[i].throw_height_ft, 1)  + ",";
      js += "peak_ft:"   + String(flight_records[i].peak_alt_ft, 1)      + ",";
      js += "score:"     + String(flight_records[i].score, 1)            + "}";
    }

    js += "]}</script>";

    String page = String(FPSTR(WSTATUS_HTML));
    page.replace("<script>", js + "<script>");
    req->send(200, "text/html", page);
  });

  // ------------------------------------------------------------
  // Runtime mode endpoints
  // ------------------------------------------------------------
  server.on("/setscore", HTTP_GET, [](AsyncWebServerRequest* req) {
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

  server.on("/settilt", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("v")) {
      uint8_t newSim = (uint8_t)constrain(req->getParam("v")->value().toInt(), 0, 2);

      if (newSim != sim_mode) {
        sim_mode  = newSim;
        tilt_mode = (sim_mode > 0);

        calibration_done   = false;
        cal_start_ms       = 0;
        cal_count          = 0;
        last_landed_alt_ft = 0.0f;

        if (!tilt_mode) {
          flight.state = STATE_CALIBRATING;
        }

        const char* modeNames[] = {"Normal", "Tilt Sim", "Parabola Sim"};
        logts(); Serial.printf("[MODE] Input mode: %s\n", modeNames[sim_mode]);
      }
    }

    req->send(200, "application/json",
              String("{\"sim_mode\":") + sim_mode +
              ",\"tilt_mode\":" + (tilt_mode ? "true" : "false") + "}");
  });

  // ------------------------------------------------------------
  // Pilot polling endpoint
  // ------------------------------------------------------------
  server.on("/pstatus", HTTP_GET, [](AsyncWebServerRequest* req) {
    float total_score = 0.0f;

    if (flight_record_count > 0) {
      for (int i = 0; i < flight_record_count; i++) {
        total_score += flight_records[i].score;
      }
      if (score_mode == 1) {
        total_score /= flight_record_count;
      }
    }

    String j;
    j.reserve(1800);

    j = "{";
    j += "\"unit_id\":"       + String(flight.unit_id) + ",";
    j += "\"state\":"         + String((int)flight.state) + ",";
    j += "\"state_name\":\""  + String(stateNames[flight.state]) + "\",";
    j += "\"flight_num\":"    + String(flight_counter) + ",";
    j += "\"flight_t\":"      + String(flight.flight_duration_ms / 1000.0f, 1) + ",";
    j += "\"throw_ht\":"      + String(flight.throw_height_ft, 1) + ",";
    j += "\"alt_tared\":"     + String(flight.altitude_ft - tare_baseline_ft, 1) + ",";
    j += "\"win_active\":"    + String(window_active ? "true" : "false") + ",";
    j += "\"win_secs\":"      + String(window_secs) + ",";
    j += "\"prep_active\":"   + String(prep_active ? "true" : "false") + ",";
    j += "\"prep_remain\":"   + String(prep_active ? max(0L, (long)(prep_fire_ms - millis()) / 1000 + 1) : 0) + ",";
    j += "\"countdown\":"     + String(window_countdown_active ? "true" : "false") + ",";
    j += "\"countdown_remain\":";

    if (window_countdown_active) {
      uint32_t elapsed = millis() - window_countdown_start;
      uint32_t remain = elapsed < WINDOW_COUNTDOWN_MS
        ? (WINDOW_COUNTDOWN_MS - elapsed) / 1000 + 1
        : 0;
      j += String(remain);
    } else {
      j += "0";
    }

    j += ",";
    j += "\"log_ready\":"  + String((!window_active && pilot_download_path.length() > 0) ? "true" : "false") + ",";
    j += "\"log_num\":"    + String(cfg.window_number) + ",";
    j += "\"tilt_mode\":"  + String(tilt_mode ? "true" : "false") + ",";
    j += "\"sim_mode\":"   + String(sim_mode) + ",";
    j += "\"score_mode\":" + String(score_mode) + ",";
    j += "\"wifi_active\":" + String((wifi_active && !wifi_shutdown_pending) ? "true" : "false") + ",";
    j += "\"total_score\":" + String(total_score, 1) + ",";
    j += "\"flights\":[";

    for (int i = 0; i < flight_record_count; i++) {
      if (i > 0) j += ",";
      j += "{\"num\":"   + String(flight_records[i].flight_num) + ",";
      j += "\"dur\":"    + String(flight_records[i].duration_s, 1) + ",";
      j += "\"throw\":"  + String(flight_records[i].throw_height_ft, 1) + ",";
      j += "\"peak\":"   + String(flight_records[i].peak_alt_ft, 1) + ",";
      j += "\"score\":"  + String(flight_records[i].score, 1) + "}";
    }

    j += "]}";

    req->send(200, "application/json", j);
  });

  // ------------------------------------------------------------
  // GPS polling endpoint
  // ------------------------------------------------------------
  server.on("/pgps", HTTP_GET, [](AsyncWebServerRequest* req) {
    String j;
    j.reserve(300);

    j = "{";
    j += "\"present\":"     + String(gps_present ? "true" : "false") + ",";
    j += "\"fix\":"         + String(gps_fix ? "true" : "false") + ",";
    j += "\"fix_quality\":" + String(gps_fix_quality) + ",";
    j += "\"satellites\":"  + String(gps_sats) + ",";
    j += "\"hdop\":"        + String(gps_hdop, 1) + ",";
    j += "\"lat\":"         + String(gps_lat, 6) + ",";
    j += "\"lon\":"         + String(gps_lon, 6) + ",";
    j += "\"alt_m\":"       + String(gps_alt_m, 1);
    j += "}";

    req->send(200, "application/json", j);
  });

  // ------------------------------------------------------------
  // Pilot window controls
  // ------------------------------------------------------------
  server.on("/pstart", HTTP_GET, [](AsyncWebServerRequest* req) {
    uint16_t secs = 300;

    if (req->hasParam("secs")) {
      secs = req->getParam("secs")->value().toInt();
    }

    secs = constrain(secs, 30, 3600);

    if (window_active) {
      closeWindowLog();
    }

    flight_record_count     = 0;
    pilot_download_path     = "";
    window_countdown_secs   = secs;
    window_countdown_start  = millis();
    window_countdown_active = true;

    logts(); Serial.printf("[WIN] Countdown started: %ds window in %ds\n",
                           secs, WINDOW_COUNTDOWN_MS / 1000);

    req->send(200, "text/plain", "OK");
  });

  server.on("/pstop", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (window_countdown_active) {
      window_countdown_active = false;
      logts(); Serial.println("[WIN] Countdown cancelled");
    } else if (window_active) {
      logts(); Serial.println("[WIN] Pilot stopped window early");
      closeWindowLog();
    }

    req->send(200, "text/plain", "OK");
  });

  // ------------------------------------------------------------
  // Summary download
  // ------------------------------------------------------------
  server.on("/summary", HTTP_GET, [](AsyncWebServerRequest* req) {
    int num = req->hasParam("n")
      ? req->getParam("n")->value().toInt()
      : cfg.window_number;

    char spath[40];
    snprintf(spath, sizeof(spath), "%s/summary_%03d.csv", LOG_DIR, num);

    char filename[80];
    snprintf(filename, sizeof(filename), "unit%02d_summary_%03d.csv",
             flight.unit_id, num);

    sendCsvFile(req, spath, filename, false, false);

    logts(); Serial.printf("[SUMMARY] Served %s\n", spath);
  });

  // ------------------------------------------------------------
  // Window log download
  // ------------------------------------------------------------
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("n")) {
      req->send(400, "text/plain", "Missing parameter: n");
      return;
    }

    if (log_open) {
      req->send(503, "text/plain", "Log still open - retry shortly");
      return;
    }

    String num = req->getParam("n")->value();
    while (num.length() < 3) num = "0" + num;

    bool del_after = req->hasParam("del") && req->getParam("del")->value() == "1";

    String path = String(LOG_DIR) + "/window_" + num + ".csv";
    String filename = "unit" + String(flight.unit_id) + "_window_" + num + ".csv";

    logts(); Serial.printf("[LOG] Serving %s -> %s%s\n",
                           path.c_str(),
                           req->client()->remoteIP().toString().c_str(),
                           del_after ? " (delete after)" : "");

    sendCsvFile(req, path.c_str(), filename.c_str(), del_after, true);
  });

  // ------------------------------------------------------------
  // Main telemetry overlay
  // ------------------------------------------------------------
  if (cfg.web_enabled) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
      req->send_P(200, "text/html", PAGE_HTML);
    });

    server.on("/json", HTTP_GET, [](AsyncWebServerRequest* req) {
      const String& json = (active_buf == 0) ? batch_json_a : batch_json_b;
      req->send(200, "application/json", json);
    });

    server.on("/tare", HTTP_GET, [](AsyncWebServerRequest* req) {
      tare_requested = true;
      req->send(200, "text/plain", "OK");
    });
  }

  // ------------------------------------------------------------
  // 404
  // ------------------------------------------------------------
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();

  if (cfg.web_enabled) {
    if (ap_mode) {
      logts(); Serial.printf("Web (AP): http://%s/\n", WiFi.softAPIP().toString().c_str());
    } else {
      logts(); Serial.printf("Web overlay: http://%s/\n", WiFi.localIP().toString().c_str());
    }
  } else {
    if (ap_mode) {
      logts(); Serial.printf("Log server (AP): http://%s/logs\n",
                             WiFi.softAPIP().toString().c_str());
    } else {
      logts(); Serial.printf("Log server: http://%s/logs  (log retrieval: /log?n=NNN)\n",
                             WiFi.localIP().toString().c_str());
      logts(); Serial.println("Web overlay: DISABLED (field mode)");
    }
  }
}