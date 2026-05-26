#include "store_config.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "conf.h"
#include "globals.h"

extern void logts();

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


