#include "include/diagnostics.h"

#include <WiFi.h>
#include "include/globals.h"
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
