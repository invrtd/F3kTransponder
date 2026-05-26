#include "runtime.h"

#include "sensors.h"

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
