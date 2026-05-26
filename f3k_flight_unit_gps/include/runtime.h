#pragma once

#include <Arduino.h>

// ── Log timestamp helper ──────────────────────────────────────
// Prints a timestamp prefix before every Serial log line.
// Format: [HH:MM:SS.mmm] from GPS UTC when fix valid,
//         [+NNNNNNNms]   from millis() always (shown alongside GPS).
// Usage: replace Serial.printf(...) with logts(); Serial.printf(...)
void logts();
