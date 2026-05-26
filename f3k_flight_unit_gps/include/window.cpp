#include "window.h"

#include "globals.h"
#include "runtime.h"

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

void windowTimersInit() {
  _timer_open  = timerBegin(1000);
  timerAttachInterrupt(_timer_open,  &onWindowOpenTimer);

  _timer_close = timerBegin(1000);
  timerAttachInterrupt(_timer_close, &onWindowCloseTimer);

  logts(); Serial.println("[HW] Window timers ready (Timer_open  Timer_close  1ms tick)");
}
