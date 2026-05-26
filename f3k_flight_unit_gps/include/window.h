#pragma once

#include <Arduino.h>

// ── Hardware timers — window timing ──────────────────────────
extern volatile unsigned long window_open_latch_ms;
extern volatile unsigned long window_close_latch_ms;

void windowTimersInit();

void armWindowOpenTimer(uint32_t delay_ms);
void armWindowCloseTimer(uint32_t delay_ms);
void disarmWindowOpenTimer();
void disarmWindowCloseTimer();
