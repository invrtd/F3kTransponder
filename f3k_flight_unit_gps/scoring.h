#pragma once

#include <Arduino.h>

// Current scoring mode:
//   0 = Secs - Ft
//   1 = JoeD V1
extern uint8_t score_mode;

float calculateScore(float dur_s, float throw_height_ft);