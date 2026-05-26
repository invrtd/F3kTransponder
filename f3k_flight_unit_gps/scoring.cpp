#include "scoring.h"

#include <Arduino.h>
#include "globals.h"

// ============================================================
//  calculateScore — unified scoring with selectable formula
//
//  Mode 0 — Secs-Ft:
//    duration_s - throw_height_ft  (window-independent, no normalization)
//    Window score = sum of all flight scores
//
//  Mode 1 — JoeD V1:
//    time_score = (duration_s / 180)^0.425 * 1000   ← t_max ALWAYS 180s
//    h <= 100ft: score = time_score + (100-h)^1.6 * 0.113  (cap 1100)
//    h >  100ft: score = time_score - (h-100)^2.3 * 0.09   (floor 0)
//    Window score = AVERAGE of all flight scores (including airborne-at-close)
// ============================================================
float calculateScore(float dur_s, float throw_height_ft) {
  if (score_mode == 1) {
    // JoeD V1 — t_max is always 3 minutes (180s), independent of window length
    const float JOED_TMAX = 180.0f;
    float time_ratio = constrain(dur_s / JOED_TMAX, 0.0f, 1.0f);
    float score_t    = powf(time_ratio, 0.425f) * 1000.0f;
    float h          = throw_height_ft;
    float final_score;
    if (h <= 100.0f) {
      float bonus = powf(100.0f - h, 1.6f) * 0.113f;
      final_score = score_t + bonus;
    } else {
      float penalty = powf(h - 100.0f, 2.3f) * 0.09f;
      final_score = score_t - penalty;
    }
    return constrain(final_score, 0.0f, 1100.0f);
  }

  // Mode 0 — Secs-Ft: duration_s - throw_height_ft (window-independent)
  if (window_secs == 0) return 0.0f;
  return dur_s - throw_height_ft;
}
