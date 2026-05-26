#include "state_machine.h"

#include <Arduino.h>
#include <math.h>

#include "conf.h"
#include "types.h"
#include "globals.h"
#include "sensors.h"
#include "logger.h"
#include "scoring.h"
#include "runtime.h"

// ============================================================
void updateStateMachine(float alt_ft) {
  // ── Thresholds — tuned from 3-session real flight data ───────
  // Data: gyro median during FLIGHT=34dps, G p5-p95=0.73-1.48G
  // Ground handling: G reaches 2.0-2.5G — IMU launch threshold raised above this
  // Soaring glider appears "at rest" on IMU — rest window duration raised to 3s
  // and gyro threshold raised above the flight median of 34dps
  const float    LANDED_ALT_FT      =  5.0f;  // ft — landing altitude threshold
  const float    LAND_REST_G_MIN    =  0.8f;  // G — rest lower bound
  const float    LAND_REST_G_MAX    =  1.2f;  // G — rest upper bound
  const float    LAND_GYRO_DPS      = 45.0f;  // °/s — raised: flight median gyro is 34dps
  const uint32_t LAND_REST_MS       = 3000;   // ms — raised: 3s steady rest is definitive

  static uint32_t launch_win_opened_ms  = 0;
  static uint32_t flight_started_ms     = 0;
  static uint32_t land_rest_start_ms    = 0;   // when rest condition first met
  static uint32_t impact_pending_ms     = 0;   // when impact G spike was detected
  static float    impact_pending_alt    = 0;   // altitude at impact spike
  static bool     was_high_this_flight  = false; // hit >10ft during this flight (catch guard)
  static bool     tilt_was_low          = false;  // tilt<12° confirmed during flight (tilt land guard)
  static uint8_t  tilt_low_consec       = 0;   // consecutive samples with tilt<12°
  static uint8_t  tilt_high_consec      = 0;   // consecutive samples with tilt>20°

  // ── Precompute IMU values used across cases ───────────────
  bool imu_ok        = imu_present && imu.valid;
  float gyro_max_dps = 0;
  if (imu_ok) {
    gyro_max_dps = max(fabsf(imu.gyro_x * 180.0f / PI),
                   max(fabsf(imu.gyro_y * 180.0f / PI),
                       fabsf(imu.gyro_z * 180.0f / PI)));
  }
  bool imu_launch_spike = imu_ok && (imu.g_force >= LAUNCH_G_THRESHOLD);
  bool imu_impact       = imu_ok && (imu.g_force >= LAND_IMPACT_G);
  bool imu_at_rest      = imu_ok &&
                          (imu.g_force >= LAND_REST_G_MIN) &&
                          (imu.g_force <= LAND_REST_G_MAX) &&
                          (gyro_max_dps < LAND_GYRO_DPS);

  switch (flight.state) {

    case STATE_CALIBRATING:
      if (calibration_done) {
        flight.state = STATE_GROUND;
        logts(); Serial.printf("[STATE] CALIBRATING → GROUND  baseline=%.2fft\n",
                      tare_baseline_ft);
      }
      break;

    case STATE_GROUND:
      flight.flight_duration_ms = 0;
      flight.throw_height_ft    = 0.0f;  // clear for next flight
      land_rest_start_ms        = 0;
      was_high_this_flight      = false;
      tilt_was_low              = false;
      tilt_low_consec           = 0;
      tilt_high_consec          = 0;
      {
        bool alt_trigger    = (alt_ft - tare_baseline_ft >= LAUNCH_ALT_FT);
        bool imu_trigger    = imu_launch_spike;
        // Altitude ceiling: a glider on the ground cannot be above 15ft.
        // If a false LANDED fires mid-air, this prevents immediately re-launching
        // at altitude when the state resets to GROUND.
        bool near_ground    = (alt_ft - tare_baseline_ft < 15.0f);
        if ((alt_trigger || imu_trigger) && near_ground) {
          flight.state            = STATE_LAUNCH_WIN;
          flight.launch_height_ft = alt_ft;
          flight.throw_height_ft  = alt_ft - last_landed_alt_ft;
          launch_win_opened_ms    = millis();
          flight_started_ms       = millis();
          flight_start_epoch_ms   = millis();   // window-relative start for summary CSV
          flight.peak_alt_ft      = alt_ft;
          logts(); Serial.printf("[STATE] GROUND → LAUNCH_WIN  alt=%.1fft  G=%.2f  trigger=%s\n",
                        alt_ft, imu_ok ? imu.g_force : 0.0f,
                        (alt_trigger && imu_trigger) ? "ALT+IMU" :
                        alt_trigger ? "ALT" : "IMU");
        }
      }
      break;

    case STATE_LAUNCH_WIN:
      flight.flight_duration_ms = millis() - flight_started_ms;
      if (alt_ft > flight.peak_alt_ft)      flight.peak_alt_ft      = alt_ft;
      if (alt_ft > flight.launch_height_ft) flight.launch_height_ft = alt_ft;
      // Update throw height every sample — peak gain above last landing
      flight.throw_height_ft = max(flight.throw_height_ft,
                                   alt_ft - last_landed_alt_ft);

      if (millis() - launch_win_opened_ms >= LAUNCH_WIN_MS) {
        flight.state = STATE_FLIGHT;
        // throw_height_ft is now frozen for the rest of this flight
        flight_counter++;
        logts(); Serial.printf("[STATE] LAUNCH_WIN → FLIGHT  lh=%.1fft  throw=%.1fft  t=%.1fs  flight#%d\n",
                      flight.launch_height_ft, flight.throw_height_ft,
                      flight.flight_duration_ms / 1000.0f, flight_counter);
        if (gps_fix) {
          logts(); Serial.printf("[TOD] Flight #%d launch: %02u:%02u:%02u UTC (GPS)\n",
                        flight_counter, gps_hour, gps_minute, gps_second);
        }

        // ── Auto-window (AP mode practice feature) ────────────
        // If no window is open and AUTO_WINDOW_SECS > 0, open one now.
        // Backdate window_start_ms by LAUNCH_WIN_MS so t_ms=0 in the CSV
        // aligns with the throw and the 5-second confirmation period is
        // credited as flight time. The window then expires naturally.
        if (ap_mode && !window_active && AUTO_WINDOW_SECS > 0) {
          window_secs     = AUTO_WINDOW_SECS;
          window_id       = millis();                    // unique local ID
          window_start_ms = millis() - LAUNCH_WIN_MS;   // backdate 5s
          window_active   = true;
          flight_record_count = 0;
          pilot_download_path = "";
          logts(); Serial.printf("[WIN] Auto-window opened: %ds  backdated 5s  id=%u\n",
                        window_secs, window_id);
          openWindowLog();
          // openWindowLog() resets flight_counter to 0, but flight #1 is
          // already confirmed and in progress — restore it so the landing
          // record shows flight_num=1, not flight_num=0.
          flight_counter = 1;
        }
      }
      // False trigger: altitude dropped AND no significant motion
      if ((alt_ft - tare_baseline_ft < LANDED_ALT_FT) && !imu_launch_spike) {
        flight.state              = STATE_GROUND;
        flight.flight_duration_ms = 0;
        flight.throw_height_ft    = 0.0f;
        logts(); Serial.println("[STATE] LAUNCH_WIN → GROUND (false trigger)");
      }
      break;

    case STATE_FLIGHT:
      flight.flight_duration_ms = millis() - flight_started_ms;
      if (alt_ft > flight.peak_alt_ft) flight.peak_alt_ft = alt_ft;

      // ── Track flight state flags for landing guards ────────
      // Catch guard: must have climbed above 10ft at some point
      if (alt_ft - tare_baseline_ft > 10.0f) was_high_this_flight = true;

      // Tilt guard: must have seen sustained low tilt (in-flight bias) before
      // tilt landing can fire. Prevents false trigger when unit never flew.
      if (imu_ok) {
        if (imu.tilt_deg < 12.0f) {
          tilt_low_consec = min((int)tilt_low_consec + 1, 255);
          if (tilt_low_consec >= 5) tilt_was_low = true;
        } else {
          tilt_low_consec = 0;
        }
        // Count consecutive high-tilt samples for tilt landing detector
        if (imu.tilt_deg > 20.0f) {
          tilt_high_consec = min((int)tilt_high_consec + 1, 255);
        } else {
          tilt_high_consec = 0;
        }
      }

      // ── Landing detection — whichever fires first ─────────

      // 0. Catch/grab — primary detector
      //    Guards: (a) must have climbed above 10ft — rules out barometric
      //    drift keeping alt negative throughout a low/failed flight
      //    (b) minimum flight time elapsed
      if (was_high_this_flight &&
          (alt_ft - tare_baseline_ft < -2.0f) &&
          (imu.g_force > 1.5f) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS)) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        // Re-zero baseline only if landing detected near ground level —
        // guards against re-zeroing when tilt fires at altitude or
        // large accumulated drift pushes reading well below baseline.
        // Suppressed in sim mode — sim controls altitude explicitly and
        // tare updates would corrupt the arc math.
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at catch: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (catch alt=%.1fft G=%.2f)  dur=%.1fs  peak=%.1fft\n",
                      alt_ft - tare_baseline_ft, imu.g_force,
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }

      // 1. Tilt landing detector
      //    In coordinated flight tilt stays 3-8°. On landing tilt rises to >20°
      //    as pilot holds glider at arbitrary angle. Require 1s sustained (8 samples).
      //    Guards: tilt must have been low during flight, min flight time elapsed,
      //    AND altitude must be near ground — aerobatics at altitude can also
      //    produce sustained high tilt (e.g. inverted, knife-edge maneuvers).
      if (tilt_was_low &&
          imu_ok &&
          (tilt_high_consec >= 8) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS) &&
          (alt_ft - tare_baseline_ft < 15.0f)) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at tilt: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (tilt %.1f° x%d samples)  dur=%.1fs  peak=%.1fft\n",
                      imu.tilt_deg, tilt_high_consec,
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }

      // 1. Hard impact spike near ground
      if (imu_impact && (alt_ft - tare_baseline_ft < 15.0f) &&
          flight.flight_duration_ms >= MIN_FLIGHT_MS) {
        impact_pending_ms  = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!tilt_mode && fabsf(alt_ft - tare_baseline_ft) <= 10.0f) {
          tare_baseline_ft = alt_ft;
          logts(); Serial.printf("[TARE] Re-zeroed at impact: %.3f ft abs\n", alt_ft);
        }
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (impact G=%.2f)  dur=%.1fs  peak=%.1fft\n",
                      imu.g_force, flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        break;
      }
      // 2. Altitude + gyro — alt below threshold AND low rotation (not just a low pass)
      // Pure altitude alone cannot distinguish low flying from landing.
      // Requiring gyro < 10 dps ensures the glider is actually stationary, not skimming.
      if ((alt_ft - tare_baseline_ft < LANDED_ALT_FT) &&
          (gyro_max_dps < 10.0f) &&
          (flight.flight_duration_ms >= MIN_FLIGHT_MS)) {
        impact_pending_ms = 0;
        if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
        if (!window_active) {
          // Window already closed and scored this flight at timeout — don't record again
        } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
          float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
          flight_records[flight_record_count++] = {
            (uint16_t)flight_counter,
            flight.flight_duration_ms / 1000.0f,
            flight.throw_height_ft,
            flight.peak_alt_ft,
            score,
            (unsigned long)max(0L, (long)(flight_start_epoch_ms - log_epoch_ms)),
            millis()              - log_epoch_ms
          };
          logts(); Serial.printf("[SCORE] Flight #%d recorded: dur=%.1fs throw=%.1fft score=%.1f (record %d/%d)\n",
                        flight_counter, flight.flight_duration_ms/1000.0f,
                        flight.throw_height_ft, score,
                        flight_record_count, MAX_FLIGHT_RECORDS);
        }
        flight.state = STATE_LANDED;
        logts(); Serial.printf("[STATE] FLIGHT → LANDED (alt)  dur=%.1fs  peak=%.1fft\n",
                      flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
        if (gps_fix) {
          logts(); Serial.printf("[TOD] Landing: %02u:%02u:%02u UTC (GPS)\n",
                        gps_hour, gps_minute, gps_second);
        }
        break;
      }
      // 3. Sustained rest (gentle landing / catch)
      // Disabled in tilt mode — stationary tilted unit falsely triggers
      // Also requires minimum flight time to avoid false triggers at launch
      if (!tilt_mode && flight.flight_duration_ms >= MIN_FLIGHT_MS) {
        if (imu_at_rest) {
          if (land_rest_start_ms == 0) land_rest_start_ms = millis();
          if (millis() - land_rest_start_ms >= LAND_REST_MS) {
          impact_pending_ms = 0;
            if (!tilt_mode) last_landed_alt_ft = alt_ft;  // sim: last_landed stays at 0
            if (!window_active) {
              // Window already closed and scored this flight at timeout — don't record again
            } else if (flight_record_count < MAX_FLIGHT_RECORDS) {
              float score = calculateScore(flight.flight_duration_ms/1000.0f, flight.throw_height_ft);
              flight_records[flight_record_count++] = {
                (uint16_t)flight_counter,
                flight.flight_duration_ms / 1000.0f,
                flight.throw_height_ft,
                flight.peak_alt_ft,
                score
              };
            }
            flight.state = STATE_LANDED;
            logts(); Serial.printf("[STATE] FLIGHT → LANDED (rest %.0fms)  dur=%.1fs  peak=%.1fft\n",
                          (float)(millis() - land_rest_start_ms),
                          flight.flight_duration_ms / 1000.0f, flight.peak_alt_ft);
          }
        } else {
          land_rest_start_ms = 0;
        }
      }
      break;

    case STATE_LANDED:
      // Auto-reset to GROUND quickly to support Quick Turn (QT) maneuver.
      // FAI: glider is "landed" the instant pilot touches it, "launched" the
      // instant pilot releases it. A good QT takes 1-3 seconds, so we must
      // be back in GROUND well within that window.
      // LANDED_RESET_MS is intentionally short — the scorer handles scoring
      // of the completed flight; the unit just needs to be ready for the next launch.
      {
        static uint32_t landed_ms = 0;
        const uint32_t  LANDED_RESET_MS = 500;  // ms — back to GROUND, ready for QT
        if (landed_ms == 0) landed_ms = millis();
        if (millis() - landed_ms >= LANDED_RESET_MS) {
          landed_ms                 = 0;
          flight.state              = STATE_GROUND;
          flight.peak_alt_ft        = 0;
          flight.flight_duration_ms = 0;
          flight.throw_height_ft    = 0.0f;
          land_rest_start_ms        = 0;
          impact_pending_ms         = 0;
          impact_pending_alt        = 0;
          was_high_this_flight      = false;
          tilt_was_low              = false;
          tilt_low_consec           = 0;
          tilt_high_consec          = 0;
          logts(); Serial.println("[STATE] LANDED → GROUND");
        }
      }
      break;
  }
}
