#include "include/flight_sim.h"

#include <Arduino.h>

#include "conf.h"
#include "include/globals.h"
#include "include/sensors.h"
#include "include/runtime.h"

// ── Static state ────────────────────────────────────────────
static unsigned long sim_phase_ms    = 0;
static bool          sim_in_flight   = false;
static uint8_t       sim_flight_idx  = 0;
static uint32_t      sim_flight_ms   = 0;
static float         sim_launch_ft   = 0.0f;
static float         sim_peak_ft     = 0.0f;
static uint8_t       last_sim_mode2  = 255;
static uint8_t       last_task_id    = 255;
static bool          last_win_active = false;  // detect window open edge
static uint8_t       sim_task_snap   = 0;      // task_id snapshotted at window open

static const uint32_t SIM_GROUND_MS = 5000UL;  // 5s between flights

void flightSimReset() {
  sim_phase_ms    = millis();
  sim_in_flight   = false;
  sim_flight_idx  = 0;
  sim_flight_ms   = 0;
  sim_launch_ft   = 0.0f;
  sim_peak_ft     = 0.0f;
  last_sim_mode2  = 255;
  last_task_id    = 255;
  last_win_active = false;
  sim_task_snap   = 0;
}

// ── Task flight plan lookup ──────────────────────────────────
// Returns list of target durations (seconds) for the active task.
// Stored in flash via PROGMEM-style const arrays.
// sim_flight_idx cycles through the list; wraps for unlimited tasks.
static uint16_t getTaskTarget(uint8_t idx) {
  // Uses sim_task_snap — the task_id snapshotted at window open,
  // not the live contest_task_id which may update mid-window.
  const uint16_t tLL[]  = {60,60,60,60,60};
  const uint16_t tA[]   = {300};
  const uint16_t tB[]   = {240,240};
  const uint16_t tC3[]  = {180,180,180};
  const uint16_t tC4[]  = {180,180,180,180};
  const uint16_t tC5[]  = {180,180,180,180,180};
  const uint16_t tD[]   = {30,45,60,75,90,105,120};
  const uint16_t tE[]   = {200,200,200,200,200};
  const uint16_t tF[]   = {180,180,180,180,180,180};
  const uint16_t tG[]   = {120,120,120,120,120,120,120};
  const uint16_t tH[]   = {60,120,180,240};
  const uint16_t tI[]   = {200,200,200,200,200};
  const uint16_t tJ[]   = {180,180,180};
  const uint16_t tK[]   = {60,90,120,150,180,210,240};
  const uint16_t tD2[]  = {300,300};
  const uint16_t tE2[]  = {200,200,200};
  const uint16_t tL[]   = {599};
  const uint16_t tM[]   = {180,300,420};
  const uint16_t tN[]   = {599};
  const uint16_t tL2[]  = {419};
  const uint16_t tB2[]  = {180,180};
  const uint16_t tA2[]  = {300};

  const uint16_t* list  = tLL;
  uint8_t         count = 5;

  switch (sim_task_snap) {
    case  6: list=tA;   count=1; break;
    case  7: list=tB;   count=2; break;
    case  8: list=tC3;  count=3; break;
    case  9: list=tD;   count=7; break;
    case 10: list=tE;   count=5; break;
    case 11: list=tF;   count=6; break;
    case 12: list=tG;   count=7; break;
    case 13: list=tH;   count=4; break;
    case 14: list=tI;   count=5; break;
    case 15: list=tJ;   count=3; break;
    case 17: list=tC4;  count=4; break;
    case 18: list=tC5;  count=5; break;
    case 19: list=tA2;  count=1; break;
    case 20: list=tB2;  count=2; break;
    case 21: list=tK;   count=7; break;
    case 26: list=tD2;  count=2; break;
    case 27: list=tE2;  count=3; break;
    case 28: list=tE2;  count=3; break;
    case 29: list=tL;   count=1; break;
    case 30: list=tM;   count=3; break;
    case 33: list=tN;   count=1; break;
    case 34: list=tL2;  count=1; break;
    default: list=tLL;  count=5; break;
  }
  return list[idx % count];
}

// ── Seed a new flight from task plan ────────────────────────
static void seedNewFlight() {
  uint16_t target_s = getTaskTarget(sim_flight_idx);
  // +/- 10s jitter using unit_id + flight_idx as deterministic seed
  // (no rand() to avoid seeding complexity; pseudo-random enough for sim)
  int8_t jitter = (int8_t)(((cfg.unit_id * 7 + sim_flight_idx * 13) % 21) - 10);
  uint16_t dur_s = (uint16_t)constrain((int)target_s + jitter, 5, 599);
  sim_flight_ms  = (uint32_t)dur_s * 1000UL;

  // Launch height: 30–60ft, varied by unit_id + flight index
  sim_launch_ft  = 30.0f + (float)((cfg.unit_id * 11 + sim_flight_idx * 17) % 31);
  // Peak: launch height + 20–40ft additional climb
  sim_peak_ft    = sim_launch_ft + 20.0f + (float)((cfg.unit_id * 5 + sim_flight_idx * 7) % 21);
  sim_flight_idx++;
  logts(); Serial.printf("[SIM] Flight %u: task=%u target=%us jitter=%+ds actual=%us launch=%.0fft peak=%.0fft tare=%.1fft last_landed=%.1fft\n",
                sim_flight_idx, sim_task_snap, target_s, jitter, dur_s, sim_launch_ft, sim_peak_ft,
                tare_baseline_ft, last_landed_alt_ft);
}

static void updateTaskAwareSimulation(float& alt_ft, unsigned long now) {
  // ── Task-aware flight simulation — fully autonomous ─────────
  // Generates realistic flight durations from the current contest_task_id.
  // Each flight uses a target drawn from the task plan +/- 10s random jitter.
  // Altitude profile: fast launch climb → soar near peak → gradual descent.
  // Only cycles during an active window; sits at GROUND outside a window.

  // ── Detect mode change ──────────────────────────────────────
  if (last_sim_mode2 != 2) {
    sim_phase_ms    = now;
    sim_in_flight   = false;
    sim_flight_idx  = 0;
    last_sim_mode2  = 2;
    last_task_id    = 255;
    last_win_active = false;
    logts(); Serial.println("[SIM] Task sim activated");
  }

  if (!window_active) {
    // Outside window — hold at ground, reset for clean start on next window
    alt_ft          = tare_baseline_ft;
    sim_in_flight   = false;
    sim_flight_idx  = 0;
    sim_phase_ms    = now;
    last_win_active = false;
    return;
  }

  // ── Window-open edge and task-change detection ──────────────
  // Snapshot contest_task_id at the moment the window opens or
  // when the 0x20 grace period updates it. This ensures the sim
  // uses the authoritative task even when the scorer's 0x21 prep
  // packets hadn't yet been updated to send ICD v1.7 task bytes.
  bool win_edge     = (window_active && !last_win_active);
  bool task_changed = (contest_task_id != last_task_id);
  last_win_active   = true;

  if (win_edge || task_changed) {
    sim_task_snap  = contest_task_id;
    last_task_id   = contest_task_id;
    sim_flight_idx = 0;
    sim_in_flight  = false;
    sim_phase_ms   = now;
    logts(); Serial.printf("[SIM] Task snapshot: task_id=%u (%s)  R%u G%u\n",
                  sim_task_snap, taskName(sim_task_snap),
                  contest_round_num, contest_group_num);
  }

  unsigned long phase_ms = now - sim_phase_ms;

  if (!sim_in_flight) {
    // Ground pause phase — hold at ground until throw
    // Last 500ms of the pause simulates the throw arc rising to
    // sim_launch_ft so the state machine sees a smooth ascent
    // rather than an instantaneous jump that confuses the tare logic.
    if (phase_ms < SIM_GROUND_MS - 500UL) {
      alt_ft = tare_baseline_ft;
    } else {
      // Rising throw arc: 0 → sim_launch_ft over 500ms
      // We don't have sim_launch_ft yet (seedNewFlight not called),
      // so use a fixed 40ft target for the throw preview.
      float throw_frac = (float)(phase_ms - (SIM_GROUND_MS - 500UL)) / 500.0f;
      alt_ft = tare_baseline_ft + 40.0f * sinf(throw_frac * (PI / 2.0f));
    }
    if (phase_ms >= SIM_GROUND_MS) {
      seedNewFlight();
      sim_in_flight = true;
      sim_phase_ms  = now;
    }
    return;
  }

  // ── Flight arc phase ─────────────────────────────────────
  if (phase_ms >= sim_flight_ms) {
    // Flight complete
    alt_ft        = tare_baseline_ft;
    sim_in_flight = false;
    sim_phase_ms  = now;
    logts(); Serial.println("[SIM] Landed");
    return;
  }

  // Piecewise realistic altitude profile:
  //   0% – 20%: sinusoidal climb from launch_ft to peak_ft
  //   20% – 80%: power-law gentle sink from peak to ~15ft
  //   80% – 100%: linear descent from ~15ft to 5ft (landing)
  float t = (float)phase_ms / (float)sim_flight_ms;  // 0.0 → 1.0

  float a;
  if (t <= 0.20f) {
    // Fast climb — sin curve from launch_ft up to peak_ft
    float frac = t / 0.20f;
    a = sim_launch_ft + (sim_peak_ft - sim_launch_ft) * sinf(frac * (PI / 2.0f));
  } else if (t <= 0.80f) {
    // Gradual soar/sink — power curve from peak down to ~15ft
    float frac = (t - 0.20f) / 0.60f;
    float sink_to = 15.0f + tare_baseline_ft;
    a = sim_peak_ft - (sim_peak_ft - sink_to) * powf(frac, 1.5f);
  } else {
    // Final descent — linear from ~15ft to 5ft
    float frac = (t - 0.80f) / 0.20f;
    a = (15.0f + tare_baseline_ft) - 10.0f * frac;
  }
  alt_ft = max(a, tare_baseline_ft + 4.0f); // floor at 4ft — must cross 5ft threshold to trigger landing
}

void flightSimUpdateAltitude(float& alt_ft, unsigned long now) {
  // ── Simulation modes (tilt_mode=true, sim_mode=1 or 2) ───────

  if (!calibration_done) {
    calibration_done   = true;
    tare_baseline_ft   = 0.0f;
    last_landed_alt_ft = 0.0f;  // reset so throw height computes correctly from sim baseline
    flight.state       = STATE_GROUND;
    if (sim_mode == 2) {
      logts(); Serial.printf("[SIM] Task-aware sim ready — task_id=%u (%s)  R%u G%u\n",
                    contest_task_id, taskName(contest_task_id),
                    contest_round_num, contest_group_num);
      logts(); Serial.println("[SIM]   Flight durations from task plan +/-10s jitter  Launch: 30-60ft");
      logts(); Serial.printf("[SIM]   tare_baseline_ft=%.1f  last_landed_alt_ft=%.1f\n",
                    tare_baseline_ft, last_landed_alt_ft);
    } else {
      logts(); Serial.println("[STATE] TILT MODE — skipping calibration → GROUND");
    }
  }

  if (sim_mode == 2) {
    updateTaskAwareSimulation(alt_ft, now);
    return;
  }

  // ── Tilt simulation (mode 1) — altitude from IMU tilt angle ─
  // 0–15° → 0ft (GROUND)  15–45° → 10ft (LAUNCH_WIN)  45–90° → tilt°=ft
  if (imu_present) {
    float t = imu_tilt_deg;
    if      (t < 15.0f) alt_ft = tare_baseline_ft + 0.0f;
    else if (t < 45.0f) alt_ft = tare_baseline_ft + 10.0f;
    else                alt_ft = tare_baseline_ft + t;
  }
}
