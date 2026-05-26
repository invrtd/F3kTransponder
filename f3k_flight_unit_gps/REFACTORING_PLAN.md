# F3K Flight Unit Firmware — Refactoring Plan

## Current State (Baseline)

**Before refactoring:** ~4,750 lines in monolithic .ino  
**After Phase 1 (completed):** 3,354 lines in .ino + modular headers

### Phase 1: ✅ Complete
- ✅ **html.h** — All pilot UI HTML/CSS/JS (1,264 lines)
- ✅ **conf.h** — Configuration constants (46 lines)  
  - Thresholds: `CAL_BUF_SIZE`, `LAUNCH_G_THRESHOLD`, `NEAR_GROUND`, etc.
  - Timing constants: `MIN_FLIGHT_MS`, `LAUNCH_WIN_MS`, `AUTO_WINDOW_SECS`
- ✅ **types.h** — Enums and structs (47 lines)
  - State enum: `CALIBRATING`, `GROUND`, `LAUNCH_WIN`, `FLIGHT`, `LANDED`
  - Task enum: `TASK_DISTANCE`, `TASK_DURATION`, etc.
  - `struct Config` for file-persisted settings
- ✅ **globals.h** — Global variables (66 lines)
  - Sensor state: `dps_present`, `imu_present`, `gps_present`, `cal_buf[]`
  - Flight tracking: `flight_start_epoch_ms`, `launch_alt_ft`
  - Scoring: `score_mode`, `flight_scores[]`, `window_scores[]`
  - WiFi: `ap_mode`, `wifi_active`
- ✅ **webserver.h / webserver.cpp** — Started (9 + 5 lines)

---

## Phase 2: Sensors Hardware Abstraction

**Goal:** Hide sensor I/O behind clean APIs; main loop just calls "read all sensors"

**Files to create:**
- **sensors.h** (declaration only)
- **sensors.cpp** (implementation)

**Functions to extract from .ino:**

### Barometer (DPS310)
```cpp
// sensors.cpp
void initBarometer()                    // Line ~1885: sensor setup
void readBarometer()                    // Periodic read, update alt_ft
float pressureToAltitudeFeet(float p)   // Line 414: conversion formula
```
**Exposes:** 
```cpp
extern float baro_alt_ft;       // current altitude (feet)
extern float baro_pressure_hpa;
extern float baro_temp_c;
```

### IMU (LSM6DSO32)
```cpp
// sensors.cpp  
void initImu()                  // Line ~1893: sensor setup
void readImu()                  // Line 421: poll accel/gyro
void calibrateImuOffset()       // Tilt calibration
```
**Exposes:**
```cpp
extern float imu_accel_g[3];    // x, y, z
extern float imu_gyro_dps[3];
extern float imu_max_g;         // peak in loop
extern bool  imu_calibrated;
```

### GPS (PA1010D)
```cpp
// sensors.cpp
void initGps()                  // Line ~1910: sensor + NMEA listener
void readGps()                  // Line ~1920: call in loop
void parseNmeaSentence()        // Parse NMEA frames
```
**Exposes:**
```cpp
extern float gps_lat, gps_lon;
extern float gps_hdop;
extern uint32_t gps_timestamp_ms;
extern bool  gps_has_fix;
extern uint16_t gps_sats;
```

**Dependencies:**
- Requires: `conf.h` (thresholds), `types.h` (sensor enums)
- Depends on: Adafruit libraries (DPS310, LSM6DS, GPS), Wire1
- Used by: Main loop (read all), State machine (launch detection), Logging (altitude/accel)

**Compilation gate:**
```
Arduino IDE: Verify sketch compiles
- No undefined symbols
- All Adafruit includes resolve
- Wire1 initialization works
```

---

## Phase 3: Logging Subsystem

**Goal:** Encapsulate all LittleFS CSV logging; main loop calls `logSample()` only

**Files to create:**
- **logger.h**
- **logger.cpp**

**Functions to extract:**

```cpp
// From lines 741–1023
void openWindowLog()            // Setup CSV file, write header
void closeWindowLog()           // Flush & close CSV
void logSample(float alt_ft, float pressure_hpa, float temp_c)  // Line 850
void writeSummaryLog()          // Line 926: flight summary CSV
void pruneLogsIfNeeded()        // Line 621: manage LittleFS quota

// Helper state
unsigned long log_file_size = 0;
uint16_t window_number = 0;        // From globals.h → logger.cpp
float log_hz = 8.0;                // Dynamic based on window duration
```

**Exposes:**
```cpp
// logger.h
void logger_init();
void logger_open_window(uint16_t window_id, uint32_t duration_s);
void logger_close_window();
void logger_log_sample(float alt_ft, float pressure_hpa, float temp_c);
void logger_write_summary(float duration_s, float score);

// State queries
bool logger_is_active();
uint32_t logger_bytes_written();
```

**Dependencies:**
- Requires: `conf.h` (log Hz, file paths), `types.h` (Config struct)
- Depends on: LittleFS, POSIX file APIs
- Used by: Main loop (sample logging), Window close (summary), HTTP endpoints (file serving)

**Special handling:**
- `littlefs_streaming` refcount (defer `LittleFS.end()` while HTTP streams active)
- Thread-safe: loop writes at 8 Hz, HTTP reads async; use volatile flags

**Compilation gate:**
```
- LittleFS mounts and reads
- CSV sample format matches expected wire
- /logs/ directory logic works
```

---

## Phase 4: Scoring Engine

**Goal:** Decouple scoring formula from state machine; switch modes at runtime

**Files to create:**
- **scoring.h**
- **scoring.cpp**

**Functions to extract:**

```cpp
// Line 715: scoring formula selector
float calculateScore(float dur_s, float throw_height_ft)

// Mode-specific implementations
float scoreModeSecs(float dur_s, float throw_height_ft);   // Secs - Ft
float scoreModeJoeD(float dur_s, float throw_height_ft);   // JoeD V1
```

**Exposes:**
```cpp
// scoring.h
enum ScoreMode { MODE_SECS_FT = 0, MODE_JOE_D_V1 = 1 };

float scoring_calculate(float duration_s, float throw_height_ft, ScoreMode mode);
void  scoring_set_mode(ScoreMode mode);
ScoreMode scoring_get_mode();

// Constants
const float SCORE_CAP = 1100.0;
const float SCORE_FLOOR = 0.0;
```

**Dependencies:**
- Requires: `conf.h` (formulas, caps)
- Used by: State machine (post-flight), HTTP `/setscore` endpoint, UDP scorer packet

**Compilation gate:**
```
- Both modes compile
- Formulas match ICD spec exactly
- No floating-point surprises
```

---

## Phase 5: State Machine & Flight Detection

**Goal:** Encapsulate `CALIBRATING → GROUND → LAUNCH_WIN → FLIGHT → LANDED` with clean state transition API

**Files to create:**
- **state_machine.h**
- **state_machine.cpp**

**Functions to extract:**

```cpp
// Line 1395: main state loop
void updateStateMachine(float alt_ft)

// Flight detection logic (inline in updateStateMachine currently)
bool detectLaunch(float accel_g, float alt_ft);
bool detectLanding(float accel_g, unsigned long flight_ms);
void transitionToState(State new_state);
```

**Exposes:**
```cpp
// state_machine.h
enum State { CALIBRATING, GROUND, LAUNCH_WIN, FLIGHT, LANDED };

void sm_init(ScoreMode mode);
void sm_update(float alt_ft, float accel_g, unsigned long now_ms);
State sm_current_state();
void sm_set_state(State s);  // For testing / override

// Accessors
float sm_flight_start_epoch_ms();
float sm_launch_alt_ft();
unsigned long sm_flight_duration_ms();
float sm_throw_height_ft();
```

**State transition table (hardcoded logic):**
```
CALIBRATING → GROUND    (on accel drop below threshold after cal window)
GROUND      → LAUNCH_WIN (on window-open timer or UDP 0x20)
LAUNCH_WIN  → FLIGHT    (on 3.5G accel spike OR 8ft alt rise, + confirmation window)
FLIGHT      → LANDED    (on 4.0G impact spike AND >10s flight duration)
LANDED      → GROUND    (window closes)
```

**⚠️ ICD-CRITICAL:** State enum values are wire-format. Must match F3K_ICD_v1_7.docx exactly:
- `CALIBRATING = 0`
- `GROUND = 1`
- `LAUNCH_WIN = 2`
- `FLIGHT = 3`
- `LANDED = 4`

**Dependencies:**
- Requires: `conf.h` (thresholds), `sensors.h` (accel/alt), `types.h` (State enum)
- Used by: Main loop, UDP packets (state byte), HTTP UI display

**Compilation gate:**
```
- All state enums unchanged (verify against ICD)
- Launch/land detection thresholds match conf.h
- Transition callbacks fire correctly
```

---

## Phase 6: Network & UDP

**Goal:** Separate packet I/O from business logic; centralize inbound command handling

**Files to create:**
- **network.h**
- **network.cpp**

**Functions to extract:**

```cpp
// Outbound UDP (5 Hz / 1 Hz depending on state)
void sendUdpPacket()            // Line 458: scoring packet to 192.168.8.101:5005
void sendDebugPacket()          // Line 508: debug telemetry to :4213
void sendGpsPacket()            // Line 553: GPS fix data to :4211
void sendAnnouncement()         // Line 1179: log announce on window close (:4214)

// Inbound UDP (listener on port 5006)
void checkWindowCommand(unsigned long now)  // Line 1237: 0x20 (start), 0x21 (prep)

// WiFi mode switching
void wifi_shutdown_start()
void wifi_shutdown_cancel()
void wifi_restore()
```

**Exposes:**
```cpp
// network.h
void net_init(bool ap_mode);
void net_update(unsigned long now);  // Call in main loop
void net_send_score(State state, float score, uint16_t window_id);
void net_send_debug(const char* tag, float value);
void net_send_gps(float lat, float lon, uint16_t sats, uint16_t hdop);
void net_announce_log(uint16_t window_id, uint32_t file_size);

// Inbound command callback (fired on 0x20/0x21)
typedef void (*WindowCommandCallback)(uint8_t op, uint16_t window_id, uint32_t secs);
void net_set_window_callback(WindowCommandCallback cb);

// Power management
void net_shutdown_wifi();
void net_restore_wifi();
```

**UDP Packet formats (from header comments, lines ~69–76):**
- **Packet 1:** Scoring data (5 Hz / 1 Hz) → port 5005
  - Byte 0–1: packet ID + flags
  - Byte 2: state enum (MUST match types.h State enum!)
  - Byte 3+: float fields (altitude, score, flight time)
- **Packet 4:** Debug telemetry (5 Hz) → port 4213
- **Packet 2:** GPS fix (1 Hz) → port 4211
- **Packet 5:** Log announcement → port 4214
- **Listener:** Port 5006 inbound
  - `0x20` = window-start (window_id, duration)
  - `0x21` = prep-countdown (duration, countdown secs)

**⚠️ ICD-CRITICAL:** Byte layouts are wire-format. Don't reorder fields or change state enum position.

**Dependencies:**
- Requires: `types.h` (State enum), `state_machine.h` (current state)
- Depends on: WiFi, AsyncWebServer (for TCP), WiFiUDP
- Used by: Main loop (periodic sends), state machine (command trigger)

**Compilation gate:**
```
- UDP sockets bind on correct ports
- AsyncWebServer still works alongside UDP
- WiFi on/off doesn't crash
```

---

## Phase 7: Window & Timer Management

**Goal:** Encapsulate hardware timer ISRs and window lifecycle; main loop calls high-level "open/close window"

**Files to create:**
- **window.h**
- **window.cpp**

**Functions to extract:**

```cpp
// Hardware timers (lines 265–294)
void IRAM_ATTR onWindowOpenTimer();     // ISR: Timer 2
void IRAM_ATTR onWindowCloseTimer();    // ISR: Timer 3
void armWindowOpenTimer(uint32_t delay_ms);
void armWindowCloseTimer(uint32_t delay_ms);
void disarmWindowOpenTimer();
void disarmWindowCloseTimer();

// Window lifecycle
void openWindow(uint16_t window_id, uint32_t duration_s);
void closeWindow();
```

**Exposes:**
```cpp
// window.h
typedef void (*WindowEventCallback)(uint8_t event);  // Event: OPEN, CLOSE, PREP_FIRED
void window_init();
void window_arm_open(uint32_t delay_ms);
void window_arm_close(uint32_t delay_ms);
void window_cancel();
void window_set_callback(WindowEventCallback cb);

// State queries
bool window_is_active();
uint32_t window_remaining_ms();
```

**State to track:**
```cpp
volatile unsigned long window_open_latch_ms;   // ISR fire timestamp
volatile unsigned long window_close_latch_ms;
unsigned long prep_fire_ms;                    // When to auto-open
```

**Dependencies:**
- Requires: `conf.h` (timing constants), `types.h`
- Depends on: ESP32 hardware timers (Timer 2, Timer 3)
- Used by: Main loop, state machine (detect open/close events)

**Compilation gate:**
```
- ISRs compile to IRAM_ATTR without warnings
- Timers arm/disarm without hangs
- No race conditions (volatile flags)
```

---

## Phase 8: Main Loop Simplification & HTTP Handlers

**Goal:** Reduce `setup()` and `loop()` to ~200 lines of clear orchestration; move HTTP endpoints to webserver.cpp

**Files to refactor:**
- **f3k_flight_unit_gps.ino** (compress to ~200 lines)
- **webserver.cpp** (expand to host all ~15 endpoints)

**Main loop skeleton after refactor:**
```cpp
void setup() {
  // Initialize all subsystems in order
  sensors_init();
  logger_init();
  window_init();
  network_init(ap_mode);
  state_machine_init();
  
  // Setup web server
  server.on("/pilot", HTTP_GET, handlePilot);
  server.on("/pstatus", HTTP_GET, handlePStatus);
  // ... (all endpoints move to webserver.cpp)
  server.begin();
}

void loop() {
  unsigned long now = millis();
  
  // Read sensors (blocking ~20 ms per loop)
  sensors_update();
  
  // Update state machine
  state_machine_update(baro_alt_ft, imu_max_g, now);
  
  // Log active flight
  if (window_is_active()) {
    logger_log_sample(baro_alt_ft, baro_pressure_hpa, baro_temp_c);
  }
  
  // Network I/O (UDP sends)
  network_update(now);
  
  // Check inbound window commands
  window_event_check(now);
  
  delay(12);  // ~83 Hz loop, leaves headroom
}
```

**HTTP endpoints to extract from .ino to webserver.cpp:**

| Endpoint | Handler | Type | Purpose |
|----------|---------|------|---------|
| `/pilot` | `handlePilot()` | GET | Pilot UI (AP mode) |
| `/wstatus` | `handleWStatus()` | GET | Window status page (AP mode) |
| `/pstatus` | `handlePStatus()` | GET | JSON telemetry for polling |
| `/pgps` | `handlePGps()` | GET | JSON GPS state |
| `/pstart?secs=N` | `handlePStart()` | GET | Start window + countdown |
| `/pstop` | `handlePStop()` | GET | Stop window / cancel countdown |
| `/log?n=N&del=1` | `handleLog()` | GET | Serve sensor CSV (+ optionally delete) |
| `/summary?n=N` | `handleSummary()` | GET | Serve score summary CSV |
| `/logs` | `handleLogsBrowser()` | GET | Log browser UI |
| `/delete?f=name` | `handleDelete()` | GET | Delete single file |
| `/wipe-logs?confirm=YES&extra=SURE` | `handleWipeLogs()` | GET | Delete all logs (escape hatch) |
| `/setscore?m=0\|1` | `handleSetScore()` | GET | Switch scoring mode |
| `/settilt?v=0\|1` | `handleSetTilt()` | GET | Toggle tilt mode |
| `/debug` | `handleDebug()` | GET | Full telemetry overlay |

**Dependencies:**
- All phases 2–7 must be complete before this phase
- webserver.cpp needs: `logger.h`, `state_machine.h`, `sensors.h`, `scoring.h`, `window.h`, `network.h`

**Compilation gate:**
```
- All 15+ endpoints compile
- AsyncWebServer handles all simultaneously
- Main loop compiles without warnings
- Memory footprint doesn't exceed partition (check .map file)
```

---

## Header Dependency Graph

```
conf.h (standalone, no includes)
types.h (includes conf.h)
globals.h (includes types.h, conf.h)

sensors.h (includes conf.h)
logger.h (includes conf.h, types.h)
scoring.h (includes conf.h)
state_machine.h (includes types.h, conf.h, sensors.h, scoring.h)
window.h (includes conf.h)
network.h (includes types.h, state_machine.h)

webserver.h (includes all of above)
webserver.cpp (includes webserver.h, html.h)

f3k_flight_unit_gps.ino (includes all .h files)
```

**Key to avoid circular deps:**
- `sensors.h` does NOT include `state_machine.h`
- `state_machine.h` includes `sensors.h` (to call sensor getters)
- `network.h` includes `state_machine.h` (to access state)
- `logger.h` does NOT include `state_machine.h` (logger is stateless)

---

## Compilation & Verification Strategy

After **each phase**:

1. **Syntax check:** Arduino IDE "Verify" (Ctrl+R)
2. **Memory check:** Inspect .map file for footprint (must stay in 3 MB app partition)
3. **Link check:** No undefined references
4. **Flash test:** Upload to board, monitor 115200 baud for boot sequence

Minimum acceptance criterion (fresh flash + POWERON reset):
```
=== F3K Flight Unit ===
[BOOT] Reset reason: POWERON
[BOOT] Build: <date> <time>
LittleFS: mounted OK
DPS310 OK
LSM6DSO32 OK
PA1010D GPS OK
Connecting to <SSID> as 192.168.8.<unit_id>
Connected! IP: 192.168.8.<unit_id>
UDP scorer  → 192.168.8.101:5005
UDP debug   → 192.168.8.101:4213
Web overlay: http://192.168.8.<unit_id>/
```

If any sensor shows `FAIL` instead of `OK`, **STOP** — don't continue to next phase until baseline works.

---

## ICD-Critical Invariants (Must NOT change)

| Invariant | Location | Reason |
|-----------|----------|--------|
| State enum values (0–4) | types.h | Wire-format byte 1 of UDP scoring packet |
| Task enum values (0–7) | types.h | Wire-format in task snapshot |
| UDP packet byte layouts | network.cpp | F3K_ICD_v1_7.docx, scorer expects exact offsets |
| Inbound command opcodes (0x20, 0x21, 0x22) | network.cpp | Server-defined, unit must follow |
| Scoring formula coefficients | scoring.cpp | Already in ICD; must match exactly |
| Launch/land thresholds (G, feet) | conf.h | Hardware-tuned; don't guess |

---

## Timeline & Order

**Recommended sequence (one at a time, verify before next):**

1. Phase 2 — **Sensors** (~2 hrs)
   - Extract `readImu()`, barometer, GPS
   - Verify boot sequence still works
   - Test altitude reading on bench (tilt simulator)

2. Phase 3 — **Logging** (~1.5 hrs)
   - Extract log lifecycle
   - Verify CSV files still write during test window
   - Check LittleFS quota logic

3. Phase 4 — **Scoring** (~30 min)
   - Extract formula selector
   - Verify `/setscore?m=0|1` endpoint still works

4. Phase 5 — **State Machine** (~2 hrs)
   - Extract state transitions, launch/land detection
   - **Most critical phase** — verify state enum unchanged
   - Test with real throw or simulator

5. Phase 6 — **Network/UDP** (~1.5 hrs)
   - Extract UDP sends, inbound command handling
   - Verify packets still reach scorer at 192.168.8.101
   - Test window-open command from server

6. Phase 7 — **Window/Timer** (~1 hr)
   - Extract hardware timer ISRs
   - Verify timers still fire autonomously
   - Test window-open grace window (~3s skew absorption)

7. Phase 8 — **Main Loop & HTTP** (~2 hrs)
   - Move endpoints to webserver.cpp
   - Simplify main loop
   - Final full system test

**Total estimated:** ~11 hours of focused work + verification on hardware

---

## Notes for Success

- **One change at a time.** Don't combine phases — each must compile and run before next.
- **No speculative refactoring.** If a function doesn't cleanly separate, leave it in place for now.
- **Keep CLAUDE.md invariants visible.** Print or bookmark the "Cross-repo invariants" section.
- **Respect the wire-format contract.** State enum, UDP layouts, and opcodes touch F3kBaseStation — changing them silently breaks scoring.
- **Test on real hardware after each phase.** The pre-Git baseline (v1.0-baseline tag) boots cleanly; if your refactor breaks that, revert one phase and debug.
- **Watch for interdependencies during implementation.** If Phase 3 (logging) needs something from Phase 5 (state), refactor the dependency back into globals.h temporarily.

---

## Success Criteria (Phase 8 complete)

- [ ] All 8 phases complete and compiling
- [ ] Boot sequence unchanged (all sensors OK)
- [ ] Window open/close fires on time (simulator + real board)
- [ ] CSV logs write at correct Hz (8 Hz ≤600s, 4 Hz >600s)
- [ ] UDP packets reach scorer with correct state byte
- [ ] HTTP endpoints all responding (pilot UI, logs browser, etc.)
- [ ] WiFi shutdown/restore works in STA mode
- [ ] Main .ino ≤ 200 lines, readable orchestration
- [ ] No warnings during compilation
- [ ] Memory footprint stays under 3 MB app partition

---

**Ready to start Phase 2?** Or need clarification on any phase first?
