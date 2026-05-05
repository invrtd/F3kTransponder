# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP32-S3 Arduino firmware for an on-board F3K (discus-launch glider) flight unit. It captures launch height + flight duration from a barometer/IMU/GPS stack, streams scoring telemetry over UDP to a separate Python BaseStation, and logs full-rate sensor CSVs to LittleFS.

Companion repos:

- [F3kBaseStation](https://github.com/lytlebrent3/F3kBaseStation) — Python scoring/contest server, runs on the same WiFi network at `192.168.8.101`. Owns the canonical contest state and pushes window-open / prep / time-sync commands to the unit on UDP `5006`.
- [F3kLowLaunch](https://github.com/lytlebrent3/F3kLowLaunch) — radio-side Lua widgets for ETHOS / OpenTX / EdgeTX. Independent of this firmware; same author, related sport.

The wire-format ICD between this firmware and BaseStation is `F3K_ICD_v1_7.docx` (not in repo). The header-comment packet diagrams in the `.ino` are the second-most-authoritative source; the ICD wins on conflict.

The entire firmware is a **single ~4600-line `.ino` sketch** at `f3k_flight_unit_gps/f3k_flight_unit_gps.ino`. There is no library/module split — everything (state machine, web UI HTML in `PROGMEM` raw-strings, UDP protocol, LittleFS logger, simulator) lives in that file. When making changes, prefer editing in place rather than splitting files; the sketch-folder name must match the `.ino` filename for Arduino IDE to open it.

## Working principles

- **One change at a time, verified on hardware before the next.** This rule was paid for. A prior debugging session on the F3kLowLaunch sister repo accumulated multiple speculative patches in sequence, hit a memory error mid-stack, and required a full revert to a known-good baseline. Don't repeat it here. After each meaningful edit, compile, flash, and confirm the boot pattern in the next section before continuing.
- **The pre-Git working firmware is tagged `v1.0-baseline`.** When in doubt, `git diff v1.0-baseline` to see what's drifted from a known-good state. This baseline was verified end-to-end on hardware: clean POWERON boot, all sensors initialized, WiFi associated, UDP listeners up, web overlay reachable, sim activated cleanly.
- **No silent rewrites of wire-format constants.** State enum values, UDP packet byte layouts, and the inbound command opcodes (`0x20`/`0x21`/`0x22`) are part of the ICD with `F3kBaseStation`. Changing them on this side without a coordinated change on the server side will silently break scoring with no compile-time warning.

## Build / flash

Arduino IDE 2.x with **esp32 by Espressif Systems** board package. Required Tools settings:

- Board: **Adafruit QT Py ESP32-S3 No PSRAM**
- Flash size: **8 MB**
- Partition scheme: **Custom** — uses `f3k_flight_unit_gps/partitions.csv` (3 MB app / 4.9 MB LittleFS). The default TinyUF2 partition will *not* hold the log capacity the rest of the firmware assumes. **Reflashing after changing partition scheme erases LittleFS** — `f3k_config.json` (unit_id, etc.) must be rewritten via the web overlay or a config-writer sketch.
- USB CDC On Boot: Enabled. Serial Monitor at 115200.

Required libraries (Library Manager — note the specific forks):

- Adafruit DPS310, LSM6DS, GPS Library, NeoPixel, Unified Sensor, BusIO
- **ESPAsyncWebServer — mathieucarbou fork v3.10.3+** (not the original)
- **AsyncTCP — dvarrel fork v3.4.10+** (not the original)

`secrets.h` is required to compile and is gitignored. Copy `secrets.example.h` → `secrets.h` and fill in `WIFI_SSID` / `WIFI_PASSWORD`. When adding new secret fields, update `secrets.example.h` in lockstep (template only, no real values).

There is no test suite, linter, or CI in this repo — verification is on-device. The minimum acceptance criterion for "this builds and runs" is the following Serial Monitor pattern at 115200 baud after a fresh flash and POWERON reset:

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

If any sensor line shows `FAIL` instead of `OK`, the firmware will continue running with that sensor's `_present` flag false (see "Sensors are non-fatal" below) — but on a known-good board, all three should come up. A missing or hung WiFi line on a known-good network usually means the AsyncTCP fork mismatch — see required libraries above.

## Architecture (big picture)

**State machine** (`updateStateMachine`, ~line 1510): `CALIBRATING → GROUND → LAUNCH_WIN → FLIGHT → LANDED → GROUND`. State enum values are wire-format and **must match the ICD exactly** — they're sent in every UDP packet's byte 1.

**Two WiFi modes**, picked at boot:

- **STA** (default): connects to BaseStation AP at `192.168.8.1`, takes static IP `192.168.8.<unit_id>`, sends UDP to the scorer at `192.168.8.101`.
- **AP** (fallback or `FORCE_AP_MODE 1`): becomes hotspot `F3K-Unit-XX` on `192.168.<unit_id>.1`. Used for solo practice and post-flight log retrieval. AP mode adds an "auto-window" feature (first throw opens a 9:55 window automatically) and richer pilot UI tabs.

**WiFi power management is intentional and load-bearing.** During an active window in STA mode, WiFi is *shut down* (`wifi_shutdown_pending`) to save battery and reduce RFI. The shutdown is deferred from the receive path to the main loop so the AsyncTCP task on Core 0 can drain first — don't move it back inline. WiFi restarts after the window closes so the scorer can pull the log.

**UDP wire protocol** (see header comments at the top of the sketch for byte layouts — packets are big-endian, fixed-size):

- Outbound: `5005` scoring (5 Hz active / 1 Hz idle), `4211` GPS, `4213` debug, `4214` log announcement.
- Inbound on `5006`: `0x20` window-start, `0x21` prep-countdown, `0x22` time-of-day sync. Prep+start has a **3-second grace window** in `checkWindowCommand` — if a `0x20` arrives within 3 s of a timer-fired open, it adopts the new id/secs in place rather than closing+reopening (absorbs scorer/unit clock skew). Don't simplify this away.

**Hardware timers, not millis()**, drive the two timing-critical edges. `Timer 2` opens the window, `Timer 3` closes it. ISRs (`onWindowOpenTimer` / `onWindowCloseTimer`) only set volatile flags; the main loop captures `millis()` when it sees the flag — never call `millis()` from these ISRs (unsafe on ESP32 Arduino v3.x).

**Logging (LittleFS)**: each window writes `/logs/window_NNN.csv` at 8 Hz (≤600 s windows) or 4 Hz (>600 s), plus `/logs/summary_NNN.csv` at close. The log file is **pre-opened during the prep countdown** so window-open is just a header-flush, not a filesystem mount. `pruneLogsIfNeeded` auto-deletes oldest at 60% full. `littlefs_streaming` refcount defers `LittleFS.end()` while HTTP downloads are active — respect it when adding new endpoints that stream files.

**Config** lives in `/f3k_config.json` on LittleFS, parsed with a hand-rolled key:value scanner (no JSON lib). Defaults are baked into the `cfg` struct initializer. `window_number` persists across reboots so log filenames don't collide.

**I2C bus**: DPS310 (0x77), LSM6DSO32 (0x6A), and PA1010D GPS (0x10) all share `Wire1` (STEMMA QT, GPIO40/41) by default. **The PA1010D clock-stretches Wire1 for 30–65 ms while parsing NMEA**, which spikes loop-max and contends with the IMU. The polling cadence in `loop()` (GPS at 100 ms, not 20 ms) is tuned for this — be careful raising it. A future board would put GPS on a separate bus.

**Sensors are non-fatal.** DPS310 retries 10× then continues with `dps_present=false` (altitude=0, scoring effectively disabled but logging still works). IMU and GPS likewise — `imu_present`/`gps_present` flags gate every read site. Don't add code that assumes a sensor exists.

**Scoring formulas** are runtime-selectable via `/setscore?m=0|1` (`score_mode` global): mode 0 = Secs-Ft (window-independent), mode 1 = JoeD V1 (window-relative time^0.425 + height bonus/penalty). `calculateScore` is the single source of truth.

**Simulator** (`DEBUG_TILT_MODE`): compile-time flag, default `2` in committed source.
- `0` = real flight
- `1` = altitude derived from physical tilt angle (bench test, requires IMU)
- `2` = autonomous parabolic flight cycles, task-aware (reads `contest_task_id` and replays realistic durations from `getTaskTarget`'s task plan tables)

`tilt_mode` (runtime bool, toggled via `/settilt`) and `sim_mode` (uint8) together describe the active mode — both modes 1 and 2 set `tilt_mode=true` to share the calibration-skip path. **Sim flights are short (~10–600 s synthetic);** disable for real launches.

**Web server** (port 80, AsyncWebServer): endpoints are listed in the file header comment block (lines ~53–66). Pilot UI HTML/CSS/JS is inlined as `PROGMEM` raw-string literals (`PILOT_HTML`, etc.) — large blocks starting around line 1984. Edit in place; there is no separate static-files step.

## Conventions worth knowing

- **`logts()` precedes every `Serial.printf`** — it stamps `[HH:MM:SS +Nms]` from GPS time when fix is valid, else just millis. Match this pattern for new log lines.
- **State enum values and UDP byte layouts are ICD-defined.** Changing a value or field offset is a wire-protocol break that ripples to F3kBaseStation. Treat the header-comment packet diagrams as authoritative.
- **No JSON library** — parsers/builders are hand-written. Stay consistent rather than pulling in ArduinoJson.
- The file header carries a `Source file CRC32` line and a verification python one-liner. If you make non-trivial source edits, update the CRC (or remove the line); don't leave a stale value.
- Line endings are LF in the repo (see `.gitattributes`), even on Windows.

## Cross-repo invariants and known smells

### The window-state contract with F3kBaseStation

Window state is currently owned by **both** the unit and the server, and the two can drift. This is a known architectural smell, not a feature — document it before "fixing" it on either side in isolation.

The intended model: **server is canonical**, unit is a follower. The server sends `0x20` (window-start) on UDP `5006` with a `window_id` and duration; the unit opens a window in response. `0x21` (prep-countdown) and `0x22` (time-sync) are the supporting cast.

The actual model: the unit's `Timer 2` fires autonomously on a ~420 s cadence (next-window cadence), and on fire it **opens the next window even if no `0x20` has arrived**. The 3-second grace window in `checkWindowCommand` exists to absorb the resulting clock skew when `0x20` does arrive nearby — but if it arrives *outside* the grace window (e.g. WiFi was down at the timer-fire instant), the unit and server now disagree about which window is active. A symptom seen in the field: the unit logs CSV files during the test-flight *prep period* between a closed window and the next, because its autonomous timer opened `window_N+1` ~200 ms after `window_N` closed, before WiFi reconnected and before any new task snapshot arrived from the server.

If you change anything in this area, hold this contract in mind:

1. **Don't add new autonomous opens.** The smell is already there; don't make it worse. Any new "open a window" path should be triggered by a server command, a deliberate AP-mode auto-window, or `DEBUG_TILT_MODE 2` — and the existing three already cover every legitimate case.
2. **Don't remove the 3 s grace window** without a coordinated server-side change. The grace exists because the server *will* send `0x20` slightly off the unit's local clock, and ripping it out causes phantom window-close-then-reopen events.
3. **A real fix lives across both repos.** Either the server commits to sending `0x20` reliably and the unit's autonomous timer becomes a fallback only (server-canonical), or the unit commits to publishing window-open events and the server learns to listen (unit-canonical). Pick one before patching.

### Secrets workflow

`secrets.h` is gitignored; `secrets.example.h` is the committed template. When you add a new secret field (e.g. an MQTT broker URL someday), update **both** files in the same commit — the example with placeholder, the real one with your value. The `.gitignore` rule is `f3k_flight_unit_gps/secrets.h`; it does not need updating.

### Line endings

`.gitattributes` pins source files to LF in the repo and converts to native on checkout. This forward-compat exists because `F3kBaseStation` is destined for a Pi port. Don't fight it; don't run `dos2unix` or `unix2dos` on tracked files.
