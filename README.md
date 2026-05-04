# F3kTransponder

ESP32-S3 based flight unit for F3K (discus launch glider) competition logging
and telemetry. Captures launch height, flight duration, and altitude data via
on-board sensors, then streams scoring and diagnostic data over WiFi to the
F3K BaseStation server.

Part of a three-repo system:

- **F3kTransponder** (this repo) — on-board ESP32-S3 firmware
- [F3kBaseStation](https://github.com/lytlebrent3/F3kBaseStation) — Python
  scoring/contest server
- [F3kLowLaunch](https://github.com/lytlebrent3/F3kLowLaunch) — radio-side
  Lua widgets for ETHOS / OpenTX / EdgeTX

## Hardware

- **MCU:** Adafruit QT Py ESP32-S3 (no PSRAM), 8 MB flash, tinyuf2 partitions
- **Pressure / altitude:** Adafruit DPS310 (I²C, STEMMA QT)
- **IMU:** Adafruit LSM6DSO32 (±32 g accel, ±2000 dps gyro)
- **GPS:** Adafruit PA1010D (mini GPS, I²C)
- **Status LED:** on-board NeoPixel
- **Storage:** LittleFS for config and logs

## Repository layout

```
F3kTransponder/
├── LICENSE
├── README.md
└── f3k_flight_unit_gps/
    ├── f3k_flight_unit_gps.ino   # main sketch
    ├── partitions.csv             # custom partition table
    ├── secrets.example.h          # template (committed)
    └── secrets.h                  # real credentials (gitignored)
```

The sketch lives in a folder of matching name so the Arduino IDE recognizes it.

## First-time setup

### 1. Install Arduino IDE

Use Arduino IDE 2.x and add the **esp32 by Espressif Systems** board package
(version 3.3.8 or compatible). In the Boards Manager, search for `esp32`.

In **Tools → Board → esp32**, select **Adafruit QT Py ESP32-S3 No PSRAM**.
The default board options used in this project:

- Flash size: **8 MB**
- Partition scheme: **TinyUF2 8MB**
- USB CDC On Boot: **Enabled**
- CPU frequency: **240 MHz** (firmware drops to 80 MHz at runtime)

### 2. Install required libraries

Via **Sketch → Include Library → Manage Libraries**:

- Adafruit DPS310
- Adafruit LSM6DS
- Adafruit GPS Library
- Adafruit NeoPixel
- Adafruit Unified Sensor (pulled in automatically)
- Adafruit BusIO (pulled in automatically)
- ESP Async WebServer
- Async TCP

### 3. Configure WiFi credentials

The sketch will not compile without a `secrets.h`. Create it from the template:

```bash
cd f3k_flight_unit_gps
cp secrets.example.h secrets.h
```

Then edit `secrets.h` and set your network:

```c
#define WIFI_SSID     "YourBaseStationSSID"
#define WIFI_PASSWORD "YourBaseStationPassword"
```

`secrets.h` is gitignored, so your credentials never leave your machine.
`secrets.example.h` is the template that *is* committed — keep it in sync
with `secrets.h` when you add new fields, but never put real values in it.

### 4. Build and flash

1. Open `f3k_flight_unit_gps/f3k_flight_unit_gps.ino` in Arduino IDE
2. Select the correct board and COM port
3. Click **Verify** to compile, then **Upload** to flash

Open the **Serial Monitor** at **115200 baud** and reset the unit. A clean
boot looks like this:

```
=== F3K Flight Unit ===
[BOOT] Reset reason: POWERON
LittleFS: mounted OK
DPS310 OK
LSM6DSO32 OK
PA1010D GPS OK
Connecting to F3KBase as 192.168.8.4
Connected! IP: 192.168.8.4
UDP scorer  → 192.168.8.101:5005
UDP debug   → 192.168.8.101:4213
Web overlay: http://192.168.8.4/
```

### 5. Pair with the BaseStation

The unit expects a BaseStation server on the configured network at
`192.168.8.101` listening on UDP `5005` (scoring) and `4213` (debug).
See the [F3kBaseStation repo](https://github.com/lytlebrent3/F3kBaseStation)
for the server side.

## Configuration

Per-unit settings (unit ID, web overlay on/off, current task ID, etc.) live
in `config.json` on LittleFS, not in the firmware. The unit creates a default
config on first boot. To change it on a deployed unit, use the web overlay
at `http://<unit-ip>/`.

## Web overlay

Once connected, the unit serves a small status page at `http://<unit-ip>/`
useful for field debugging — current task, last flight altitude, baseline
tare, and live sensor readings.

## Debug / simulation modes

The firmware supports a built-in flight simulator for bench testing without
launching the model. With `DEBUG_TILT_MODE 2`, the unit replays an autonomous
parabolic flight profile (10 s duration, 10 ft peak, 5 s ground pause)
repeatedly until the working window ends.

> **Note:** simulator-driven flight durations are intentionally short
> (~10 s) and don't reflect real F3K task durations. For real flights,
> disable the sim mode in the source.

## Versioning

- **v1.0-baseline** — first Git-tracked commit. Functionally equivalent to
  the pre-Git working firmware, with WiFi credentials extracted to
  `secrets.h` (no behavior change).

## License

See [LICENSE](LICENSE).
