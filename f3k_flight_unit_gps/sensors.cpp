#include "sensors.h"
#include "globals.h"

#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_GPS.h>

// ============================================================
//  Private sensor objects
// ============================================================

static Adafruit_DPS310    dps;
static Adafruit_LSM6DSO32 lsm;

// GPS is always on Wire1 / STEMMA QT per project wiring.
static Adafruit_GPS gps_sensor(&Wire1);

// ============================================================
//  Public sensor globals declared in sensors.h
// ============================================================

// -- Barometer --
bool  dps_present = false;
float dps_temp_c  = 0.0f;

// -- IMU --
bool  imu_present = false;

float imu_accel_x = 0.0f;
float imu_accel_y = 0.0f;
float imu_accel_z = 0.0f;

float imu_gyro_x = 0.0f;
float imu_gyro_y = 0.0f;
float imu_gyro_z = 0.0f;

float imu_g_force  = 0.0f;
float imu_tilt_deg = 0.0f;

// -- GPS --
bool gps_present = false;
bool gps_fix     = false;

float gps_lat   = 0.0f;
float gps_lon   = 0.0f;
float gps_alt_m = 0.0f;

uint8_t gps_sats        = 0;
uint8_t gps_fix_quality = 0;
float   gps_hdop        = 99.9f;

uint8_t  gps_hour = 0;
uint8_t  gps_minute = 0;
uint8_t  gps_second = 0;
uint16_t gps_milliseconds = 0;

// ============================================================
//  Internal I2C setup guard
// ============================================================

static bool i2c_initialized = false;

static void sensors_init_i2c_once() {
  if (i2c_initialized) return;

#if USE_STEMMA_QT
  // STEMMA QT JST bus: SDA=GPIO41, SCL=GPIO40
  Wire1.begin(41, 40);
  Wire1.setClock(400000);
  Serial.println("I2C: STEMMA QT Wire1 SDA=41 SCL=40 400kHz");
#else
  // Standard QT Py ESP32-S3 pads: SDA=GPIO8, SCL=GPIO9
  Wire.begin(8, 9);
  Wire.setClock(400000);
  Serial.println("I2C: Standard Wire SDA=8 SCL=9 400kHz");

  // GPS still uses Wire1 / STEMMA QT even when scoring sensors use Wire.
  Wire1.begin(41, 40);
  Wire1.setClock(400000);
  Serial.println("I2C: GPS Wire1 SDA=41 SCL=40 400kHz");
#endif

  i2c_initialized = true;
}

// ============================================================
//  Barometer — DPS310
// ============================================================

float pressureToAltitudeFeet(float pressure_hpa) {
  return 44330.0f *
         (1.0f - powf(pressure_hpa / SEA_LEVEL_HPA, 0.1903f)) *
         M_TO_FT;
}

void sensors_init_barometer() {
  sensors_init_i2c_once();

#if USE_STEMMA_QT
  TwoWire* dps_wire = &Wire1;
#else
  TwoWire* dps_wire = &Wire;
#endif

  dps_present = false;

  if (!dps.begin_I2C(0x77, dps_wire)) {
    Serial.print("DPS310 not found — retrying");

    for (int attempt = 1; attempt <= 10; attempt++) {
      Serial.printf(" %d", attempt);
      delay(500);

      if (dps.begin_I2C(0x77, dps_wire)) {
        dps_present = true;
        break;
      }
    }

    Serial.println();

    if (!dps_present) {
      Serial.println("ERROR: DPS310 not found after 10 attempts.");
      Serial.println("Unit will continue with altitude=0.");
      return;
    }

    Serial.println("DPS310 OK on retry");
  } else {
    dps_present = true;
    Serial.println("DPS310 OK");
  }

  dps.configurePressure(DPS310_8HZ, DPS310_8SAMPLES);
  dps.configureTemperature(DPS310_8HZ, DPS310_8SAMPLES);
}

void sensors_read_barometer(float& alt_ft, float& pressure_hpa, float& temp_c) {
  if (!dps_present) {
    alt_ft       = 0.0f;
    pressure_hpa = SEA_LEVEL_HPA;
    temp_c       = dps_temp_c;
    return;
  }

  sensors_event_t temp_event;
  sensors_event_t pressure_event;

  dps.getEvents(&temp_event, &pressure_event);

  temp_c       = temp_event.temperature;
  dps_temp_c   = temp_c;
  pressure_hpa = pressure_event.pressure;

  // Basic sanity check. DPS310 should normally be near 800–1100 hPa.
  if (pressure_hpa > 800.0f && pressure_hpa < 1100.0f) {
    alt_ft = pressureToAltitudeFeet(pressure_hpa);
  } else {
    alt_ft = 0.0f;
  }
}

// ============================================================
//  IMU — LSM6DSO32
// ============================================================

void sensors_init_imu() {
  sensors_init_i2c_once();

#if USE_STEMMA_QT
  TwoWire* imu_wire = &Wire1;
#else
  TwoWire* imu_wire = &Wire;
#endif

  imu_present = false;

  if (!lsm.begin_I2C(0x6A, imu_wire)) {
    Serial.println("WARNING: LSM6DSO32 not found — IMU unavailable.");
    return;
  }

  imu_present = true;
  Serial.println("LSM6DSO32 OK");

  // Launch detection needs high-G range.
  lsm.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);

  // Match original loop timing: 26 Hz.
  lsm.setAccelDataRate(LSM6DS_RATE_26_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_26_HZ);

  // Wide gyro range for launch / catch events.
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  Serial.println("IMU: accel ±32G, gyro ±2000 dps, rate 26 Hz");
}

void sensors_read_imu() {
  if (!imu_present) return;

  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t temp_event;

  lsm.getEvent(&accel_event, &gyro_event, &temp_event);

  imu_accel_x = accel_event.acceleration.x;
  imu_accel_y = accel_event.acceleration.y;
  imu_accel_z = accel_event.acceleration.z;

  imu_gyro_x = gyro_event.gyro.x;
  imu_gyro_y = gyro_event.gyro.y;
  imu_gyro_z = gyro_event.gyro.z;

  const float mag = sqrtf(
    imu_accel_x * imu_accel_x +
    imu_accel_y * imu_accel_y +
    imu_accel_z * imu_accel_z
  );

  imu_g_force = mag / 9.80665f;

  // Tilt from vertical. fabs(z) keeps upright/inverted from exploding.
  const float safe_mag = fmaxf(mag, 0.001f);
  float ratio = fabsf(imu_accel_z) / safe_mag;
  ratio = constrain(ratio, 0.0f, 1.0f);

  imu_tilt_deg = acosf(ratio) * 180.0f / PI;
}

// ============================================================
//  GPS — PA1010D
// ============================================================

void sensors_init_gps() {
  sensors_init_i2c_once();

  gps_present     = false;
  gps_fix         = false;
  gps_fix_quality = 0;
  gps_sats        = 0;
  gps_hdop        = 99.9f;

  gps_sensor.begin(GPS_I2C_ADDR);

  // RMC + GGA gives fix quality, sats, HDOP, lat/lon, and altitude.
  gps_sensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // GPS telemetry is not timing-critical.
  gps_sensor.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(200);

  if (gps_sensor.available() > 0) {
    gps_present = true;
    Serial.println("PA1010D GPS OK");
    return;
  }

  // Second chance after boot settling.
  delay(800);

  if (gps_sensor.available() > 0) {
    gps_present = true;
    Serial.println("PA1010D GPS OK delayed");
  } else {
    gps_present = false;
    Serial.println("WARNING: PA1010D GPS not found — continuing without GPS.");
  }
}

void sensors_read_gps() {
  if (!gps_present) return;

  // Drain a bounded number of bytes so GPS cannot dominate the loop.
  for (int i = 0; i < 32; i++) {
    gps_sensor.read();
  }

  if (!gps_sensor.newNMEAreceived()) return;

  if (!gps_sensor.parse(gps_sensor.lastNMEA())) return;

  gps_fix_quality = gps_sensor.fixquality;
  gps_fix         = (gps_fix_quality > 0);

  if (!gps_fix) {
    gps_sats = gps_sensor.satellites;
    gps_hdop = gps_sensor.HDOP;
    return;
  }

  gps_lat = gps_sensor.latitudeDegrees;
  if (gps_sensor.lat == 'S') gps_lat = -gps_lat;

  gps_lon = gps_sensor.longitudeDegrees;
  if (gps_sensor.lon == 'W') gps_lon = -gps_lon;

  gps_alt_m = gps_sensor.altitude;
  gps_sats  = gps_sensor.satellites;
  gps_hdop  = gps_sensor.HDOP;
  gps_hour         = gps_sensor.hour;

  gps_minute       = gps_sensor.minute;
  gps_second       = gps_sensor.seconds;
  gps_milliseconds = gps_sensor.milliseconds;
}