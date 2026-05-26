#include "include/display_batch.h"

#include "include/globals.h"
#include "include/sensors.h"
#include "include/runtime.h"

// ============================================================
void flushDisplayBatch() {
  // diag struct already populated by snapDiagnostics() — no need to re-read here

  if (tare_requested && buf_count > 0) {
    float sum = 0;
    for (int i = 0; i < buf_count; i++) sum += pressureToAltitudeFeet(buf_pressure[i]);
    tare_baseline_ft = sum / buf_count;
    tare_requested   = false;
    logts(); Serial.printf("[TARE] Baseline set: %.3f ft\n", tare_baseline_ft);
  }

  if (buf_count == 0) { buf_count = 0; return; }

  String arr_alt_ft = "[", arr_alt_tare = "[", arr_pressure = "[", arr_temp = "[";
  // Reserve ~12 chars per sample per array to avoid repeated reallocation
  arr_alt_ft.reserve(buf_count * 12);
  arr_alt_tare.reserve(buf_count * 12);
  arr_pressure.reserve(buf_count * 12);
  arr_temp.reserve(buf_count * 10);
  float sum_alt = 0;

  for (int i = 0; i < buf_count; i++) {
    float alt_ft   = pressureToAltitudeFeet(buf_pressure[i]);
    float alt_tare = alt_ft - tare_baseline_ft;
    sum_alt += alt_ft;
    arr_alt_ft   += String(alt_ft,             3);
    arr_alt_tare += String(alt_tare,           3);
    arr_pressure += String(buf_pressure[i],    4);
    arr_temp     += String(buf_temperature[i], 3);
    if (i < buf_count - 1) {
      arr_alt_ft += ","; arr_alt_tare += ",";
      arr_pressure += ","; arr_temp += ",";
    }
  }
  arr_alt_ft += "]"; arr_alt_tare += "]";
  arr_pressure += "]"; arr_temp += "]";

  float mean_alt_ft = sum_alt / buf_count;

  String& wb = (active_buf == 0) ? batch_json_b : batch_json_a;
  wb.reserve(600);   // pre-size to avoid reallocation during concatenation
  wb  = "{";
  wb += "\"ready\":true,";
  wb += "\"unit_id\":"           + String(flight.unit_id)                      + ",";
  wb += "\"state\":"             + String((int)flight.state)                   + ",";
  wb += "\"state_name\":\""      + String(stateNames[flight.state])            + "\",";
  wb += "\"sample_count\":"      + String(buf_count)                           + ",";
  wb += "\"tare_baseline_ft\":"  + String(tare_baseline_ft, 3)                 + ",";
  wb += "\"mean_alt_ft\":"       + String(mean_alt_ft, 3)                      + ",";
  wb += "\"mean_alt_tared_ft\":" + String(mean_alt_ft - tare_baseline_ft, 3)   + ",";
  wb += "\"peak_alt_ft\":"       + String(flight.peak_alt_ft, 1)               + ",";
  wb += "\"launch_height_ft\":"  + String(flight.launch_height_ft, 1)          + ",";
  wb += "\"flight_duration_s\":" + String(flight.flight_duration_ms / 1000.0f, 1) + ",";
  wb += "\"alt_ft\":"            + arr_alt_ft                                  + ",";
  wb += "\"alt_tared\":"         + arr_alt_tare                                + ",";
  wb += "\"pressure\":"          + arr_pressure                                + ",";
  wb += "\"temp\":"              + arr_temp                                    + ",";
  wb += "\"rssi_dbm\":"          + String(diag.rssi_dbm)                       + ",";
  wb += "\"cpu_load_pct\":"      + String(diag.cpu_load_pct, 1)                + ",";
  wb += "\"loop_avg_us\":"       + String(diag.loop_avg_us, 0)                 + ",";
  wb += "\"loop_max_us\":"       + String(diag.loop_max_us, 0)                 + ",";
  wb += "\"free_heap_b\":"       + String(diag.free_heap)                      + ",";
  // IMU — LSM6DSO32
  wb += "\"imu_present\":"       + String(imu_present ? "true" : "false")      + ",";
  wb += "\"accel_x\":"           + String(imu.accel_x, 3)                      + ",";
  wb += "\"accel_y\":"           + String(imu.accel_y, 3)                      + ",";
  wb += "\"accel_z\":"           + String(imu.accel_z, 3)                      + ",";
  wb += "\"gyro_x\":"            + String(imu.gyro_x,  4)                      + ",";
  wb += "\"gyro_y\":"            + String(imu.gyro_y,  4)                      + ",";
  wb += "\"gyro_z\":"            + String(imu.gyro_z,  4)                      + ",";
  wb += "\"g_force\":"           + String(imu.g_force,  3)                     + ",";
  wb += "\"tilt_deg\":"          + String(imu.tilt_deg, 2);
  wb += "}";

  active_buf = (active_buf == 0) ? 1 : 0;
  buf_count  = 0;
}
