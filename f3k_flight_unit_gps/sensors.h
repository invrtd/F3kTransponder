#pragma once
#include <Arduino.h>
#include "conf.h"

// ============================================================
//  Sensor Abstraction Layer
//  
//  Encapsulates DPS310 barometer, LSM6DSO32 IMU, and PA1010D GPS.
//  All sensor state exposed via global externs.
// ============================================================

// -- Barometer (DPS310) -------
float pressureToAltitudeFeet(float pressure_hpa);
void  sensors_init_barometer();
void  sensors_read_barometer(float& alt_ft, float& pressure_hpa, float& temp_c);
extern float dps_temp_c;       // live temperature from DPS310

// -- IMU (LSM6DSO32) ----------
void  sensors_init_imu();
void  sensors_read_imu();
extern float imu_accel_x, imu_accel_y, imu_accel_z;  // m/s²
extern float imu_gyro_x,  imu_gyro_y,  imu_gyro_z;   // rad/s
extern float imu_g_force;                             // magnitude in G
extern float imu_tilt_deg;                            // from vertical

// -- GPS (PA1010D) ------------
void  sensors_init_gps();
void  sensors_read_gps();
extern float gps_lat, gps_lon;        // decimal degrees
extern float gps_alt_m;                // MSL altitude
extern uint8_t  gps_sats;              // satellites used
extern float gps_hdop;                 // horizontal dilution of precision
extern uint8_t  gps_fix_quality;       // 0=none 1=GPS 2=DGPS 6=est
extern bool     gps_fix;               // true when fix_quality > 0

// -- Sensor presence flags -----
extern bool dps_present;               // DPS310 initialized successfully
extern bool imu_present;               // IMU initialized successfully
extern bool gps_present;               // GPS initialized successfully

extern uint8_t  gps_hour;
extern uint8_t  gps_minute;
extern uint8_t  gps_second;
extern uint16_t gps_milliseconds;