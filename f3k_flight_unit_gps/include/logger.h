#pragma once

#include <Arduino.h>

// ============================================================
// logger.h
//
// Minimal extraction header for the existing LittleFS logging
// functions moved out of f3k_flight_unit_gps.ino.
//
// Function names are intentionally unchanged so current .ino call
// sites do not need to be edited.
// ============================================================

// Deletes at most one old log/summary per call when pruning is pending.
void pruneLogsIfNeeded();

// Opens the per-window CSV log when a scoring window starts.
void openWindowLog();

// Writes one sensor sample row to the active window CSV.
void logSample(float alt_ft, float pressure_hpa, float temp_c);

// Writes the summary_NNN.csv score file after a window closes.
void writeSummaryLog();

// Flushes/closes the active CSV log and schedules announcements/pruning.
void closeWindowLog();