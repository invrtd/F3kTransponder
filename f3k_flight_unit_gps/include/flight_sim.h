#pragma once

#include <Arduino.h>

// ============================================================
//  Flight simulation helpers
//
//  Debug/test-only altitude generation used when tilt_mode=true.
//  This keeps the production loop from carrying the large autonomous
//  task-aware simulation block inline.
// ============================================================

// Updates alt_ft in-place for DEBUG_TILT_MODE 1 or 2.
// Caller remains responsible for writing flight.altitude_ft, buffering
// pressure/temp samples, and calling updateStateMachine(alt_ft).
void flightSimUpdateAltitude(float& alt_ft, unsigned long now);

// Resets static simulator state. Useful when changing modes or starting
// controlled tests from a known simulator baseline.
void flightSimReset();
