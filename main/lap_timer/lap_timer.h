#pragma once
#include <stdint.h>
#include <stdbool.h>

// Feed the lap timer with fresh GPS data.  A crossing only counts when the fix
// is valid, the vehicle is moving, and the position is inside the finish radius.
void lap_timer_update(double lat, double lon, float speed_mph, bool has_fix);

// Elapsed time for the in-progress lap, or 0 if the session has not started.
uint64_t lap_timer_get_current_us(void);

// Most recently completed lap.
uint64_t lap_timer_get_last_lap_us(void);

// Best completed lap in the current session.
uint64_t lap_timer_get_best_us(void);

// Predictive delta against the reference lap.  Negative means ahead.
int32_t  lap_timer_get_delta_us(void);

// Clear all session timing state.
void lap_timer_reset_session(void);
