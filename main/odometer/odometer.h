#pragma once
#include <stdint.h>
#include <stdbool.h>

// Open NVS and restore the newest valid odometer record.
void odometer_init(void);

// Add traveled distance to the in-memory odometer.  Persistence is handled by
// odometer_periodic_save() or odometer_force_save().
void odometer_add_meters(uint32_t meters);

// Save only after the configured distance interval to limit flash wear.
void odometer_periodic_save(void);

// Immediately commit the current odometer value to NVS.
void odometer_force_save(void);

// Raw total distance in meters.
uint64_t odometer_get_meters(void);

// Total distance converted to miles for display.
int odometer_get_miles(void);
