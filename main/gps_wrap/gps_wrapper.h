#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// C-facing wrapper around TinyGPS++.  The parser itself is C++, but the rest of
// the firmware can stay in C by using this API.

/* =======================
   FEED GPS
   ======================= */

void gps_encode_char(char c);

/* =======================
   SPEED
   ======================= */

// True when TinyGPS++ has parsed a new speed field since the last check.
int   gps_speed_updated(void);

// Vehicle speed in MPH, or 0.0f when the speed field is invalid.
float gps_get_speed_mph(void);

/* =======================
   FIX STATUS
   ======================= */

// A fix is valid only while the location is both valid and recent.
bool  gps_has_fix(void);

// Satellites used by the current fix, or 0 when unknown.
int   gps_sats_used(void);

// Horizontal dilution of precision; lower is better, -1 means unavailable.
float gps_hdop(void);

/* =======================
   LOCATION ACCESS
   ======================= */

// True when a new location was parsed from the stream.
bool   gps_location_updated(void);

// Raw TinyGPS++ validity flag for the current location.
bool   gps_location_valid(void);

double gps_get_lat(void);
double gps_get_lon(void);

// Course over ground in degrees, or 0.0f when invalid.
float gps_get_course_deg(void);

/* =======================
   DISTANCE
   ======================= */

// Great-circle distance in meters between two WGS84 coordinates.
double gps_distance_between(double lat1,
                            double lon1,
                            double lat2,
                            double lon2);

#ifdef __cplusplus
}
#endif
