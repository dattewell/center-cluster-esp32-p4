#include "TinyGPS++.h"
#include "gps_wrapper.h"

static TinyGPSPlus gps;

extern "C" {
// Thin C ABI around TinyGPS++ so the C application code can consume the C++
// parser without knowing about C++ types or name mangling.

void gps_encode_char(char c){
    // Feed one raw NMEA byte from the UART stream into TinyGPS++.
    gps.encode(c);
}

/* =======================
   SPEED
   ======================= */

int gps_speed_updated(void){
    return gps.speed.isUpdated();
}

float gps_get_speed_mph(void){
    if (gps.speed.isValid())
        return gps.speed.mph();
    return 0.0f;
}

/* =======================
   FIX STATUS
   ======================= */

bool gps_has_fix(void){
    // Treat stale locations as no fix so the UI does not keep showing old data
    // after GPS reception drops out.
    return gps.location.isValid() &&
           gps.location.age() < 2000;
}

int gps_sats_used(void){
    if (gps.satellites.isValid())
        return gps.satellites.value();
    return 0;
}

float gps_hdop(void){
    if (gps.hdop.isValid())
        return gps.hdop.hdop();
    return -1.0f;
}

/* =======================
   LOCATION ACCESS
   ======================= */

bool gps_location_updated(void){
    return gps.location.isUpdated();
}

bool gps_location_valid(void){
    return gps.location.isValid();
}

double gps_get_lat(void){
    return gps.location.lat();
}

double gps_get_lon(void){
    return gps.location.lng();
}
float gps_get_course_deg(void){
    if (gps.course.isValid())
        return gps.course.deg();
    return 0.0f;
}

/* =======================
   TIME (UTC)
   ======================= */

bool gps_time_valid(void){
    return gps.time.isValid();
}

int gps_get_hour(void){
    return gps.time.isValid() ? gps.time.hour() : -1;
}

int gps_get_minute(void){
    return gps.time.isValid() ? gps.time.minute() : -1;
}

int gps_get_second(void){
    return gps.time.isValid() ? gps.time.second() : -1;
}

bool gps_date_valid(void){
    return gps.date.isValid();
}

int gps_get_year(void){
    return gps.date.isValid() ? gps.date.year() : -1;
}

int gps_get_month(void){
    return gps.date.isValid() ? gps.date.month() : -1;
}

int gps_get_day(void){
    return gps.date.isValid() ? gps.date.day() : -1;
}

/* =======================
   DISTANCE
   ======================= */

double gps_distance_between(double lat1,
                            double lon1,
                            double lat2,
                            double lon2){
    // Reuse TinyGPS++'s distance helper for odometer increments between fixes.
    return TinyGPSPlus::distanceBetween(
        lat1,
        lon1,
        lat2,
        lon2
    );
}

}
