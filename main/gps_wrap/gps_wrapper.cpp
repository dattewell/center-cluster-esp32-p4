#include <string.h>
#include "esp_log.h"
#include "TinyGPS++.h"
#include "gps_wrapper.h"

static const char *TAG_GPS = "GPS_SENSOR";

static TinyGPSPlus gps;

// Minimal streaming UBX frame parser, run alongside the NMEA parser on the
// same raw byte stream. It only exists to catch a UBX-MON-VER (class 0x0A,
// id 0x04) reply to gps_ubx_query_chip_info() and log its version/extension
// strings as clean text instead of a raw binary dump; every other UBX
// message class is parsed just enough to validate and skip. Harmless to run
// on non-UBX streams - it just never finds a 0xB5 0x62 sync and does nothing.
typedef enum {
    UBX_WAIT_SYNC1,
    UBX_WAIT_SYNC2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN_LO,
    UBX_LEN_HI,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B,
} ubx_parse_state_t;

#define UBX_MAX_PAYLOAD 220  // fixed 40-byte MON-VER body + up to 6 extension strings

static ubx_parse_state_t ubx_state = UBX_WAIT_SYNC1;
static uint8_t  ubx_class, ubx_id, ubx_ck_a, ubx_ck_b;
static uint16_t ubx_len, ubx_payload_idx;
static uint8_t  ubx_payload[UBX_MAX_PAYLOAD];

// UBX-MON-VER payload: 30-byte swVersion, 10-byte hwVersion, then zero or
// more 30-byte extension strings - each field null-padded/terminated ASCII.
static void ubx_log_mon_ver(const uint8_t *payload, uint16_t len) {
    if (len < 40) {
        ESP_LOGW(TAG_GPS, "UBX-MON-VER payload too short (%d bytes)", len);
        return;
    }
    char sw[31] = {0};
    char hw[11] = {0};
    memcpy(sw, payload, 30);
    memcpy(hw, payload + 30, 10);
    ESP_LOGI(TAG_GPS, "GPS chip swVersion: %s", sw);
    ESP_LOGI(TAG_GPS, "GPS chip hwVersion: %s", hw);

    for (uint16_t off = 40; (uint16_t)(off + 30) <= len; off += 30) {
        char ext[31] = {0};
        memcpy(ext, payload + off, 30);
        ESP_LOGI(TAG_GPS, "GPS chip extension: %s", ext);
    }
}

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
   UBX (u-blox binary protocol)
   ======================= */

void gps_ubx_feed_byte(uint8_t b) {
    switch (ubx_state) {
        case UBX_WAIT_SYNC1:
            ubx_state = (b == 0xB5) ? UBX_WAIT_SYNC2 : UBX_WAIT_SYNC1;
            break;
        case UBX_WAIT_SYNC2:
            ubx_state = (b == 0x62) ? UBX_CLASS : UBX_WAIT_SYNC1;
            break;
        case UBX_CLASS:
            ubx_class = b;
            ubx_state = UBX_ID;
            break;
        case UBX_ID:
            ubx_id = b;
            ubx_state = UBX_LEN_LO;
            break;
        case UBX_LEN_LO:
            ubx_len = b;
            ubx_state = UBX_LEN_HI;
            break;
        case UBX_LEN_HI:
            ubx_len |= ((uint16_t)b << 8);
            ubx_payload_idx = 0;
            if (ubx_len == 0) {
                ubx_state = UBX_CK_A;
            } else if (ubx_len > UBX_MAX_PAYLOAD) {
                // Bigger than anything we care about (not a MON-VER reply) - resync.
                ubx_state = UBX_WAIT_SYNC1;
            } else {
                ubx_state = UBX_PAYLOAD;
            }
            break;
        case UBX_PAYLOAD:
            ubx_payload[ubx_payload_idx++] = b;
            if (ubx_payload_idx >= ubx_len) {
                ubx_state = UBX_CK_A;
            }
            break;
        case UBX_CK_A:
            ubx_ck_a = b;
            ubx_state = UBX_CK_B;
            break;
        case UBX_CK_B:
            ubx_ck_b = b;
            {
                uint8_t calc_a = 0, calc_b = 0;
                uint8_t hdr[4] = {ubx_class, ubx_id, (uint8_t)(ubx_len & 0xFF), (uint8_t)(ubx_len >> 8)};
                for (int i = 0; i < 4; i++) { calc_a += hdr[i]; calc_b += calc_a; }
                for (int i = 0; i < ubx_len; i++) { calc_a += ubx_payload[i]; calc_b += calc_a; }

                if (calc_a == ubx_ck_a && calc_b == ubx_ck_b) {
                    if (ubx_class == 0x0A && ubx_id == 0x04) {
                        ubx_log_mon_ver(ubx_payload, ubx_len);
                    }
                } else {
                    ESP_LOGW(TAG_GPS, "UBX checksum mismatch, class=0x%02X id=0x%02X", ubx_class, ubx_id);
                }
            }
            ubx_state = UBX_WAIT_SYNC1;
            break;
    }
}

void gps_ubx_query_chip_info(uart_port_t uart_num) {
    // Sends a UBX-MON-VER poll requesting the module's software/hardware
    // version and identifying extension strings. This is a fixed, well-known
    // byte sequence (class 0x0A, id 0x04, empty payload, with its checksum),
    // not something computed at runtime, since the payload never changes.
    static const uint8_t ubx_mon_ver_poll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    uart_write_bytes(uart_num, ubx_mon_ver_poll, sizeof(ubx_mon_ver_poll));
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
