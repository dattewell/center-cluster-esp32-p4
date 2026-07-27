#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "bsp_board_extra.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "ui.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "gps_wrapper.h"
#include "odometer/odometer.h"


//-----Pin Assignment---------//

//GPS RX - GPIO 35
#define GPS_RX_PIN        35
#define GPS_UART_NUM      UART_NUM_3
// GPS TX (module RX line) - GPIO 36. Chosen because it's adjacent to
// GPS_RX_PIN and likely broken out on the same header, but not verified
// against the board's silkscreen - confirm before wiring.
#define GPS_TX_PIN        36

//Water Temp - GPIO 51
#define WATER_TEMP_ADC_CHANNEL ADC_CHANNEL_2

//--------------------------//

//-------------RPM (coil/distributor via H11L1 optoisolator) - GPIO 47-------------//
#define RPM_PULSE_GPIO       47
#define RPM_PULSES_PER_REV   2      // 4-cyl, 4-stroke: cylinders / 2
#define RPM_SAMPLE_MS        250    // how often the pulse count is turned into an RPM figure

// RPM dial angle range, calibrated against the dial artwork.
#define RPM_NEEDLE_ANGLE_0RPM     (695)
#define RPM_NEEDLE_ANGLE_MAX_RPM  (2895)
#define RPM_DIAL_MAX_RPM          6000
//----------------------------------------------------------------------------------//

//-------------AFR power gating (BTS7004 on GPIO4, TXB0104D OE on GPIO28)-------------//
#define AFR_ENABLE_GPIO      4      // BTS7004-1EPP IN - powers the AFR sensor
#define AFR_OE_GPIO          28     // TXB0104D OE - enables the AFR UART level shifter
#define RPM_MIN_RUNNING      500    // must be at/above this to start qualifying AFR power-on
#define RPM_QUALIFY_MS       30000  // sustained duration above RPM_MIN_RUNNING before AFR powers on
#define RPM_STOPPED_BELOW    100    // below this is treated as "engine stopped", not just idling low
#define RPM_STOPPED_HOLD_MS  5000   // sustained duration below RPM_STOPPED_BELOW before AFR powers back off
//-------------------------------------------------------------------------------------//

//-------------AFR UART (14point7 Spartan) - GPIO 20/21-------------//
// TX/RX assignment is a best guess (20=TX, 21=RX) pending hardware confirmation -
// swap AFR_TX_PIN/AFR_RX_PIN if the device doesn't respond once the parser is added.
#define AFR_UART_NUM   UART_NUM_1
#define AFR_TX_PIN     20
#define AFR_RX_PIN     21
#define AFR_BAUD_RATE  9600
//--------------------------------------------------------------------//

#define ENABLE_LOGS true
#define ADC_UPDATE_PERIOD_MS 10
#define FILTER_SAMPLES_DEFAULT 8


//------------GPS-----------//         
#define GPS_BAUD_RATE     38400
#define GPS_BUF_SIZE          1024
#define GPS_MIN_VALID_MPH 3.0f
//--------------------------//

//--------UPDATE/REFRESH_DELAYS------//
#define TEMP_UPDATE_DELAY 250 // 4 updates a second
//-----------------------------------//

//-------------TEMP-------------//
// Sender node: SENSOR_SUPPLY --[R_PULLUP]-- (node, straight into the ADC pin,
// no external divider) --[sender]-- GND, with a 100nF cap on the node for
// noise filtering.
#define R_PULLUP       10000.0f
#define SENSOR_SUPPLY  3.3f
#define ADC_WIDTH ADC_BITWIDTH_12
#define ADC_ATTEN ADC_ATTEN_DB_12

// PROVISIONAL calibration for the actual installed sender (the original table
// here was for a generic/different sender and read ~100C+ high across the
// board). Fitted as an NTC Beta-model curve (ln(R) = B/T_kelvin + A) via
// least-squares regression through three real measured points spanning the
// practical range: ice water (0C, ~2217 ohm avg of 5 samples), room temp
// (24C, ~756 ohm), and a thermometer-confirmed 76C reading (~114.6 ohm avg
// of 5 samples), giving B ~= 3763, A ~= -6.05 - this tracks all three points
// within ~2.2%. Refine further with more measured points if accuracy in a
// particular range matters.
#define TEMP_SENSOR_TABLE_SIZE 9
const float tempC[] = {
    0, 20, 40, 60, 80, 100, 120, 140, 150
};

const float sensorR[] = {
    2265, 885, 390, 190, 100, 57, 34, 21, 17
};

// A disconnected sender leaves the pull-up with nowhere to sink current, so
// the divider node floats up toward SENSOR_SUPPLY - well past the coldest
// calibrated point (sensorR[0], 0C - only ~2265 ohm now, versus the hundreds
// of thousands to millions of ohms actually observed when disconnected).
// read_temp_resistance()'s formula actually swings negative right as the
// divider node crosses SENSOR_SUPPLY, so treat both "unreasonably high" and
// negative resistance as "not connected" rather than silently clamping to a
// temperature (which, for the negative case, would otherwise misreport as
// scalding hot - the opposite of reality).
#define TEMP_SENSOR_DISCONNECTED_R 5000.0f
//------------------------------//

// Matches the initial lv_img_set_angle() set on ui_SpeedoNeedle in ui_MainSpeedo.c (0 mph rest position).
#define SPEEDO_NEEDLE_REST_ANGLE   (-670)
// How far the sweep rotates from rest before returning; sign/magnitude tuned by eye
// against the dial artwork to land near the 80 mph mark, since this needle has no
// existing mph->angle calibration to derive it from.
#define SPEEDO_SWEEP_ANGLE_DELTA   (2455)
// Needle angle at 80 mph (the top of the dial's marked scale) for live gauge
// readings - distinct from SPEEDO_SWEEP_ANGLE_DELTA above, which deliberately
// overshoots past this for the boot-time sweep flourish.
#define SPEEDO_NEEDLE_ANGLE_80MPH  (2110)

//-------------LOGGING------------//
static const char *TAG_TEMP = "TEMP_SENSOR";
static const char *TAG_GPS = "GPS_SENSOR";
static const char *TAG_AFR = "AFR_SENSOR";
//--------------------------------//

//------------DATA_SENT_OUT---------//
// Internal, display-friendly gauge model.  Every producer path writes these
// engineering-unit values first, then the UART packet layer scales them to the
// fixed-point protocol expected by the remote gauge display.
typedef struct {
    float water_temp_c;
    bool  water_temp_valid;  // false when the sender reads as disconnected - see TEMP_SENSOR_DISCONNECTED_R
    float afr;
    float afr_temp_c;   // wideband sensor cell temperature - drives update_afr_status()
    float speed_mph;
    int   rpm;
} gauge_data_t;

static gauge_data_t g_gauge_data;

// Set by update_afr_power_gate(), read by afr_task() - true once the AFR
// sensor's power (BTS7004) and UART level shifter (TXB0104D) are enabled.
static volatile bool g_afr_enabled = false;


//---------------------------------------//

typedef struct {
    lv_obj_t *value_label;
    lv_obj_t *mirror_label; // optional duplicate label on another screen showing the same text, or NULL
    lv_obj_t *needle;
    const char *log_tag;
    int16_t angle_min;
    int16_t angle_max;
    int decimals;   // decimal places shown in the value label
} gauge_channel_t;

// Updates a gauge's value label(s) and needle angle together, but only if the
// displayed value actually changed (avoids redundant redraws every tick).
static void update_gauge_channel(const gauge_channel_t *ch, float new_value, int16_t angle){
    char buf[12];
    snprintf(buf, sizeof(buf), "%.*f", ch->decimals, new_value);

    // Only update text if changed
    const char *old_text = lv_label_get_text(ch->value_label);
    if (strcmp(old_text, buf) == 0) return;

    lv_label_set_text(ch->value_label, buf);
    if (ch->mirror_label) lv_label_set_text(ch->mirror_label, buf);

    if (angle > ch->angle_max) angle = ch->angle_max;
    if (angle < ch->angle_min) angle = ch->angle_min;
    ESP_LOGI(ch->log_tag, "value=%.1f angle=%d", new_value, angle);
    lv_img_set_angle(ch->needle, angle);
}

//-------------------------AFR sensor (LSU ADV) temperature status-------------------------//
// Thresholds per the LSU ADV element's operating spec: below 350C the
// controller is still waiting out the condensation phase (no valid data);
// 350-730C the heater is ramping; 730-820C is the element's rated operating
// range (AFR valid); 820-900C the heater has backed off and exhaust heat
// alone is pushing the cell hotter than rated, so AFR is still shown but
// flagged; above 900C is an alarm condition and AFR should be treated as
// unreliable.
#define AFR_TEMP_HEATING_C     350.0f
#define AFR_TEMP_OPERATING_C   730.0f
#define AFR_TEMP_WARNING_HOT_C 820.0f
#define AFR_TEMP_TOO_HOT_C     900.0f
#define AFR_ALARM_BLINK_MS     500

#define AFR_STATUS_COLOR_GOOD    0x14A604
#define AFR_STATUS_COLOR_BAD     0xF80303
#define AFR_STATUS_COLOR_WARNING 0xE0954B

typedef enum {
    AFR_TEMP_STATE_COLD,
    AFR_TEMP_STATE_HEATING,
    AFR_TEMP_STATE_OPERATING,
    AFR_TEMP_STATE_WARNING_HOT,
    AFR_TEMP_STATE_TOO_HOT,
} afr_temp_state_t;

static afr_temp_state_t afr_classify_temp(float temp_c) {
    if (temp_c < AFR_TEMP_HEATING_C)     return AFR_TEMP_STATE_COLD;
    if (temp_c < AFR_TEMP_OPERATING_C)   return AFR_TEMP_STATE_HEATING;
    if (temp_c < AFR_TEMP_WARNING_HOT_C) return AFR_TEMP_STATE_OPERATING;
    if (temp_c < AFR_TEMP_TOO_HOT_C)     return AFR_TEMP_STATE_WARNING_HOT;
    return AFR_TEMP_STATE_TOO_HOT;
}

static const char *afr_temp_state_word(afr_temp_state_t state) {
    switch (state) {
        case AFR_TEMP_STATE_COLD:        return "Cold";
        case AFR_TEMP_STATE_HEATING:      return "Heating";
        case AFR_TEMP_STATE_OPERATING:    return "Operating";
        case AFR_TEMP_STATE_WARNING_HOT:  return "Warning Hot";
        default:                          return "Too Hot";
    }
}

static uint32_t afr_temp_state_color(afr_temp_state_t state) {
    switch (state) {
        case AFR_TEMP_STATE_OPERATING:   return AFR_STATUS_COLOR_GOOD;
        case AFR_TEMP_STATE_WARNING_HOT: return AFR_STATUS_COLOR_WARNING;
        default:                         return AFR_STATUS_COLOR_BAD;
    }
}

// Converts the latest AFR reading to the AFR needle's angle range and refreshes
// the AFR label/needle (mirrored onto ui_AfrV2 on the RPM/AFR screen).
// Shows "--" instead while the AFR sensor isn't powered on at all (g_afr_enabled
// false - no sustained RPM, see update_afr_power_gate()), or blanks both labels
// while the sensor is powered but in the TOO_HOT alarm state (same condition
// that lights up ui_AFRStatusBad), since the reading is unreliable either way.
static void update_afr(float new_value, float afr_temp_c){
    if (!g_afr_enabled) {
        if (strcmp(lv_label_get_text(ui_AfrV), "--") != 0) {
            lv_label_set_text(ui_AfrV, "--");
            lv_label_set_text(ui_AfrV2, "--");
        }
        lv_obj_add_flag(ui_AfrD, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (afr_classify_temp(afr_temp_c) == AFR_TEMP_STATE_TOO_HOT) {
        if (strcmp(lv_label_get_text(ui_AfrV), "") != 0) {
            lv_label_set_text(ui_AfrV, "");
            lv_label_set_text(ui_AfrV2, "");
        }
        lv_obj_add_flag(ui_AfrD, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(ui_AfrD, LV_OBJ_FLAG_HIDDEN);
    int16_t afr_angle = (int16_t)((new_value - 10.0) * (1640 - 845) / 8 + 845);
    gauge_channel_t ch = { .value_label = ui_AfrV, .mirror_label = ui_AfrV2, .needle = ui_AfrD, .log_tag = "AFR", .angle_min = 845, .angle_max = 1640, .decimals = 1 };
    update_gauge_channel(&ch, new_value, afr_angle);
}

// Converts the latest water temp reading to the temp needle's angle range and
// refreshes the temp label/needle.
static void update_water_temp(float new_value, bool valid){
    // Starts true so the very first call (before a reading has been taken)
    // forces an initial "--", clearing SquareLine's placeholder text rather
    // than waiting for the sender to go valid then invalid once.
    static bool last_valid = true;

    if (!valid) {
        if (last_valid) {
            lv_label_set_text(ui_TempV, "--");
            last_valid = false;
        }
        lv_obj_add_flag(ui_TempD, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    last_valid = true;

    lv_obj_clear_flag(ui_TempD, LV_OBJ_FLAG_HIDDEN);
    int16_t temp_angle = (int16_t)(2747 - (new_value - 40.0) * (2747 - 1940) / 80);
    gauge_channel_t ch = { .value_label = ui_TempV, .mirror_label = NULL, .needle = ui_TempD, .log_tag = "TEMP", .angle_min = 1940, .angle_max = 2747, .decimals = 0 };
    update_gauge_channel(&ch, new_value, temp_angle);
}

// Converts the latest RPM reading to the RPM dial's angle range and refreshes
// the RPM label/needle.
static void update_rpm(int rpm){
    int16_t rpm_angle = (int16_t)(RPM_NEEDLE_ANGLE_0RPM +
        (int32_t)rpm * (RPM_NEEDLE_ANGLE_MAX_RPM - RPM_NEEDLE_ANGLE_0RPM) / RPM_DIAL_MAX_RPM);
    gauge_channel_t ch = { .value_label = ui_RpmV, .mirror_label = NULL, .needle = ui_RpmD, .log_tag = "RPM", .angle_min = RPM_NEEDLE_ANGLE_0RPM, .angle_max = RPM_NEEDLE_ANGLE_MAX_RPM, .decimals = 0 };
    update_gauge_channel(&ch, rpm, rpm_angle);
}

// Drives the three AFR sensor-temperature widgets from one classification:
//  - ui_AFRStatusBad: "Cold"/"Heating" (red) below the rated range, blinking
//    "Too Hot" (red) above it, blank in between (Operating/Warning Hot).
//  - ui_AFRStatusGood: "Operating" (green) in the rated range, "Warning Hot"
//    (amber) just above it (AFR still valid but flagged), blank otherwise.
//  - ui_AFRStatus: always-on combined "<state>\n<temp>C" readout, one word
//    per band (Cold/Heating/Operating/Warning Hot/Too Hot) colored to match.
// All three are blanked outright while the AFR sensor isn't powered on at all
// (g_afr_enabled false - no sustained RPM, see update_afr_power_gate()), since
// there's no real temperature reading to classify yet.
static void update_afr_status(float temp_c) {
    static afr_temp_state_t last_bad_state = (afr_temp_state_t)-1;
    static afr_temp_state_t last_good_state = (afr_temp_state_t)-1;
    static bool blink_on = false;
    static int64_t last_blink_ms = 0;
    static char last_status_text[24] = "";
    // Starts true so the very first call (with g_afr_enabled still false at
    // boot) forces an initial blank, clearing SquareLine's design-time
    // placeholder text ("Heating\n360C" etc.) instead of leaving it on-screen
    // until the sensor happens to cycle enabled->disabled once.
    static bool was_enabled = true;

    if (!g_afr_enabled) {
        if (was_enabled) {
            lv_label_set_text(ui_AFRStatusGood, "");
            lv_label_set_text(ui_AFRStatusBad, "");
            lv_label_set_text(ui_AFRStatus, "");
            last_bad_state = (afr_temp_state_t)-1;
            last_good_state = (afr_temp_state_t)-1;
            last_status_text[0] = '\0';
            was_enabled = false;
        }
        return;
    }
    was_enabled = true;

    afr_temp_state_t state = afr_classify_temp(temp_c);

    // ui_AFRStatusBad: Cold/Heating shown solid, Too Hot blinks as an alarm,
    // Operating/Warning Hot leave it blank.
    if (state == AFR_TEMP_STATE_COLD || state == AFR_TEMP_STATE_HEATING) {
        if (state != last_bad_state) {
            lv_label_set_text(ui_AFRStatusBad, afr_temp_state_word(state));
        }
    } else if (state == AFR_TEMP_STATE_TOO_HOT) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_blink_ms >= AFR_ALARM_BLINK_MS) {
            last_blink_ms = now_ms;
            blink_on = !blink_on;
            lv_label_set_text(ui_AFRStatusBad, blink_on ? "Too Hot" : "");
        }
    } else if (state != last_bad_state) {
        lv_label_set_text(ui_AFRStatusBad, "");
    }
    last_bad_state = state;

    // ui_AFRStatusGood: Operating/Warning Hot shown, blank otherwise.
    if (state != last_good_state) {
        if (state == AFR_TEMP_STATE_OPERATING || state == AFR_TEMP_STATE_WARNING_HOT) {
            lv_label_set_text(ui_AFRStatusGood, afr_temp_state_word(state));
            lv_obj_set_style_text_color(ui_AFRStatusGood, lv_color_hex(afr_temp_state_color(state)), LV_PART_MAIN | LV_STATE_DEFAULT);
        } else {
            lv_label_set_text(ui_AFRStatusGood, "");
        }
        last_good_state = state;
    }

    // ui_AFRStatus: always-on combined readout.
    char status_text[24];
    snprintf(status_text, sizeof(status_text), "%s\n%.0f\xC2\xB0" "C", afr_temp_state_word(state), temp_c);
    if (strcmp(status_text, last_status_text) != 0) {
        lv_label_set_text(ui_AFRStatus, status_text);
        lv_obj_set_style_text_color(ui_AFRStatus, lv_color_hex(afr_temp_state_color(state)), LV_PART_MAIN | LV_STATE_DEFAULT);
        strncpy(last_status_text, status_text, sizeof(last_status_text) - 1);
        last_status_text[sizeof(last_status_text) - 1] = '\0';
    }
}
//--------------------------------------------------------------------------------------------//

// Updates one odometer digit label, but only if it differs from what's
// currently shown.
static void update_odo_if_needed(lv_obj_t *label, int digit) {
    char new_value[2] = { (char)('0' + digit), '\0' };
    const char *old_text = lv_label_get_text(label);
    if (strcmp(old_text, new_value) != 0) {
        #if ENABLE_LOGS
            ESP_LOGI("ODOMETER", "old_text=%s new_value=%s", old_text, new_value);
        #endif
        lv_label_set_text(label, new_value);
    }
}

//-----------------------TEMP--------------------------//

float read_temp_resistance(int millivolts){
    // The ADC pin sits directly on the sender's pull-up node (no external
    // divider), so the calibrated reading is the divider voltage itself -
    // just solve the pull-up/sensor divider for thermistor resistance.
    float signal_voltage = millivolts / 1000.0f;

    float sensor_resistance = R_PULLUP *
        (signal_voltage / (SENSOR_SUPPLY - signal_voltage));

    return sensor_resistance;
}

//-----------------------------------------------------//

void update_odometer(int miles) {
    static int last_miles = -1;
    if (miles != last_miles) {
        last_miles = miles;
        #if ENABLE_LOGS
            ESP_LOGI("ODOMETER", "miles=%d", miles);
        #endif    // Loop through each of the 6 odometer labels and update them with the corresponding digit
        lv_obj_t *odo_labels[6] = {
            ui_Odometer1, ui_Odometer2, ui_Odometer3, ui_Odometer4, ui_Odometer5, ui_Odometer6
        };
        int divisor = 100000;  // hundred-thousands down to units
        for (int i = 0; i < 6; i++) {
            update_odo_if_needed(odo_labels[i], (miles / divisor) % 10);
            divisor /= 10;
        }
    }
}

// Refreshes the mph/km-h speed labels and needles. Shows "--" on both while
// GPS has no fix, since a stale/zero speed reading would otherwise look like
// a real value; and de-jitters near-zero GPS noise by treating anything
// below GPS_MIN_VALID_MPH as stationary.
void update_speedo(float speed_mph, bool has_fix) {
    static int last_speed = -1;
    static bool last_fix = true;

    if (!has_fix) {
        if (last_fix) {
            lv_label_set_text(ui_SpeedoValue, "--");
            lv_label_set_text(ui_SpeedV, "--");
            lv_label_set_text(ui_SpeedV2, "--");
            last_fix = false;
            last_speed = -1;   // force a redraw once the fix comes back
        }
        return;
    }

    if (speed_mph < GPS_MIN_VALID_MPH) speed_mph = 0.0f;
    int speed_int = (int)speed_mph;

    if (!last_fix || speed_int != last_speed) {
        int16_t mph_angle = (int16_t)(SPEEDO_NEEDLE_REST_ANGLE +
            speed_mph * (SPEEDO_NEEDLE_ANGLE_80MPH - SPEEDO_NEEDLE_REST_ANGLE) / 80);
        gauge_channel_t ch = { .value_label = ui_SpeedoValue, .mirror_label = NULL, .needle = ui_SpeedoNeedle, .log_tag = "Mph_Speed", .angle_min = SPEEDO_NEEDLE_REST_ANGLE, .angle_max = SPEEDO_SWEEP_ANGLE_DELTA, .decimals = 0 };
        update_gauge_channel(&ch, speed_mph, mph_angle);

        int speed_kmh = (int)(speed_mph * 1.609344f);
        int16_t kmh_angle = (int16_t)(595 -speed_kmh*1143/100);
        gauge_channel_t ch2 = { .value_label = ui_SpeedV, .mirror_label = ui_SpeedV2, .needle = ui_SpeedD, .log_tag = "Kmh_Speed", .angle_min = 595, .angle_max = 1143, .decimals = 0 };
        update_gauge_channel(&ch2, speed_kmh, kmh_angle );

        last_speed = speed_int;
        last_fix = true;
    }
}

// Defined further down alongside the rest of the GPS/timezone handling;
// forward-declared here so gauge_timer (and update_gps_time_display) can use it.
static bool gps_get_nz_local_time(struct tm *out_nz_tm);

// Refreshes the GPS local-time labels (ui_TimeV / ui_TimeV2), blanking both
// while GPS doesn't have a valid time yet.
static void update_gps_time_display(void) {
    // Starts true so the very first call (before GPS has any fix) forces an
    // initial blank, clearing SquareLine's design-time placeholder text
    // ("16:45") instead of leaving it on-screen until GPS time happens to
    // go valid then invalid once.
    static bool last_valid = true;
    static int last_hour = -1;
    static int last_min = -1;

    struct tm nz_tm;
    bool valid = gps_get_nz_local_time(&nz_tm);

    if (!valid) {
        if (last_valid) {
            lv_label_set_text(ui_TimeV, "");
            lv_label_set_text(ui_TimeV2, "");
            last_valid = false;
        }
        return;
    }

    if (!last_valid || nz_tm.tm_hour != last_hour || nz_tm.tm_min != last_min) {
        char buf[6];
        snprintf(buf, sizeof(buf), "%02d:%02d", nz_tm.tm_hour, nz_tm.tm_min);
        lv_label_set_text(ui_TimeV, buf);
        lv_label_set_text(ui_TimeV2, buf);
        last_hour = nz_tm.tm_hour;
        last_min = nz_tm.tm_min;
        last_valid = true;
    }
}

void gauge_timer(lv_timer_t * t) {
    // UI-only timer.  Sensor tasks update shared state; this timer formats the
    // values for LVGL and smooths visual movement such as the RPM arc.

    //update ODOMETER
    int miles = odometer_get_miles();
    update_odometer(miles);

    // update AFR
    update_afr(g_gauge_data.afr, g_gauge_data.afr_temp_c);
    update_afr_status(g_gauge_data.afr_temp_c);

    //update temp
    update_water_temp(g_gauge_data.water_temp_c, g_gauge_data.water_temp_valid);

    //update speed
    update_speedo(g_gauge_data.speed_mph, gps_has_fix());

    //update RPM
    update_rpm(g_gauge_data.rpm);

    //update GPS time
    update_gps_time_display();
}

//------------------------------------------------------------------------//



float resistance_to_C(float R) {
    // Table lookup with linear interpolation between measured sender points.
    // The resistance table is descending as temperature rises.
    if (R >= sensorR[0]) return tempC[0];
    if (R <= sensorR[TEMP_SENSOR_TABLE_SIZE - 1]) return tempC[TEMP_SENSOR_TABLE_SIZE - 1];

    for (int i = 0; i < TEMP_SENSOR_TABLE_SIZE - 1; i++) {
        if (R <= sensorR[i] && R >= sensorR[i+1]) {
            float t = tempC[i] + (sensorR[i] - R) * (tempC[i+1] - tempC[i]) / (sensorR[i] - sensorR[i+1]);
            return t;
        }
    }
    return tempC[0];
}



//---------------------------------GPS------------------------//

void save_miles_task(void *arg){
    // Persisting every meter would wear NVS quickly.  The odometer module only
    // commits after its distance threshold, and this task gives it regular
    // chances to do that work.
    while (1){
        odometer_periodic_save();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

//-------------------------------AFR (14point7 Spartan)-------------------------------//
#define AFR_POLL_PERIOD_MS      200
#define AFR_RESPONSE_TIMEOUT_MS 300

// GETDEFINITIONS replies with one line per field, not a single field-name
// list, e.g.:
//   0,a,i,-,A/F,14P7 A/F,AFR,7,20,
//   1,a,i,-,LSU T,14P7 LSU T,C,710,850,
//   2,a,i,-,DebugAIC,14P7 DbgAIC,,1,100,
//   3,a,i,-,DebugIp,14P7 DbgIp,,1,100
// as <index>,<type>,<io>,<->,<short_name>,<long_name>,<unit>,<min>,<max>,
// The leading index is exactly the field index used by "G" below, so rather
// than inferring order from text position, this just remembers which line's
// index mentioned "A/F"/"AFR" (or "LSU" for the sensor cell temperature).
// Not every unit necessarily has a channel called that, so these defaults
// (matching the real capture above) are kept as a fallback.
static int afr_field_index = 0;
static int afr_temp_field_index = 1;

// Case-insensitive substring search. Not using strcasestr() here - it isn't
// reliably declared by this toolchain's <string.h> without extra feature-test
// macros (the same issue bit timegm() earlier), so a small hand-rolled
// version avoids that risk entirely.
static const char *afr_stristr(const char *haystack, const char *needle) {
    if (!*needle) return haystack;
    for (; *haystack; haystack++) {
        const char *h = haystack;
        const char *n = needle;
        while (*h && *n && tolower((unsigned char)*h) == tolower((unsigned char)*n)) {
            h++;
            n++;
        }
        if (!*n) return haystack;
    }
    return NULL;
}

// Scans a (possibly multi-line) GETDEFINITIONS reply for the lines describing
// the AFR and sensor-temperature channels and records their indices. Logs
// what it found either way.
static void afr_parse_definitions(const char *text) {
    char copy[400];
    strncpy(copy, text, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *saveptr = NULL;
    char *line = strtok_r(copy, "\r\n", &saveptr);
    bool found_afr = false, found_temp = false;

    while (line != NULL) {
        if (!found_afr && (afr_stristr(line, "A/F") || afr_stristr(line, "AFR"))) {
            afr_field_index = atoi(line);
            found_afr = true;
            ESP_LOGI(TAG_AFR, "Discovered AFR field at index %d: %s", afr_field_index, line);
        } else if (!found_temp && afr_stristr(line, "LSU")) {
            afr_temp_field_index = atoi(line);
            found_temp = true;
            ESP_LOGI(TAG_AFR, "Discovered sensor temp field at index %d: %s", afr_temp_field_index, line);
        }
        line = strtok_r(NULL, "\r\n", &saveptr);
    }

    if (!found_afr) {
        ESP_LOGW(TAG_AFR, "GETDEFINITIONS didn't mention A/F or AFR - keeping default field index %d", afr_field_index);
    }
    if (!found_temp) {
        ESP_LOGW(TAG_AFR, "GETDEFINITIONS didn't mention LSU temp - keeping default field index %d", afr_temp_field_index);
    }
}

typedef struct {
    bool afr_valid;
    float afr;
    bool temp_valid;
    float temp_c;
} afr_reading_t;

// "G" replies with one line per field too, as <index>:<type>:<value>, e.g.:
//   0:a:50.0
//   1:a:780
//   2:a:4789
//   3:a:2242
// This scans those lines for the ones matching afr_field_index/afr_temp_field_index
// (as found by afr_parse_definitions()) and returns their values.
static bool afr_parse_g_response(const char *text, afr_reading_t *out) {
    out->afr_valid = false;
    out->temp_valid = false;

    char copy[400];
    strncpy(copy, text, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *saveptr = NULL;
    char *line = strtok_r(copy, "\r\n", &saveptr);

    while (line != NULL) {
        int idx;
        char type;
        float value;
        if (sscanf(line, "%d:%c:%f", &idx, &type, &value) == 3) {
            if (idx == afr_field_index) {
                out->afr = value;
                out->afr_valid = true;
            } else if (idx == afr_temp_field_index) {
                out->temp_c = value;
                out->temp_valid = true;
            }
        }
        line = strtok_r(NULL, "\r\n", &saveptr);
    }
    return out->afr_valid || out->temp_valid;
}

// Sends an AFR command and waits for its reply, returning the number of
// bytes received (0 on timeout) with buf null-terminated either way.
static int afr_send_command(const char *cmd, uint8_t *buf, size_t buf_size) {
    uart_write_bytes(AFR_UART_NUM, cmd, strlen(cmd));
    int len = uart_read_bytes(AFR_UART_NUM, buf, buf_size - 1, pdMS_TO_TICKS(AFR_RESPONSE_TIMEOUT_MS));
    if (len < 0) len = 0;
    buf[len] = '\0';
    return len;
}

void afr_task(void *arg) {
    // Request/response protocol, not a continuous stream: only talks to the
    // Spartan while g_afr_enabled (set by update_afr_power_gate once RPM has
    // qualified). On the first poll after power-on, runs a one-time setup
    // sequence (identify the unit, configure slow-heat protection, and
    // discover the AFR field's index), then polls with "G" from then on.
    uint8_t buf[400];
    bool queried_definitions = false;

    while (1) {
        if (!g_afr_enabled) {
            queried_definitions = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!queried_definitions) {
            // Give the sensor/level shifter a moment to come up cleanly.
            vTaskDelay(pdMS_TO_TICKS(200));

            if (afr_send_command("GETHW\r\n", buf, sizeof(buf)) > 0)
                ESP_LOGI(TAG_AFR, "GETHW: %s", (char *)buf);

            if (afr_send_command("GETFW\r\n", buf, sizeof(buf)) > 0)
                ESP_LOGI(TAG_AFR, "GETFW: %s", (char *)buf);

            // Our power gate (update_afr_power_gate()) enables the sensor
            // once RPM has merely been sustained for 30s, which doesn't
            // guarantee the exhaust near the sensor bung is hot yet - this
            // tells the controller to wait (up to 10 min) for exhaust gas to
            // reach 350C before driving the heater, avoiding thermal shock.
            if (afr_send_command("SETSLOWHEAT3\r\n", buf, sizeof(buf)) > 0)
                ESP_LOGI(TAG_AFR, "SETSLOWHEAT3: %s", (char *)buf);

            if (afr_send_command("GETDEFINITIONS\r\n", buf, sizeof(buf)) > 0) {
                ESP_LOGI(TAG_AFR, "GETDEFINITIONS: %s", (char *)buf);
                afr_parse_definitions((char *)buf);
            } else {
                ESP_LOGW(TAG_AFR, "No response to GETDEFINITIONS - keeping default field order");
            }
            queried_definitions = true;
        }

        if (afr_send_command("G\r\n", buf, sizeof(buf)) > 0) {
            afr_reading_t reading;
            if (afr_parse_g_response((char *)buf, &reading)) {
                if (reading.afr_valid)  g_gauge_data.afr = reading.afr;
                if (reading.temp_valid) g_gauge_data.afr_temp_c = reading.temp_c;
                #if ENABLE_LOGS
                    ESP_LOGI(TAG_AFR, "G: %s -> afr=%.2f temp=%.0fC", (char *)buf, g_gauge_data.afr, g_gauge_data.afr_temp_c);
                #endif
            } else {
                ESP_LOGW(TAG_AFR, "Unparsable G response: %s", (char *)buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(AFR_POLL_PERIOD_MS));
    }
}
//--------------------------------------------------------------------------------------//

#define NZ_TZ "NZST-12NZDT-13,M9.5.0,M4.1.0/3"

// Converts a GPS UTC date/time into NZ local time (NZST/NZDT), DST included,
// using the C library's POSIX TZ rule instead of hand-rolled calendar math.
//
// newlib here doesn't expose timegm(), so instead of a direct UTC->epoch
// conversion, mktime() is used with TZ briefly forced to UTC0 (mktime()
// interprets its input using whatever the current TZ is), then TZ is
// restored to NZ time for the localtime_r() conversion. This isn't
// reentrant-safe against another task reading local time mid-call, but
// this is the only place in the app that touches TZ.
static bool gps_get_nz_local_time(struct tm *out_nz_tm) {
    if (!gps_date_valid() || !gps_time_valid()) return false;

    struct tm utc_tm = {
        .tm_year = gps_get_year() - 1900,
        .tm_mon  = gps_get_month() - 1,
        .tm_mday = gps_get_day(),
        .tm_hour = gps_get_hour(),
        .tm_min  = gps_get_minute(),
        .tm_sec  = gps_get_second(),
    };

    setenv("TZ", "UTC0", 1);
    tzset();
    time_t epoch = mktime(&utc_tm);
    setenv("TZ", NZ_TZ, 1);
    tzset();

    localtime_r(&epoch, out_nz_tm);
    return true;
}

static void uart_init(uart_port_t uart_num, int txPin, int rxPin, int bufSize, int baud) {
    // Shared UART setup for the two gauge transmit ports and the GPS receive
    // port.  Unused pins are passed as UART_PIN_NO_CHANGE.
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT 
    };

    ESP_ERROR_CHECK(uart_driver_install(
        uart_num,
        bufSize,   // RX buffer
        0,         // TX buffer
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(
        uart_num,
        txPin,
        rxPin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}


// LVGL async callback: applies a formatted GPS status string to ui_GPSDEBUG.
static void gps_debug_label_cb(void *user_data) {
    lv_label_set_text(ui_GPSDEBUG, (const char *)user_data);
}

void gps_task(void *arg) {
    // Feed NMEA bytes into TinyGPS++, update the speed label asynchronously,
    // and use high-quality position updates for lap timing and odometer growth.
    uint8_t buf[128];
    static double last_lat = 0.0;
    static double last_lon = 0.0;
    static bool first_fix = true;

    while (true) {
        int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), portMAX_DELAY);
        #if ENABLE_LOGS
            if (len > 0) {
                ESP_LOGI(TAG_GPS, "UART RX %d bytes: %.*s", len, len, (char *)buf);
            } else if (len < 0) {
                ESP_LOGW(TAG_GPS, "uart_read_bytes error: %d", len);
            }
        #endif
        for (int i = 0; i < len; i++) {
            gps_encode_char(buf[i]);
            gps_ubx_feed_byte(buf[i]);
        }

        bool has_fix = gps_has_fix();
        int  sats_used = gps_sats_used();
        float hdop     = gps_hdop();

        if (has_fix) {
            float new_speed = gps_get_speed_mph();
            g_gauge_data.speed_mph = new_speed;

            if (sats_used >= 5 && hdop < 3.5f && gps_location_updated()){
                double current_lat = gps_get_lat();
                double current_lon = gps_get_lon();
                
                //------------------------------------------------//

                if (first_fix) {
                    last_lat = current_lat;
                    last_lon = current_lon;
                    first_fix = false;
                    continue;
                }

                double meters = gps_distance_between(last_lat, last_lon, current_lat, current_lon);

                // Filter GPS jitter (very important)
                if (new_speed > GPS_MIN_VALID_MPH && meters > 0.05 && meters < 100.0) {
                    static double meter_accumulator = 0.0;

                    meter_accumulator += meters;

                    if (meter_accumulator >= 1.0) {
                        uint32_t whole = (uint32_t)meter_accumulator;
                        odometer_add_meters(whole);
                        meter_accumulator -= whole;
                    }
                }

                last_lat = current_lat;
                last_lon = current_lon;
            }


        } else {
            g_gauge_data.speed_mph = 0;
        }
        #if ENABLE_LOGS
            struct tm nz_tm = {0};
            bool have_nz_time = gps_get_nz_local_time(&nz_tm);

            ESP_LOGI(TAG_GPS,
                    "GPS Fix: %s | Speed: %.0f MPH | Sats Used: %d | HDOP: %.1f | NZ Time: %02d:%02d:%02d",
                    has_fix ? "YES" : "NO",
                    g_gauge_data.speed_mph,
                    sats_used,
                    hdop,
                    have_nz_time ? nz_tm.tm_hour : -1,
                    have_nz_time ? nz_tm.tm_min  : -1,
                    have_nz_time ? nz_tm.tm_sec  : -1);

            static char gps_debug_buf[80];
            snprintf(gps_debug_buf, sizeof(gps_debug_buf),
                    "Fix:%s\nSpd:%.0f\nSats:%d\nHDOP:%.1f\n%02d:%02d:%02d",
                    has_fix ? "Y" : "N",
                    g_gauge_data.speed_mph,
                    sats_used,
                    hdop,
                    have_nz_time ? nz_tm.tm_hour : -1,
                    have_nz_time ? nz_tm.tm_min  : -1,
                    have_nz_time ? nz_tm.tm_sec  : -1);
            lv_async_call(gps_debug_label_cb, gps_debug_buf);
        #endif
        vTaskDelay(1);
    }
}

//------------------------------------------------------------------------//


//------------------------------ADC_UART---------------------------------------//
static adc_oneshot_unit_handle_t g_adc2_handle;
static adc_cali_handle_t g_adc2_cali_handle;

static void adc_global_init(void) {
    // ADC attenuation is configured once at boot.  Individual reads below are
    // sampled and averaged per sensor to reduce noise.
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &g_adc2_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc2_handle, WATER_TEMP_ADC_CHANNEL, &chan_config));

    // Raw ADC counts aren't linearly related to voltage, so use IDF's curve-fitting
    // calibration to get real millivolts instead of a hand-rolled reference constant.
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_2,
        .chan = WATER_TEMP_ADC_CHANNEL,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc2_cali_handle));

    ESP_LOGI("ADC", "ADC Global Init Complete");
}

// Averages `samples` raw ADC reads, then converts the average to a calibrated
// millivolt reading.
int sample_sum_adc2_mv(adc_channel_t adc_channel, int samples){
    uint32_t sum = 0;
    int raw = 0;

    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(g_adc2_handle, adc_channel, &raw);
        sum += raw;
    }

    int avg_raw = sum / samples;
    int millivolts = 0;
    adc_cali_raw_to_voltage(g_adc2_cali_handle, avg_raw, &millivolts);
    return millivolts;
}

// Configures GPIO 47 as a PCNT (pulse counter) input, counting rising edges
// from the ignition coil signal after the H11L1 optoisolator's clean logic
// output. RPM is derived by periodically reading and clearing this count.
static pcnt_unit_handle_t rpm_pcnt_unit = NULL;

static void rpm_pcnt_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = 30000,
        .low_limit = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &rpm_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,  // ignore sub-microsecond noise glitches
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(rpm_pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = RPM_PULSE_GPIO,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(rpm_pcnt_unit, &chan_config, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    ESP_ERROR_CHECK(pcnt_unit_enable(rpm_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(rpm_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(rpm_pcnt_unit));
}

// Reads and clears the pulse count, converting it into an instantaneous RPM
// figure using the elapsed time since the last call.
static int rpm_read_and_reset(void) {
    static int64_t last_read_us = 0;
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = (last_read_us == 0) ? ((int64_t)RPM_SAMPLE_MS * 1000) : (now_us - last_read_us);
    last_read_us = now_us;

    int count = 0;
    pcnt_unit_get_count(rpm_pcnt_unit, &count);
    pcnt_unit_clear_count(rpm_pcnt_unit);

    if (count <= 0 || elapsed_us <= 0) return 0;

    // rpm = (pulses / pulses_per_rev) / (elapsed_us / 60e6)
    return (int)(((int64_t)count * 60000000LL) / (RPM_PULSES_PER_REV * elapsed_us));
}

// Enables the AFR sensor's power (BTS7004) and UART level shifter (TXB0104D)
// once RPM has been sustained above RPM_MIN_RUNNING for RPM_QUALIFY_MS, and
// keeps them on through brief dips - only powering back off once RPM has
// been genuinely near zero (engine stopped, not just idling low) for
// RPM_STOPPED_HOLD_MS. This avoids repeatedly power-cycling the wideband
// sensor's heater on every momentary idle fluctuation.
static void update_afr_power_gate(int rpm) {
    static int64_t qualifying_since_ms = -1;
    static int64_t stopped_since_ms = -1;
    int64_t now_ms = esp_timer_get_time() / 1000;

    if (g_afr_enabled) {
        if (rpm < RPM_STOPPED_BELOW) {
            if (stopped_since_ms < 0) stopped_since_ms = now_ms;
            if (now_ms - stopped_since_ms >= RPM_STOPPED_HOLD_MS) {
                g_afr_enabled = false;
                gpio_set_level(AFR_ENABLE_GPIO, 0);
                gpio_set_level(AFR_OE_GPIO, 0);
                qualifying_since_ms = -1;
                stopped_since_ms = -1;
                ESP_LOGI(TAG_AFR, "Engine stopped - AFR sensor powered off");
            }
        } else {
            stopped_since_ms = -1;
        }
        return;
    }

    if (rpm >= RPM_MIN_RUNNING) {
        if (qualifying_since_ms < 0) qualifying_since_ms = now_ms;
        if (now_ms - qualifying_since_ms >= RPM_QUALIFY_MS) {
            g_afr_enabled = true;
            gpio_set_level(AFR_ENABLE_GPIO, 1);
            gpio_set_level(AFR_OE_GPIO, 1);
            ESP_LOGI(TAG_AFR, "RPM sustained >= %d for %d ms - AFR sensor powered on", RPM_MIN_RUNNING, RPM_QUALIFY_MS);
        }
    } else {
        qualifying_since_ms = -1;
    }
}

static void adc_task(void *arg) {
    // Main analog/pulse acquisition loop.  Each sensor group has its own
    // cadence so slow-changing values like temp do not waste cycles.
    int64_t last_temp_ms = 0;
    int64_t last_rpm_ms  = 0;
    static float water_filtered = -1;


    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        // ---------- Temperature Update ---------- //
        if (now_ms - last_temp_ms >= TEMP_UPDATE_DELAY) {
            last_temp_ms = now_ms;

            // Water temp (ADC2)
            int mv_water = sample_sum_adc2_mv(WATER_TEMP_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float R_water = read_temp_resistance(mv_water);
            #if ENABLE_LOGS
                ESP_LOGI(TAG_TEMP, "mv_water=%d R_water=%.1f", mv_water, R_water);
                ESP_LOGI(TAG_TEMP,"Water: %.1fC",g_gauge_data.water_temp_c);
            #endif

            bool water_sensor_connected = (R_water >= 0 && R_water < TEMP_SENSOR_DISCONNECTED_R);
            g_gauge_data.water_temp_valid = water_sensor_connected;

            if (water_sensor_connected) {
                float water_new = resistance_to_C(R_water);

                if (water_filtered < 0) water_filtered = water_new;

                water_filtered = water_filtered * 0.95f + water_new * 0.05f;

                g_gauge_data.water_temp_c = water_filtered;
            }
        }

        // ---------- RPM / AFR power gate ---------- //
        if (now_ms - last_rpm_ms >= RPM_SAMPLE_MS) {
            last_rpm_ms = now_ms;
            int rpm = rpm_read_and_reset();
            g_gauge_data.rpm = rpm;
            update_afr_power_gate(rpm);
            #if ENABLE_LOGS
                ESP_LOGI("RPM", "rpm=%d", rpm);
            #endif
        }

        // AFR itself is read over UART once powered on - see AFR_UART_NUM
        // (parser not yet implemented).

        // Optional logging
        #if ENABLE_LOGS
           // ESP_LOGI(TAG_TEMP,"Water: %.1fC",g_gauge_data.water_temp_c);
        #endif

        vTaskDelay(pdMS_TO_TICKS(ADC_UPDATE_PERIOD_MS));
    }
}




//------------------------------------------------------------------------//



// LVGL animation exec callback: applies the current animated angle to the needle image.
static void speedo_needle_angle_anim_cb(void *obj, int32_t angle) {
    lv_img_set_angle((lv_obj_t *)obj, (int16_t)angle);
}

// Plays the boot-time speedometer needle sweep (rest -> swept angle -> rest).
static void start_speedo_boot_sweep(void) {
    // One animation with playback: sweeps out over half the time, then LVGL
    // automatically reverses back to the start value over the other half.
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, ui_SpeedoNeedle);
    lv_anim_set_exec_cb(&a, speedo_needle_angle_anim_cb);
    lv_anim_set_values(&a, SPEEDO_NEEDLE_REST_ANGLE, SPEEDO_SWEEP_ANGLE_DELTA);
    lv_anim_set_time(&a, 1750);
    lv_anim_set_playback_time(&a, 1750);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

//-------------------------last-screen persistence-------------------------//
// Remembers which of the three swipeable screens (MainSpeedo/OtherData/RPMAFR)
// was on-screen when the unit was last powered off, so boot returns there
// instead of always landing on MainSpeedo. Screen-change events fire from
// SquareLine-generated swipe handlers already wired up in ui_MainSpeedo.c/
// ui_OtherData.c/ui_RPMAFR.c, so rather than editing those (they get
// overwritten on every SquareLine re-export), this listens for LVGL's own
// LV_EVENT_SCREEN_LOADED event, which _ui_screen_change()'s lv_scr_load_anim()
// fires on any screen change regardless of what triggered it.
#define UI_STATE_NAMESPACE "ui_state"
#define UI_STATE_KEY_SCREEN "last_screen"

typedef enum {
    UI_SCREEN_MAIN_SPEEDO = 0,
    UI_SCREEN_OTHER_DATA  = 1,
    UI_SCREEN_RPMAFR      = 2,
} ui_screen_id_t;

static nvs_handle_t ui_state_handle;

// Assumes nvs_flash_init() has already run (odometer_init() does this at boot).
static void ui_state_init(void) {
    ESP_ERROR_CHECK(nvs_open(UI_STATE_NAMESPACE, NVS_READWRITE, &ui_state_handle));
}

static ui_screen_id_t ui_state_load_last_screen(void) {
    uint8_t screen_id = UI_SCREEN_MAIN_SPEEDO;
    nvs_get_u8(ui_state_handle, UI_STATE_KEY_SCREEN, &screen_id);
    if (screen_id > UI_SCREEN_RPMAFR) screen_id = UI_SCREEN_MAIN_SPEEDO;
    return (ui_screen_id_t)screen_id;
}

static void screen_loaded_cb(lv_event_t *e) {
    lv_obj_t *screen = lv_event_get_target(e);
    uint8_t screen_id;

    if (screen == ui_MainSpeedo)     screen_id = UI_SCREEN_MAIN_SPEEDO;
    else if (screen == ui_OtherData) screen_id = UI_SCREEN_OTHER_DATA;
    else if (screen == ui_RPMAFR)    screen_id = UI_SCREEN_RPMAFR;
    else return;

    nvs_set_u8(ui_state_handle, UI_STATE_KEY_SCREEN, screen_id);
    nvs_commit(ui_state_handle);
}
//----------------------------------------------------------------------------//

// One-shot timer callback: switches from BootScreen to whichever screen was
// last selected (see ui_state_load_last_screen()), kicking off the needle
// sweep only when that screen is MainSpeedo.
static void boot_to_last_screen_cb(lv_timer_t *timer) {
    lv_timer_del(timer);

    switch (ui_state_load_last_screen()) {
        case UI_SCREEN_OTHER_DATA:
            _ui_screen_change(&ui_OtherData, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_OtherData_screen_init);
            break;
        case UI_SCREEN_RPMAFR:
            _ui_screen_change(&ui_RPMAFR, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_RPMAFR_screen_init);
            break;
        default:
            _ui_screen_change(&ui_MainSpeedo, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_MainSpeedo_screen_init);
            start_speedo_boot_sweep();
            break;
    }
}

void app_main(void) {
    // Boot sequence: display/LVGL first, then hardware inputs, persistent
    // odometer state, UI construction, UARTs, and finally the data-source tasks.
    setenv("TZ", NZ_TZ, 1);
    tzset();

    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = false,
        }
    };
    bsp_display_start_with_config(&cfg);

    adc_global_init();
    odometer_init();

    // AFR power gate outputs - held low (disabled) until the RPM gate
    // qualifies (see update_afr_power_gate()).
    gpio_set_direction(AFR_ENABLE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(AFR_OE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(AFR_ENABLE_GPIO, 0);
    gpio_set_level(AFR_OE_GPIO, 0);
    rpm_pcnt_init();

    ui_init();
    ui_state_init();
    lv_obj_add_event_cb(ui_MainSpeedo, screen_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
    lv_obj_add_event_cb(ui_OtherData, screen_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
    lv_obj_add_event_cb(ui_RPMAFR, screen_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);

    lv_timer_create(gauge_timer, 10, NULL);
    lv_timer_create(boot_to_last_screen_cb, 2000, NULL);


    xTaskCreatePinnedToCore(save_miles_task, "save_miles_task", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 5, NULL, 0);

    uart_init(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, (GPS_BUF_SIZE*2), GPS_BAUD_RATE);
    gps_ubx_query_chip_info(GPS_UART_NUM);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 0);

    uart_init(AFR_UART_NUM, AFR_TX_PIN, AFR_RX_PIN, 256, AFR_BAUD_RATE);
    xTaskCreatePinnedToCore(afr_task, "afr_task", 4096, NULL, 5, NULL, 0);

    bsp_display_backlight_off();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    bsp_display_brightness_set(50); 

}
