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
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "driver/adc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "gps_wrapper.h"
#include "odometer/odometer.h"
#include "lap_timer.h"
#include "canbus.h"


// =======================================================
// SENSOR SOURCE CONFIG
// =======================================================
// Build-time switch for the whole dash data path:
// - ANALOG reads local ADC/GPS/tach inputs directly.
// - CAN receives decoded ECU data and maps it into the same gauge_data_t model.
#define SENSOR_SOURCE_ANALOG 0
#define SENSOR_SOURCE_CAN    1

#define SENSOR_SOURCE SENSOR_SOURCE_ANALOG
// =======================================================
//-----Pin Assignment---------//

//GPS RX - GPIO 35
#define GPS_RX_PIN        35
#define GPS_UART_NUM      UART_NUM_3
#define GPS_TX_PIN        UART_PIN_NO_CHANGE  


//Water Temp - GPIO 20
#define WATER_TEMP_ADC_CHANNEL ADC1_CHANNEL_4

// AFR - GPIO 49
#define AFR_ADC_CHANNEL ADC2_CHANNEL_0 

//--------------------------//


#define UART_PORT UART_NUM_1
#define UART1_PORT UART_NUM_2
#define GAUGE_PKT_SOF   0xA5
#define GAUGE_PKT_LEN   26
#define UART_TX_BUF_SIZE 256
#define UART_BAUD_RATE 2000000

#define ENABLE_LOGS true
#define ADC_UPDATE_PERIOD_MS 10
#define FILTER_SAMPLES_DEFAULT 8


//------------GPS-----------//         
#define GPS_BAUD_RATE     115200
#define GPS_BUF_SIZE          1024
#define GPS_MIN_VALID_MPH 3.0f

static volatile float g_speed_mph = 0.0f;
//--------------------------//

//--------UPDATE/REFRESH_DELAYS------//
#define TEMP_UPDATE_DELAY 250 // 4 updates a second
#define AFR_UPDATE_DELAY   20 // 50 updates a second
//-----------------------------------//

//-------------TEMP-------------//
#define R_PULLUP       1000.0f
#define R1 10000.0f
#define R2 20000.0f
#define ADC_SCALE (R2 / (R1 + R2))
#define ADC_MAX        4095.0f
#define ADC_VREF       3.7f
#define SENSOR_SUPPLY  5.0f
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_11

#define TEMP_SENSOR_TABLE_SIZE 9
const float tempF[] = {
    32, 68, 104, 140, 176, 212, 248, 284, 302
};

const float sensorR[] = {
    12000, 8000, 5830, 3020, 1670, 975, 599, 386, 316
};
//------------------------------//


//-------------WIDEBAND AFR-------------//
#define AFR_DIVIDER_GAIN ((68.0f + 33.0f) / 33.0f)   // ≈ 3.06
#define AFR_OFFSET 0.7f 
#define AFR_FILTER_ALPHA 0.3f   // smoothing
//--------------------------------------//


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
    float water_temp_f;
    float afr;
} gauge_data_t;

static gauge_data_t g_gauge_data;


// Wire payload sent after GAUGE_PKT_SOF and a sequence byte.
// Values are multiplied by 10 to avoid sending floats over UART.
typedef struct __attribute__((packed)) {
    uint16_t water_temp;     // °F x10
    uint16_t afr;            // AFR x10
} gauge_payload_t;


//---------------------------------------//

typedef struct {
    lv_obj_t *value_label;
    lv_obj_t *needle;
    const char *log_tag;
    int16_t angle_min;
    int16_t angle_max;
} gauge_channel_t;

static void update_gauge_channel(const gauge_channel_t *ch, float new_value, int16_t angle){
    char buf[12];
    snprintf(buf, sizeof(buf), "%4.1f", new_value);

    // Only update text if changed
    const char *old_text = lv_label_get_text(ch->value_label);
    if (strcmp(old_text, buf) == 0) return;

    lv_label_set_text(ch->value_label, buf);

    if (angle > ch->angle_max) angle = ch->angle_max;
    if (angle < ch->angle_min) angle = ch->angle_min;
    ESP_LOGI(ch->log_tag, "value=%.1f angle=%d", new_value, angle);
    lv_img_set_angle(ch->needle, angle);
}

static void update_afr(float new_value){
    int16_t afr_angle = (int16_t)((new_value - 10.0) * (1640 - 845) / 8 + 845);
    gauge_channel_t ch = { ui_AfrV, ui_AfrD, "AFR", 845, 1640 };
    update_gauge_channel(&ch, new_value, afr_angle);
}

static void update_water_temp(float new_value){
    int16_t temp_angle = (int16_t)(2747 - (new_value - 40.0) * (2747 - 1940) / 80);
    gauge_channel_t ch = { ui_TempV, ui_TempD, "TEMP", 1940, 2747 };
    update_gauge_channel(&ch, new_value, temp_angle);
}


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

float read_temp_resistance(int raw){
    // ADC measures the divided-down sender voltage.  Convert back through the
    // input divider and then solve the pull-up/sensor divider for thermistor
    // resistance before table interpolation.

    float adc_voltage = (raw / ADC_MAX) * ADC_VREF;

    // Undo scaling divider
    float signal_voltage = adc_voltage / ADC_SCALE;

    // Calculate sensor resistance
    float sensor_resistance = R_PULLUP *
        (signal_voltage / (SENSOR_SUPPLY - signal_voltage));

    return sensor_resistance;
}

//-----------------------------------------------------//


void gauge_timer(lv_timer_t * t) {
    // UI-only timer.  Sensor tasks update shared state; this timer formats the
    // values for LVGL and smooths visual movement such as the RPM arc.


    //update ODOMETER
    int miles = odometer_get_miles();
    #if ENABLE_LOGS
        ESP_LOGI("ODOMETER", "miles=%d", miles);
    #endif
    //loop through each of the 6 odometer labels and update them with the corresponding digit
    lv_obj_t *odo_labels[6] = {
        ui_Odometer1, ui_Odometer2, ui_Odometer3, ui_Odometer4, ui_Odometer5, ui_Odometer6
    };
    int divisor = 100000;  // hundred-thousands down to units
    for (int i = 0; i < 6; i++) {
        update_odo_if_needed(odo_labels[i], (miles / divisor) % 10);
        divisor /= 10;
    }


    // update AFR
    update_afr(g_gauge_data.afr);

    //update temp
    update_water_temp(g_gauge_data.water_temp_f);


    if (SENSOR_SOURCE == SENSOR_SOURCE_CAN){
        float speed_mph = g_speed_mph;

        if (speed_mph < GPS_MIN_VALID_MPH)
            speed_mph = 0.0f;

        int speed = (int)speed_mph;
        static char buf[8];
        snprintf(buf, sizeof(buf), "%d", speed);
        //lv_label_set_text(ui_label_mph_value, buf);
        

        char afr_buf[12];
        snprintf(afr_buf, sizeof(afr_buf), "%4.1f", g_gauge_data.afr);
    
    }


}

//------------------------------------------------------------------------//



float resistance_to_F(float R) {
    // Table lookup with linear interpolation between measured sender points.
    // The resistance table is descending as temperature rises.
    if (R >= sensorR[0]) return tempF[0];
    if (R <= sensorR[TEMP_SENSOR_TABLE_SIZE - 1]) return tempF[TEMP_SENSOR_TABLE_SIZE - 1];

    for (int i = 0; i < TEMP_SENSOR_TABLE_SIZE - 1; i++) {
        if (R <= sensorR[i] && R >= sensorR[i+1]) {
            float t = tempF[i] + (sensorR[i] - R) * (tempF[i+1] - tempF[i]) / (sensorR[i] - sensorR[i+1]);
            return t; 
        }
    }
    return tempF[0];
}



static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    // CRC covers the sequence byte and payload, leaving SOF outside the check
    // and reserving the final two bytes for the CRC itself.
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}


//---------------------------------GPS------------------------//

static void speed_update_cb(void *arg){
    // Runs on the LVGL side via lv_async_call so gps_task does not directly
    // mutate widgets from its FreeRTOS context.
    lv_obj_t *label = (lv_obj_t *)arg;
    static bool last_fix = true;
    static int last_speed = -1;

    bool has_fix = gps_has_fix();

    if (!has_fix) {
        if (last_fix) {
            lv_label_set_text(ui_SpeedV, "--");
            last_fix = false;
        }
        return;
    }

    float speed_mph = g_speed_mph;

    if (speed_mph < GPS_MIN_VALID_MPH)
        speed_mph = 0.0f;

    int speed = (int)speed_mph;
    ESP_LOGI("Speed", "speed=%d",speed);

    if (!last_fix || speed != last_speed) {
        static char buf[8];
        snprintf(buf, sizeof(buf), "%d", speed);
        lv_label_set_text(ui_SpeedV, buf);
        if (speed>=110) {
            lv_img_set_angle(ui_SpeedD,-600);}
        else {
            lv_img_set_angle(ui_SpeedD,595 -speed*1143/100);}// 595 = 0, -548=100, 1143/100 = 11.43 per km, 1.143 deg
         
                
        last_speed = speed;
        last_fix = true;
    }
}

void save_miles_task(void *arg){
    // Persisting every meter would wear NVS quickly.  The odometer module only
    // commits after its distance threshold, and this task gives it regular
    // chances to do that work.
    while (1){
        odometer_periodic_save();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
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
        for (int i = 0; i < len; i++) {
            gps_encode_char(buf[i]);
        }

        bool has_fix = gps_has_fix();
        int  sats_used = gps_sats_used();
        float hdop     = gps_hdop();

        static int last_speed = -1;

        if (has_fix) {
            float new_speed = gps_get_speed_mph();
            g_speed_mph = new_speed;

            int speed_int = (int)new_speed;

            if (speed_int != last_speed) {
                lv_async_call(speed_update_cb, ui_SpeedoNeedle);
                last_speed = speed_int;
            }
            if (sats_used >= 5 && hdop < 3.5f && gps_location_updated()){
                double current_lat = gps_get_lat();
                double current_lon = gps_get_lon();
                
                //----- Lap timer(comment out if not needed )------//
                lap_timer_update(current_lat, current_lon, new_speed, has_fix);
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
            g_speed_mph = 0;                
            lv_async_call(speed_update_cb, ui_SpeedoNeedle);
        }
        #if ENABLE_LOGS
            ESP_LOGI(TAG_GPS,
                    "GPS Fix: %s | Speed: %.0f MPH | Sats Used: %d | HDOP: %.1f",
                    has_fix ? "YES" : "NO",
                    g_speed_mph,
                    sats_used,
                    hdop);
        #endif
        vTaskDelay(1);
    }
}

//------------------------------------------------------------------------//


//------------------------------ADC_UART---------------------------------------//
static void adc_global_init(void) {
    // ADC attenuation is configured once at boot.  Individual reads below are
    // sampled and averaged per sensor to reduce noise.
    adc1_config_width(ADC_WIDTH);

    // ADC1 channels
    adc1_config_channel_atten(WATER_TEMP_ADC_CHANNEL, ADC_ATTEN_DB_11);
    

    // ADC2 channels
    adc2_config_channel_atten(AFR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    ESP_LOGI("ADC", "ADC Global Init Complete");
}

uint32_t sample_sum_adc1(adc1_channel_t adc_channel, int samples){
    // Simple boxcar average for ADC1 sensors.
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc1_get_raw(adc_channel);
    }
    return sum / samples;
}

uint32_t sample_sum_adc2(adc2_channel_t adc_channel, int samples){
    // ADC2 read API returns through an output parameter, so average manually.
    uint32_t sum = 0;
    int raw = 0;

    for (int i = 0; i < samples; i++) {
        adc2_get_raw(adc_channel, ADC_WIDTH, &raw);
        sum += raw;
    }

    return sum / samples;
}

static void adc_task(void *arg) {
    // Main analog acquisition loop.  Each sensor group has its own cadence so
    // slow-changing values like fuel do not waste cycles, while AFR/pressure
    // still update quickly enough for the display and UART stream.
    int64_t last_temp_ms     = 0;
    int64_t last_afr_ms      = 0;
    int64_t last_tx_ms       = 0;
    static float water_filtered = -1;
    

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        // ---------- Temperature Update ---------- //
        if (now_ms - last_temp_ms >= TEMP_UPDATE_DELAY) {
            last_temp_ms = now_ms;

            // Water temp (ADC1)
            //int raw_water = adc1_get_raw(WATER_TEMP_ADC_CHANNEL);
            int raw_water = sample_sum_adc1(WATER_TEMP_ADC_CHANNEL, FILTER_SAMPLES_DEFAULT);
            float R_water = read_temp_resistance(raw_water);


            float water_new = resistance_to_F(R_water);

            if (water_filtered < 0) water_filtered = water_new;

            water_filtered = water_filtered * 0.95f + water_new * 0.05f;

            g_gauge_data.water_temp_f = water_filtered;
        }

        // ---------- Wideband AFR (ADC2) ---------- //
        if (now_ms - last_afr_ms >= AFR_UPDATE_DELAY) { 
            last_afr_ms = now_ms; 
            int raw_afr; 
            adc2_get_raw(AFR_ADC_CHANNEL, ADC_WIDTH, &raw_afr); 
            float adc_voltage = ((float)raw_afr / 4095.0f) * ADC_VREF; 
            // Undo voltage divider to get actual AEM output voltage 
            float wb_voltage = adc_voltage * AFR_DIVIDER_GAIN; 
            // Apply AEM linear scaling (Page 11) 
            float afr = (2.3750f * wb_voltage) + 7.3125f;
            afr += AFR_OFFSET;
            // Optional clamp for sanity 
            if (afr < 7.0f) afr = 7.0f; 
            if (afr > 22.0f) afr = 22.0f; 
            // Simple EMA filter 
            static float afr_filtered = 14.7f; 
            afr_filtered = afr_filtered + AFR_FILTER_ALPHA * (afr - afr_filtered); 
            g_gauge_data.afr = afr_filtered;
        }


        // Optional logging
        #if ENABLE_LOGS
            ESP_LOGI(TAG_TEMP,
                    "Water: %.1fF",
                    g_gauge_data.water_temp_f);

            ESP_LOGI(TAG_AFR,
                    "AFR: %.2f",
                    g_gauge_data.afr);
        #endif

        vTaskDelay(pdMS_TO_TICKS(ADC_UPDATE_PERIOD_MS));
    }
}




//------------------------------------------------------------------------//

// Matches the initial lv_img_set_angle() set on ui_SpeedoNeedle in ui_MainSpeedo.c (0 mph rest position).
#define SPEEDO_NEEDLE_REST_ANGLE   (-670)
// How far the sweep rotates from rest before returning; sign/magnitude tuned by eye
// against the dial artwork to land near the 80 mph mark, since this needle has no
// existing mph->angle calibration to derive it from.
#define SPEEDO_SWEEP_ANGLE_DELTA   (2455)

static void speedo_needle_angle_anim_cb(void *obj, int32_t angle) {
    lv_img_set_angle((lv_obj_t *)obj, (int16_t)angle);
}

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

static void boot_to_main_speedo_cb(lv_timer_t *timer) {
    lv_timer_del(timer);
    _ui_screen_change(&ui_MainSpeedo, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_MainSpeedo_screen_init);
    start_speedo_boot_sweep();
}

void app_main(void) {
    // Boot sequence: display/LVGL first, then hardware inputs, persistent
    // odometer state, UI construction, UARTs, and finally the data-source tasks.
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
    lv_display_t *disp = bsp_display_start_with_config(&cfg);

    adc_global_init();
    odometer_init();

    ui_init();
    lv_timer_create(gauge_timer, 10, NULL);
    lv_timer_create(boot_to_main_speedo_cb, 1000, NULL);


    xTaskCreatePinnedToCore(save_miles_task, "save_miles_task", 4096, NULL, 4, NULL, 0);

    bsp_display_backlight_off();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    bsp_display_brightness_set(50); 

}
