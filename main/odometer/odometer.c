#include "odometer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_crc.h"
#include <string.h>

#define ODO_NAMESPACE   "odometer"
#define ODO_KEY_A       "odo_a"
#define ODO_KEY_B       "odo_b"

#define SAVE_INTERVAL_M 100

static const char *TAG = "ODOMETER";

typedef struct {
    // One persisted odometer copy.  version lets the newest valid slot win, and
    // crc catches torn writes or flash corruption before a value is trusted.
    uint64_t meters;
    uint32_t version;
    uint32_t crc;
} odo_record_t;

static nvs_handle_t odo_handle;

static uint64_t total_meters = 95944*1609;   // compiled default
static uint64_t last_saved_meters = 0;
static uint32_t current_version = 0;
static bool use_slot_a = true;

/* ===============================
   CRC Helper
   =============================== */

static uint32_t calculate_crc(const odo_record_t *rec)
{
    // Exclude the crc field itself so the checksum represents only the stored
    // odometer value and version.
    return esp_crc32_le(0,
                        (const uint8_t *)rec,
                        sizeof(odo_record_t) - sizeof(uint32_t));
}

static bool record_valid(odo_record_t *rec)
{
    uint32_t crc = calculate_crc(rec);
    return (crc == rec->crc);
}

/* ===============================
   INIT
   =============================== */

void odometer_init(void)
{
    // NVS can require erase/re-init after partition changes or version changes.
    // Once open, both slots are checked and the newest valid record is selected.
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(nvs_open(ODO_NAMESPACE, NVS_READWRITE, &odo_handle));

    odo_record_t rec_a = {0};
    odo_record_t rec_b = {0};

    bool valid_a = false;
    bool valid_b = false;

    if (nvs_get_blob(odo_handle, ODO_KEY_A, &rec_a, &(size_t){sizeof(rec_a)}) == ESP_OK)
        valid_a = record_valid(&rec_a);

    if (nvs_get_blob(odo_handle, ODO_KEY_B, &rec_b, &(size_t){sizeof(rec_b)}) == ESP_OK)
        valid_b = record_valid(&rec_b);

    if (valid_a && valid_b) {
        // Both records are good; use the highest version and write the next
        // update to the opposite slot to keep wear balanced.
        if (rec_a.version >= rec_b.version) {
            total_meters = rec_a.meters;
            current_version = rec_a.version;
            use_slot_a = false;
        } else {
            total_meters = rec_b.meters;
            current_version = rec_b.version;
            use_slot_a = true;
        }
        ESP_LOGI(TAG, "Loaded dual copy: %llu meters (v%u)",
                 total_meters, current_version);
    }
    else if (valid_a) {
        // One-slot recovery lets the odometer survive an interrupted write.
        total_meters = rec_a.meters;
        current_version = rec_a.version;
        use_slot_a = false;
        ESP_LOGW(TAG, "Recovered from slot A only");
    }
    else if (valid_b) {
        total_meters = rec_b.meters;
        current_version = rec_b.version;
        use_slot_a = true;
        ESP_LOGW(TAG, "Recovered from slot B only");
    }
    else {
        ESP_LOGW(TAG, "No valid record found. Using default: %llu meters",
                 total_meters);
        current_version = 1;
    }

    last_saved_meters = total_meters;
}

/* ===============================
   ADD DISTANCE
   =============================== */

void odometer_add_meters(uint32_t meters)
{
    total_meters += meters;
}

/* ===============================
   INTERNAL SAVE
   =============================== */

static void save_record(void)
{
    // Alternate between two NVS keys.  If power drops during a commit, the
    // previous slot should still contain a valid CRC/version pair.
    odo_record_t rec;

    rec.meters = total_meters;
    rec.version = ++current_version;
    rec.crc = calculate_crc(&rec);

    const char *key = use_slot_a ? ODO_KEY_A : ODO_KEY_B;

    ESP_ERROR_CHECK(nvs_set_blob(odo_handle, key, &rec, sizeof(rec)));
    ESP_ERROR_CHECK(nvs_commit(odo_handle));

    use_slot_a = !use_slot_a;
    last_saved_meters = total_meters;

    ESP_LOGI(TAG, "Saved %llu meters (v%u) to %s",
             total_meters, rec.version, key);
}

/* ===============================
   PERIODIC SAVE
   =============================== */

void odometer_periodic_save(void)
{
    // Distance-based save interval reduces flash wear while still bounding how
    // much distance can be lost after an unexpected power-off.
    if ((total_meters - last_saved_meters) >= SAVE_INTERVAL_M)
    {
        save_record();
    }
}

/* ===============================
   FORCE SAVE
   =============================== */

void odometer_force_save(void)
{
    save_record();
}

/* ===============================
   GETTERS
   =============================== */

uint64_t odometer_get_meters(void)
{
    return total_meters;
}

int odometer_get_miles(void)
{
    return (int)(total_meters / 1609.344);
}
