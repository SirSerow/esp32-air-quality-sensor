#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "nvs_flash.h"

#include "app_config.h"

#include "storage.h"

static const char *TAG = "STORAGE";
static const char *TAG_SD = "SDCARD";
static const char *TAG_NVS = "NVS";

typedef struct {
    int16_t temperature_c_x100;
    int16_t humidity_pct_x100;
    uint16_t eco2_ppm;
    uint16_t tvoc_ppb;
    uint8_t aqi;
    uint8_t ens_validity;
    uint8_t has_aht;
    uint8_t has_ens;
} sensor_log_entry_legacy_t;

static bool parse_sensor_csv_line(const char *line, sensor_log_entry_t *out_entry)
{
    unsigned long unix_time = 0;
    int temp_x100 = 0;
    int humidity_x100 = 0;
    unsigned int eco2_ppm = 0;
    unsigned int tvoc_ppb = 0;
    unsigned int aqi = 0;
    unsigned int ens_validity = 0;
    unsigned int has_aht = 0;
    unsigned int has_ens = 0;

    int fields = sscanf(line,
                        "%lu,%d,%d,%u,%u,%u,%u,%u,%u",
                        &unix_time,
                        &temp_x100,
                        &humidity_x100,
                        &eco2_ppm,
                        &tvoc_ppb,
                        &aqi,
                        &ens_validity,
                        &has_aht,
                        &has_ens);
    if (fields != 9) {
        return false;
    }

    out_entry->unix_time = (uint32_t)unix_time;
    out_entry->temperature_c_x100 = (int16_t)temp_x100;
    out_entry->humidity_pct_x100 = (int16_t)humidity_x100;
    out_entry->eco2_ppm = (uint16_t)eco2_ppm;
    out_entry->tvoc_ppb = (uint16_t)tvoc_ppb;
    out_entry->aqi = (uint8_t)aqi;
    out_entry->ens_validity = (uint8_t)ens_validity;
    out_entry->has_aht = (uint8_t)has_aht;
    out_entry->has_ens = (uint8_t)has_ens;
    return true;
}

static bool entry_matches_filters(const sensor_log_entry_t *entry,
                                  bool require_valid_time,
                                  bool has_from,
                                  uint32_t from_unix_time,
                                  bool has_to,
                                  uint32_t to_unix_time)
{
    if (require_valid_time && entry->unix_time == 0) {
        return false;
    }
    if (has_from && entry->unix_time < from_unix_time) {
        return false;
    }
    if (has_to && entry->unix_time > to_unix_time) {
        return false;
    }
    return true;
}

static void append_entry_with_tail_window(sensor_log_entry_t *entries,
                                          size_t max_entries,
                                          size_t *loaded,
                                          const sensor_log_entry_t *entry)
{
    if (max_entries == 0) {
        return;
    }

    if (*loaded < max_entries) {
        entries[*loaded] = *entry;
        (*loaded)++;
        return;
    }

    memmove(entries, &entries[1], (max_entries - 1) * sizeof(*entries));
    entries[max_entries - 1] = *entry;
}

static void sd_log_debug_levels(void)
{
    ESP_LOGI(TAG_SD,
             "SD pins: MISO=%d MOSI=%d CLK=%d CS=%d",
             PIN_NUM_MISO,
             PIN_NUM_MOSI,
             PIN_NUM_CLK,
             PIN_NUM_CS);
    ESP_LOGI(TAG_SD,
             "GPIO levels (pre-mount): MISO=%d MOSI=%d CLK=%d CS=%d",
             gpio_get_level(PIN_NUM_MISO),
             gpio_get_level(PIN_NUM_MOSI),
             gpio_get_level(PIN_NUM_CLK),
             gpio_get_level(PIN_NUM_CS));
}

static void sd_log_mount_hint(esp_err_t err)
{
    ESP_LOGW(TAG_SD, "Mount failed: %s (%d)", esp_err_to_name(err), (int)err);

    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_SD, "Timeout usually means no SD response. Check CS/MISO/MOSI/CLK wiring and 3.3V power.");
        ESP_LOGW(TAG_SD, "Also verify CS pin is unique and card/module uses 3.3V logic.");
    } else if (err == ESP_FAIL) {
        ESP_LOGW(TAG_SD, "Card responded but mount/init failed. Card format (FAT32) or module quality may be the cause.");
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG_SD, "SD/SPI host may already be initialized by another component.");
    }
}

static esp_err_t sdcard_init_storage(storage_ctx_t *ctx)
{
    ESP_LOGI(TAG_SD, "Initializing SD card via SDSPI...");
    sd_log_debug_levels();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    ESP_LOGI(TAG_SD, "SDSPI host slot=%d max_freq_khz=%d", host.slot, host.max_freq_khz);

    esp_err_t err = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG_SD, "spi_bus_initialize failed: %s (%d)", esp_err_to_name(err), (int)err);
        return err;
    }
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG_SD, "SPI bus already initialized, continuing");
    } else {
        ESP_LOGI(TAG_SD, "SPI bus initialized");
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    ESP_LOGI(TAG_SD, "SDSPI slot config: host_id=%d cs=%d", slot_config.host_id, slot_config.gpio_cs);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ESP_LOGI(TAG_SD, "Mounting FATFS at %s", MOUNT_POINT);
    err = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &ctx->sd_card);
    if (err != ESP_OK) {
        sd_log_mount_hint(err);
        return err;
    }

    sdmmc_card_print_info(stdout, ctx->sd_card);
    ESP_LOGI(TAG_SD, "SD card mounted successfully");

    if (access(LOG_PATH, F_OK) != 0) {
        ESP_LOGI(TAG_SD, "Log file not found, creating: %s", LOG_PATH);
        FILE *f = fopen(LOG_PATH, "w");
        if (!f) {
            ESP_LOGE(TAG_SD, "Failed to create log file: %s", LOG_PATH);
            return ESP_FAIL;
        }
        fprintf(f, "unix_time,temp_x100,humidity_x100,eco2_ppm,tvoc_ppb,aqi,ens_validity,has_aht,has_ens\n");
        fclose(f);
    } else {
        ESP_LOGI(TAG_SD, "Using existing log file: %s", LOG_PATH);
    }

    ctx->sd_ready = true;
    return ESP_OK;
}

static esp_err_t sd_append_sensor_sample(const storage_ctx_t *ctx, const sensor_log_entry_t *entry)
{
    if (!ctx || !ctx->sd_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(LOG_PATH, "a");
    if (!f) {
        return ESP_FAIL;
    }

    int n = fprintf(f,
                    "%lu,%d,%d,%u,%u,%u,%u,%u,%u\n",
                    (unsigned long)entry->unix_time,
                    (int)entry->temperature_c_x100,
                    (int)entry->humidity_pct_x100,
                    (unsigned)entry->eco2_ppm,
                    (unsigned)entry->tvoc_ppb,
                    (unsigned)entry->aqi,
                    (unsigned)entry->ens_validity,
                    (unsigned)entry->has_aht,
                    (unsigned)entry->has_ens);
    fclose(f);

    if (n < 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t nvs_init_storage(storage_ctx_t *ctx)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &ctx->nvs);
    if (err == ESP_OK) {
        ctx->nvs_ready = true;
    }
    return err;
}

static esp_err_t nvs_log_sensor_sample(nvs_handle_t nvs, const sensor_log_entry_t *entry)
{
    bool retried_after_reclaim = false;

retry_write:
    uint32_t wr_idx = 0;
    uint32_t count = 0;

    esp_err_t err = nvs_get_u32(nvs, "wr_idx", &wr_idx);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        wr_idx = 0;
    } else if (err != ESP_OK) {
        return err;
    }

    err = nvs_get_u32(nvs, "count", &count);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        count = 0;
    } else if (err != ESP_OK) {
        return err;
    }

    char slot_key[8];
    snprintf(slot_key, sizeof(slot_key), "r%03lu", (unsigned long)(wr_idx % NVS_SLOT_COUNT));

    err = nvs_set_blob(nvs, slot_key, entry, sizeof(*entry));
    if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE && !retried_after_reclaim) {
        ESP_LOGW(TAG, "NVS full while writing %s, erasing old log entries", slot_key);
        err = nvs_erase_all(nvs);
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_commit(nvs);
        if (err != ESP_OK) {
            return err;
        }
        retried_after_reclaim = true;
        goto retry_write;
    }
    if (err != ESP_OK) {
        return err;
    }

    wr_idx = (wr_idx + 1) % NVS_SLOT_COUNT;
    if (count < NVS_SLOT_COUNT) {
        count++;
    }

    err = nvs_set_u32(nvs, "wr_idx", wr_idx);
    if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE && !retried_after_reclaim) {
        ESP_LOGW(TAG, "NVS full while updating write index, erasing old log entries");
        err = nvs_erase_all(nvs);
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_commit(nvs);
        if (err != ESP_OK) {
            return err;
        }
        retried_after_reclaim = true;
        goto retry_write;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u32(nvs, "count", count);
    if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE && !retried_after_reclaim) {
        ESP_LOGW(TAG, "NVS full while updating count, erasing old log entries");
        err = nvs_erase_all(nvs);
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_commit(nvs);
        if (err != ESP_OK) {
            return err;
        }
        retried_after_reclaim = true;
        goto retry_write;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_commit(nvs);
    if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE && !retried_after_reclaim) {
        ESP_LOGW(TAG, "NVS full on commit, erasing old log entries");
        err = nvs_erase_all(nvs);
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_commit(nvs);
        if (err != ESP_OK) {
            return err;
        }
        retried_after_reclaim = true;
        goto retry_write;
    }

    return err;
}

static esp_err_t nvs_get_u32_default(nvs_handle_t nvs, const char *key, uint32_t default_value, uint32_t *out_value)
{
    esp_err_t err = nvs_get_u32(nvs, key, out_value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *out_value = default_value;
        return ESP_OK;
    }
    return err;
}

static void nvs_log_stats(void)
{
    nvs_stats_t stats = {0};
    esp_err_t err = nvs_get_stats(NULL, &stats);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_NVS, "NVS stats: used=%u free=%u total=%u namespaces=%u",
                 (unsigned)stats.used_entries,
                 (unsigned)stats.free_entries,
                 (unsigned)stats.total_entries,
                 (unsigned)stats.namespace_count);
    } else {
        ESP_LOGW(TAG_NVS, "Failed to read NVS stats: %s", esp_err_to_name(err));
    }
}

static bool nvs_read_entry_compat(nvs_handle_t nvs, const char *key, sensor_log_entry_t *out_entry)
{
    size_t size = sizeof(*out_entry);
    esp_err_t err = nvs_get_blob(nvs, key, out_entry, &size);
    if (err == ESP_OK && size == sizeof(*out_entry)) {
        return true;
    }

    sensor_log_entry_legacy_t legacy = {0};
    size = sizeof(legacy);
    err = nvs_get_blob(nvs, key, &legacy, &size);
    if (err == ESP_OK && size == sizeof(legacy)) {
        memset(out_entry, 0, sizeof(*out_entry));
        out_entry->temperature_c_x100 = legacy.temperature_c_x100;
        out_entry->humidity_pct_x100 = legacy.humidity_pct_x100;
        out_entry->eco2_ppm = legacy.eco2_ppm;
        out_entry->tvoc_ppb = legacy.tvoc_ppb;
        out_entry->aqi = legacy.aqi;
        out_entry->ens_validity = legacy.ens_validity;
        out_entry->has_aht = legacy.has_aht;
        out_entry->has_ens = legacy.has_ens;
        out_entry->unix_time = 0;
        return true;
    }

    return false;
}

static size_t load_filtered_entries_from_sd(const storage_ctx_t *ctx,
                                            sensor_log_entry_t *entries,
                                            size_t max_entries,
                                            bool newest_only,
                                            bool require_valid_time,
                                            bool has_from,
                                            uint32_t from_unix_time,
                                            bool has_to,
                                            uint32_t to_unix_time,
                                            bool *has_more)
{
    if (has_more) {
        *has_more = false;
    }
    if (!ctx || !ctx->sd_ready || max_entries == 0) {
        return 0;
    }

    FILE *f = fopen(LOG_PATH, "r");
    if (!f) {
        return 0;
    }

    size_t loaded = 0;
    size_t matched = 0;
    char line[192];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "unix_time,", 10) == 0) {
            continue;
        }

        sensor_log_entry_t entry = {0};
        if (!parse_sensor_csv_line(line, &entry)) {
            continue;
        }
        if (!entry_matches_filters(&entry, require_valid_time, has_from, from_unix_time, has_to, to_unix_time)) {
            continue;
        }

        matched++;
        if (newest_only) {
            append_entry_with_tail_window(entries, max_entries, &loaded, &entry);
            continue;
        }

        if (loaded < max_entries) {
            entries[loaded++] = entry;
            continue;
        }

        if (has_more) {
            *has_more = true;
        }
        break;
    }

    fclose(f);

    if (newest_only && has_more && matched > max_entries) {
        *has_more = true;
    }

    return loaded;
}

static size_t load_filtered_entries_from_nvs(const storage_ctx_t *ctx,
                                             sensor_log_entry_t *entries,
                                             size_t max_entries,
                                             bool newest_only,
                                             bool require_valid_time,
                                             bool has_from,
                                             uint32_t from_unix_time,
                                             bool has_to,
                                             uint32_t to_unix_time,
                                             bool *has_more)
{
    (void)ctx;

    if (has_more) {
        *has_more = false;
    }
    if (max_entries == 0) {
        return 0;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return 0;
    }

    uint32_t wr_idx = 0;
    uint32_t count = 0;
    err = nvs_get_u32_default(nvs, "wr_idx", 0, &wr_idx);
    if (err == ESP_OK) {
        err = nvs_get_u32_default(nvs, "count", 0, &count);
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        return 0;
    }

    if (count > NVS_SLOT_COUNT) {
        count = NVS_SLOT_COUNT;
    }

    uint32_t start = (wr_idx + NVS_SLOT_COUNT - count) % NVS_SLOT_COUNT;
    size_t loaded = 0;
    size_t matched = 0;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (start + i) % NVS_SLOT_COUNT;
        char key[8];
        snprintf(key, sizeof(key), "r%03lu", (unsigned long)slot);

        sensor_log_entry_t entry = {0};
        if (!nvs_read_entry_compat(nvs, key, &entry)) {
            continue;
        }
        if (!entry_matches_filters(&entry, require_valid_time, has_from, from_unix_time, has_to, to_unix_time)) {
            continue;
        }

        matched++;
        if (newest_only) {
            append_entry_with_tail_window(entries, max_entries, &loaded, &entry);
            continue;
        }

        if (loaded < max_entries) {
            entries[loaded++] = entry;
            continue;
        }

        if (has_more) {
            *has_more = true;
        }
        break;
    }

    nvs_close(nvs);

    if (newest_only && has_more && matched > max_entries) {
        *has_more = true;
    }

    return loaded;
}

static size_t load_entries_after_time_from_sd(const storage_ctx_t *ctx,
                                              sensor_log_entry_t *entries,
                                              size_t entry_capacity,
                                              size_t batch_limit,
                                              uint32_t since_unix_time,
                                              bool require_valid_time,
                                              bool *has_more,
                                              uint32_t *next_since)
{
    if (has_more) {
        *has_more = false;
    }
    if (next_since) {
        *next_since = since_unix_time;
    }
    if (!ctx || !ctx->sd_ready || entry_capacity == 0 || batch_limit == 0) {
        return 0;
    }

    FILE *f = fopen(LOG_PATH, "r");
    if (!f) {
        return 0;
    }

    size_t loaded = 0;
    uint32_t boundary_time = 0;
    bool boundary_active = false;
    char line[192];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "unix_time,", 10) == 0) {
            continue;
        }

        sensor_log_entry_t entry = {0};
        if (!parse_sensor_csv_line(line, &entry)) {
            continue;
        }
        if ((require_valid_time && entry.unix_time == 0) || entry.unix_time <= since_unix_time) {
            continue;
        }

        if (loaded < batch_limit) {
            if (loaded < entry_capacity) {
                entries[loaded] = entry;
            }
            loaded++;
            boundary_time = entry.unix_time;
            if (next_since) {
                *next_since = entry.unix_time;
            }
            continue;
        }

        if (!boundary_active) {
            boundary_active = true;
        }
        if (entry.unix_time != boundary_time) {
            if (has_more) {
                *has_more = true;
            }
            break;
        }
        if (loaded < entry_capacity) {
            entries[loaded] = entry;
        }
        loaded++;
        if (next_since) {
            *next_since = entry.unix_time;
        }
    }

    fclose(f);
    return loaded < entry_capacity ? loaded : entry_capacity;
}

static size_t load_entries_after_time_from_nvs(const storage_ctx_t *ctx,
                                               sensor_log_entry_t *entries,
                                               size_t entry_capacity,
                                               size_t batch_limit,
                                               uint32_t since_unix_time,
                                               bool require_valid_time,
                                               bool *has_more,
                                               uint32_t *next_since)
{
    (void)ctx;

    if (has_more) {
        *has_more = false;
    }
    if (next_since) {
        *next_since = since_unix_time;
    }
    if (entry_capacity == 0 || batch_limit == 0) {
        return 0;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return 0;
    }

    uint32_t wr_idx = 0;
    uint32_t count = 0;
    err = nvs_get_u32_default(nvs, "wr_idx", 0, &wr_idx);
    if (err == ESP_OK) {
        err = nvs_get_u32_default(nvs, "count", 0, &count);
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        return 0;
    }

    if (count > NVS_SLOT_COUNT) {
        count = NVS_SLOT_COUNT;
    }

    uint32_t start = (wr_idx + NVS_SLOT_COUNT - count) % NVS_SLOT_COUNT;
    size_t loaded = 0;
    uint32_t boundary_time = 0;
    bool boundary_active = false;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (start + i) % NVS_SLOT_COUNT;
        char key[8];
        snprintf(key, sizeof(key), "r%03lu", (unsigned long)slot);

        sensor_log_entry_t entry = {0};
        if (!nvs_read_entry_compat(nvs, key, &entry)) {
            continue;
        }
        if ((require_valid_time && entry.unix_time == 0) || entry.unix_time <= since_unix_time) {
            continue;
        }

        if (loaded < batch_limit) {
            if (loaded < entry_capacity) {
                entries[loaded] = entry;
            }
            loaded++;
            boundary_time = entry.unix_time;
            if (next_since) {
                *next_since = entry.unix_time;
            }
            continue;
        }

        if (!boundary_active) {
            boundary_active = true;
        }
        if (entry.unix_time != boundary_time) {
            if (has_more) {
                *has_more = true;
            }
            break;
        }
        if (loaded < entry_capacity) {
            entries[loaded] = entry;
        }
        loaded++;
        if (next_since) {
            *next_since = entry.unix_time;
        }
    }

    nvs_close(nvs);
    return loaded < entry_capacity ? loaded : entry_capacity;
}

size_t storage_load_recent_entries_from_nvs(const storage_ctx_t *ctx,
                                            sensor_log_entry_t *entries,
                                            size_t max_entries)
{
    (void)ctx;

    if (max_entries == 0) {
        return 0;
    }

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return 0;
    }

    uint32_t wr_idx = 0;
    uint32_t count = 0;
    err = nvs_get_u32_default(nvs, "wr_idx", 0, &wr_idx);
    if (err == ESP_OK) {
        err = nvs_get_u32_default(nvs, "count", 0, &count);
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        return 0;
    }

    if (count > NVS_SLOT_COUNT) {
        count = NVS_SLOT_COUNT;
    }

    uint32_t start = (wr_idx + NVS_SLOT_COUNT - count) % NVS_SLOT_COUNT;
    size_t loaded = 0;
    for (uint32_t i = 0; i < count && loaded < max_entries; i++) {
        uint32_t slot = (start + i) % NVS_SLOT_COUNT;
        char key[8];
        snprintf(key, sizeof(key), "r%03lu", (unsigned long)slot);

        sensor_log_entry_t entry = {0};
        if (!nvs_read_entry_compat(nvs, key, &entry)) {
            continue;
        }
        entries[loaded++] = entry;
    }

    nvs_close(nvs);
    return loaded;
}

static size_t load_recent_entries_from_sd(const storage_ctx_t *ctx,
                                          sensor_log_entry_t *entries,
                                          size_t max_entries)
{
    if (!ctx || !ctx->sd_ready || max_entries == 0) {
        return 0;
    }

    FILE *f = fopen(LOG_PATH, "r");
    if (!f) {
        return 0;
    }

    size_t loaded = 0;
    char line[192];
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, "unix_time,", 10) == 0) {
            continue;
        }

        sensor_log_entry_t entry = {0};
        if (!parse_sensor_csv_line(line, &entry)) {
            continue;
        }

        if (loaded < max_entries) {
            entries[loaded++] = entry;
        } else {
            memmove(entries, &entries[1], (max_entries - 1) * sizeof(*entries));
            entries[max_entries - 1] = entry;
        }
    }

    fclose(f);
    return loaded;
}

esp_err_t storage_init(storage_ctx_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));

    esp_err_t err = nvs_init_storage(ctx);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_NVS, "NVS logging enabled (%s)", NVS_NAMESPACE);
        nvs_log_stats();
    } else {
        ESP_LOGW(TAG_NVS, "NVS init failed, continue without persistence: %s", esp_err_to_name(err));
    }

    err = sdcard_init_storage(ctx);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_SD, "SD card logging enabled (%s)", LOG_PATH);
        return ESP_OK;
    }

    ctx->sd_ready = false;
    ESP_LOGW(TAG_SD, "SD card not available at startup, fallback to NVS: %s", esp_err_to_name(err));
    ESP_LOGW(TAG_SD, "SD card init failed, error code: %d", err);
    return ctx->nvs_ready ? ESP_OK : err;
}

esp_err_t storage_append_entry(storage_ctx_t *ctx, const sensor_log_entry_t *entry)
{
    if (!ctx || !entry) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->sd_ready) {
        return sd_append_sensor_sample(ctx, entry);
    }
    if (ctx->nvs_ready) {
        return nvs_log_sensor_sample(ctx->nvs, entry);
    }
    return ESP_ERR_INVALID_STATE;
}

size_t storage_load_recent_entries(const storage_ctx_t *ctx,
                                   sensor_log_entry_t *entries,
                                   size_t max_entries,
                                   bool *from_sd)
{
    if (from_sd) {
        *from_sd = false;
    }

    if (ctx && ctx->sd_ready) {
        size_t count = load_recent_entries_from_sd(ctx, entries, max_entries);
        if (from_sd) {
            *from_sd = true;
        }
        return count;
    }

    return storage_load_recent_entries_from_nvs(ctx, entries, max_entries);
}

size_t storage_load_filtered_entries(const storage_ctx_t *ctx,
                                     sensor_log_entry_t *entries,
                                     size_t max_entries,
                                     bool newest_only,
                                     bool require_valid_time,
                                     bool has_from,
                                     uint32_t from_unix_time,
                                     bool has_to,
                                     uint32_t to_unix_time,
                                     bool *from_sd,
                                     bool *has_more)
{
    if (from_sd) {
        *from_sd = false;
    }

    if (ctx && ctx->sd_ready) {
        size_t count = load_filtered_entries_from_sd(ctx,
                                                     entries,
                                                     max_entries,
                                                     newest_only,
                                                     require_valid_time,
                                                     has_from,
                                                     from_unix_time,
                                                     has_to,
                                                     to_unix_time,
                                                     has_more);
        if (from_sd) {
            *from_sd = true;
        }
        return count;
    }

    return load_filtered_entries_from_nvs(ctx,
                                          entries,
                                          max_entries,
                                          newest_only,
                                          require_valid_time,
                                          has_from,
                                          from_unix_time,
                                          has_to,
                                          to_unix_time,
                                          has_more);
}

size_t storage_load_entries_after_time(const storage_ctx_t *ctx,
                                       sensor_log_entry_t *entries,
                                       size_t entry_capacity,
                                       size_t batch_limit,
                                       uint32_t since_unix_time,
                                       bool require_valid_time,
                                       bool *from_sd,
                                       bool *has_more,
                                       uint32_t *next_since)
{
    if (from_sd) {
        *from_sd = false;
    }

    if (ctx && ctx->sd_ready) {
        size_t count = load_entries_after_time_from_sd(ctx,
                                                       entries,
                                                       entry_capacity,
                                                       batch_limit,
                                                       since_unix_time,
                                                       require_valid_time,
                                                       has_more,
                                                       next_since);
        if (from_sd) {
            *from_sd = true;
        }
        return count;
    }

    return load_entries_after_time_from_nvs(ctx,
                                            entries,
                                            entry_capacity,
                                            batch_limit,
                                            since_unix_time,
                                            require_valid_time,
                                            has_more,
                                            next_since);
}

bool storage_load_latest_entry(const storage_ctx_t *ctx,
                               sensor_log_entry_t *entry,
                               bool require_valid_time,
                               bool *from_sd)
{
    if (!entry) {
        return false;
    }

    bool has_more = false;
    size_t count = storage_load_filtered_entries(ctx,
                                                 entry,
                                                 1,
                                                 true,
                                                 require_valid_time,
                                                 false,
                                                 0,
                                                 false,
                                                 0,
                                                 from_sd,
                                                 &has_more);
    (void)has_more;
    return count == 1;
}

bool storage_is_sd_ready(const storage_ctx_t *ctx)
{
    return ctx && ctx->sd_ready;
}

const char *storage_log_path(void)
{
    return LOG_PATH;
}

void storage_format_unix_time(uint32_t unix_time, char *buf, size_t buf_len)
{
    if (unix_time == 0) {
        snprintf(buf, buf_len, "-");
        return;
    }

    time_t t = (time_t)unix_time;
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    strftime(buf, buf_len, "%Y-%m-%d %H:%M:%S JST", &tm_local);
}
