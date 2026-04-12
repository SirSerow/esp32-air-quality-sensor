#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "app_config.h"

#include "oled_display.h"
#include "sensor_loop.h"
#include "storage.h"

typedef struct {
    sensors_t sensors;
    storage_ctx_t *storage;
} sensor_loop_ctx_t;

static const char *TAG = "SENSOR";
static sensor_loop_ctx_t s_loop_ctx;

static void loop_task(void *pvParameter)
{
    sensor_loop_ctx_t *ctx = (sensor_loop_ctx_t *)pvParameter;
    sensors_t *s = &ctx->sensors;

    while (1) {
        sensor_log_entry_t log_entry = {0};
        time_t now = 0;
        time(&now);
        if (now > 0) {
            log_entry.unix_time = (uint32_t)now;
        }

        float t_c = 0.0f;
        float rh = 0.0f;
        esp_err_t err = ahtxx_get_measurement(s->aht, &t_c, &rh);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "AHT: T=%.2f C  RH=%.2f %%", t_c, rh);
            log_entry.has_aht = 1;
            log_entry.temperature_c_x100 = (int16_t)(t_c * 100.0f);
            log_entry.humidity_pct_x100 = (int16_t)(rh * 100.0f);
        } else {
            ESP_LOGW(TAG, "AHT read failed: %s", esp_err_to_name(err));
        }

        ens160_validity_flags_t flag = 0;
        err = ens160_get_validity_status(s->ens, &flag);
        if (err == ESP_OK) {
            if (flag == ENS160_VALFLAG_NORMAL) {
                ens160_air_quality_data_t aq;
                err = ens160_get_measurement(s->ens, &aq);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "ENS: AQI=%u  TVOC=%u ppb  eCO2=%u ppm",
                             aq.uba_aqi, aq.tvoc, aq.eco2);
                    log_entry.has_ens = 1;
                    log_entry.ens_validity = (uint8_t)flag;
                    log_entry.aqi = aq.uba_aqi;
                    log_entry.tvoc_ppb = aq.tvoc;
                    log_entry.eco2_ppm = aq.eco2;
                } else {
                    ESP_LOGW(TAG, "ENS read failed: %s", esp_err_to_name(err));
                    log_entry.ens_validity = UINT8_MAX;
                }
            } else {
                ESP_LOGI(TAG, "ENS not ready yet (validity=%d)", (int)flag);
                log_entry.ens_validity = (uint8_t)flag;
            }
        } else {
            ESP_LOGW(TAG, "ENS validity read failed: %s", esp_err_to_name(err));
            log_entry.ens_validity = UINT8_MAX;
        }

        err = storage_append_entry(ctx->storage, &log_entry);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Log write failed: %s", esp_err_to_name(err));
        }

        oled_display_publish_entry(&log_entry);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

esp_err_t sensor_loop_start(const sensors_t *sensors, storage_ctx_t *storage)
{
    if (!sensors || !storage) {
        return ESP_ERR_INVALID_ARG;
    }

    s_loop_ctx.sensors = *sensors;
    s_loop_ctx.storage = storage;

    BaseType_t created = xTaskCreate(loop_task, "loop_task", 4096, &s_loop_ctx, 5, NULL);
    return created == pdPASS ? ESP_OK : ESP_FAIL;
}
