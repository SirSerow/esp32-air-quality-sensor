#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

#include "app_config.h"
#include "app_types.h"
#include "network.h"
#include "oled_display.h"
#include "sensor_loop.h"
#include "storage.h"
#include "web_server.h"

static const char *TAG = "MAIN";
static const char *TAG_TIME = "TIME";
static const char *TAG_WIFI = "WIFI";
static const char *TAG_OLED = "OLED";

static ahtxx_handle_t init_aht_with_retries(i2c_master_bus_handle_t bus)
{
    ahtxx_config_t cfg = I2C_AHT20_CONFIG_DEFAULT;

    for (int attempt = 1; attempt <= SENSOR_INIT_RETRY_COUNT; attempt++) {
        ahtxx_handle_t handle = NULL;
        ahtxx_init(bus, &cfg, &handle);
        if (handle) {
            ESP_LOGI(TAG, "AHT initialized on attempt %d/%d", attempt, SENSOR_INIT_RETRY_COUNT);
            return handle;
        }

        ESP_LOGW(TAG,
                 "AHT init failed on attempt %d/%d, retrying in %d ms",
                 attempt,
                 SENSOR_INIT_RETRY_COUNT,
                 SENSOR_INIT_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_RETRY_DELAY_MS));
    }

    ESP_LOGW(TAG, "AHT unavailable after %d attempts, continuing without temperature/humidity sensor",
             SENSOR_INIT_RETRY_COUNT);
    return NULL;
}

static ens160_handle_t init_ens_with_retries(i2c_master_bus_handle_t bus)
{
    ens160_config_t cfg = I2C_ENS160_CONFIG_DEFAULT;

    for (int attempt = 1; attempt <= SENSOR_INIT_RETRY_COUNT; attempt++) {
        ens160_handle_t handle = NULL;
        ens160_init(bus, &cfg, &handle);
        if (handle) {
            ESP_LOGI(TAG, "ENS160 initialized on attempt %d/%d", attempt, SENSOR_INIT_RETRY_COUNT);
            return handle;
        }

        ESP_LOGW(TAG,
                 "ENS160 init failed on attempt %d/%d, retrying in %d ms",
                 attempt,
                 SENSOR_INIT_RETRY_COUNT,
                 SENSOR_INIT_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_RETRY_DELAY_MS));
    }

    ESP_LOGW(TAG, "ENS160 unavailable after %d attempts, continuing without air-quality sensor",
             SENSOR_INIT_RETRY_COUNT);
    return NULL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C + AHT21 + ENS160");
    network_configure_timezone_jst();

    static storage_ctx_t storage;
    esp_err_t err = storage_init(&storage);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Persistent storage init degraded: %s", esp_err_to_name(err));
    }

    err = network_sync_time_via_sta();
    if (err != ESP_OK) {
        ESP_LOGW(TAG_TIME, "Time sync before AP failed: %s", esp_err_to_name(err));
    }

    err = network_start_softap();
    if (err != ESP_OK) {
        ESP_LOGW(TAG_WIFI, "WiFi AP start failed: %s", esp_err_to_name(err));
    } else {
        err = web_server_start(&storage);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_WIFI, "Web server start failed: %s", esp_err_to_name(err));
        }
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    ahtxx_handle_t aht = init_aht_with_retries(bus);
    ens160_handle_t ens = init_ens_with_retries(bus);

    err = oled_display_init(bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_OLED, "OLED init failed, continuing without display: %s", esp_err_to_name(err));
    }

    static sensors_t sensors;
    sensors.aht = aht;
    sensors.ens = ens;
    sensors.nvs = storage.nvs;
    sensors.nvs_ready = storage.nvs_ready;

    err = sensor_loop_start(&sensors, &storage);
    ESP_ERROR_CHECK(err);
}
