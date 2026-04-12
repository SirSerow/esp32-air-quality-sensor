#include <assert.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"

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

    ahtxx_config_t aht_cfg = I2C_AHT20_CONFIG_DEFAULT;
    ahtxx_handle_t aht = NULL;
    ahtxx_init(bus, &aht_cfg, &aht);
    assert(aht);

    ens160_config_t ens_cfg = I2C_ENS160_CONFIG_DEFAULT;
    ens160_handle_t ens = NULL;
    ens160_init(bus, &ens_cfg, &ens);
    assert(ens);

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
