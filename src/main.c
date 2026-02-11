#include <stdio.h>
#include <assert.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Logging / errors
#include "esp_err.h"
#include "esp_log.h"

// ESP-IDF 5.x I2C master-bus API (IMPORTANT)
#include "driver/i2c_master.h"

// K0I05 components
#include "ahtxx.h"
#include "ens160.h"

// EEPROM

static const char *TAG = "MAIN";

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    6
#define I2C_SCL_GPIO    7
#define I2C_FREQ_HZ     100000

typedef struct {
    ahtxx_handle_t aht;
    ens160_handle_t ens;
} sensors_t;

static void loop_task(void *pvParameter)
{
    sensors_t *s = (sensors_t *)pvParameter;

    while (1) {
        // ---- AHT21 (temperature / humidity) ----
        float t_c = 0.0f, rh = 0.0f;
        esp_err_t err = ahtxx_get_measurement(s->aht, &t_c, &rh);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "AHT: T=%.2f C  RH=%.2f %%", t_c, rh);
        } else {
            ESP_LOGW(TAG, "AHT read failed: %s", esp_err_to_name(err));
        }

        // ---- ENS160 (AQI / TVOC / eCO2) ----
        // ENS160 can be "warming up" at first; validity flag tells you when data is normal.
        ens160_validity_flags_t flag;
        err = ens160_get_validity_status(s->ens, &flag);
        if (err == ESP_OK && flag == ENS160_VALFLAG_NORMAL) {
            ens160_air_quality_data_t aq;
            err = ens160_get_measurement(s->ens, &aq);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "ENS: AQI=%u  TVOC=%u ppb  eCO2=%u ppm",
                         aq.uba_aqi, aq.tvoc, aq.eco2);
            } else {
                ESP_LOGW(TAG, "ENS read failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGI(TAG, "ENS not ready yet (validity=%d)", (int)flag);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C + AHT21 + ENS160");

    // 1) Create I2C bus (ESP-IDF 5.x)
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

    // 2) Init AHTxx (AHT21 works with AHT20-style config in this driver family)
    ahtxx_config_t aht_cfg = I2C_AHT20_CONFIG_DEFAULT;
    ahtxx_handle_t aht = NULL;
    ahtxx_init(bus, &aht_cfg, &aht);
    assert(aht);

    // 3) Init ENS160
    ens160_config_t ens_cfg = I2C_ENS160_CONFIG_DEFAULT;
    ens160_handle_t ens = NULL;
    ens160_init(bus, &ens_cfg, &ens);
    assert(ens);

    // 4) Start loop task
    static sensors_t sensors;
    sensors.aht = aht;
    sensors.ens = ens;

    xTaskCreate(loop_task, "loop_task", 4096, &sensors, 5, NULL);
}
