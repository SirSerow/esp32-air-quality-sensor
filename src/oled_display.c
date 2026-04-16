#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "app_config.h"

#include "network.h"
#include "oled_display.h"

static const char *TAG = "OLED";

typedef enum {
    OLED_VIEW_AIR_QUALITY = 0,
    OLED_VIEW_NETWORK,
    OLED_VIEW_OFF,
} oled_view_t;

static i2c_master_dev_handle_t s_oled_dev = NULL;
static bool s_oled_available = false;
static oled_view_t s_oled_view = OLED_VIEW_AIR_QUALITY;
static volatile uint32_t s_button_press_count = 0;
static volatile TickType_t s_button_last_tick = 0;
static portMUX_TYPE s_button_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_latest_entry_mutex = NULL;
static sensor_log_entry_t s_latest_entry = {0};
static bool s_latest_entry_valid = false;

static esp_err_t oled_send_command(uint8_t command)
{
    if (!s_oled_dev) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t payload[2] = {0x00, command};
    return i2c_master_transmit(s_oled_dev, payload, sizeof(payload), -1);
}

static esp_err_t oled_set_power(bool enabled)
{
    return oled_send_command(enabled ? 0xAF : 0xAE);
}

static esp_err_t oled_clear(void)
{
    if (!s_oled_dev) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[OLED_WIDTH + 1] = {0};
    data[0] = 0x40;

    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++) {
        esp_err_t err = oled_send_command((uint8_t)(0xB0 + page));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set page failed: %s", esp_err_to_name(err));
            return err;
        }

        err = oled_send_command(0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set low col failed: %s", esp_err_to_name(err));
            return err;
        }

        err = oled_send_command(0x10);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set high col failed: %s", esp_err_to_name(err));
            return err;
        }

        err = i2c_master_transmit(s_oled_dev, data, sizeof(data), -1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "clear data failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}

static void oled_font5x7(char c, uint8_t out[5])
{
    memset(out, 0, 5);

    if (c >= '0' && c <= '9') {
        static const uint8_t digits[10][5] = {
            {0x3E, 0x51, 0x49, 0x45, 0x3E},
            {0x00, 0x42, 0x7F, 0x40, 0x00},
            {0x62, 0x51, 0x49, 0x49, 0x46},
            {0x22, 0x49, 0x49, 0x49, 0x36},
            {0x18, 0x14, 0x12, 0x7F, 0x10},
            {0x2F, 0x49, 0x49, 0x49, 0x31},
            {0x3E, 0x49, 0x49, 0x49, 0x32},
            {0x01, 0x71, 0x09, 0x05, 0x03},
            {0x36, 0x49, 0x49, 0x49, 0x36},
            {0x26, 0x49, 0x49, 0x49, 0x3E},
        };
        memcpy(out, digits[c - '0'], 5);
        return;
    }

    switch (c) {
    case 'A': memcpy(out, (uint8_t[5]){0x7E, 0x11, 0x11, 0x11, 0x7E}, 5); break;
    case 'B': memcpy(out, (uint8_t[5]){0x7F, 0x49, 0x49, 0x49, 0x36}, 5); break;
    case 'C': memcpy(out, (uint8_t[5]){0x3E, 0x41, 0x41, 0x41, 0x22}, 5); break;
    case 'D': memcpy(out, (uint8_t[5]){0x7F, 0x41, 0x41, 0x22, 0x1C}, 5); break;
    case 'E': memcpy(out, (uint8_t[5]){0x7F, 0x49, 0x49, 0x49, 0x41}, 5); break;
    case 'F': memcpy(out, (uint8_t[5]){0x7F, 0x09, 0x09, 0x09, 0x01}, 5); break;
    case 'G': memcpy(out, (uint8_t[5]){0x3E, 0x41, 0x49, 0x49, 0x7A}, 5); break;
    case 'H': memcpy(out, (uint8_t[5]){0x7F, 0x08, 0x08, 0x08, 0x7F}, 5); break;
    case 'I': memcpy(out, (uint8_t[5]){0x00, 0x41, 0x7F, 0x41, 0x00}, 5); break;
    case 'J': memcpy(out, (uint8_t[5]){0x20, 0x40, 0x41, 0x3F, 0x01}, 5); break;
    case 'K': memcpy(out, (uint8_t[5]){0x7F, 0x08, 0x14, 0x22, 0x41}, 5); break;
    case 'L': memcpy(out, (uint8_t[5]){0x7F, 0x40, 0x40, 0x40, 0x40}, 5); break;
    case 'M': memcpy(out, (uint8_t[5]){0x7F, 0x02, 0x0C, 0x02, 0x7F}, 5); break;
    case 'N': memcpy(out, (uint8_t[5]){0x7F, 0x04, 0x08, 0x10, 0x7F}, 5); break;
    case 'O': memcpy(out, (uint8_t[5]){0x3E, 0x41, 0x41, 0x41, 0x3E}, 5); break;
    case 'P': memcpy(out, (uint8_t[5]){0x7F, 0x09, 0x09, 0x09, 0x06}, 5); break;
    case 'Q': memcpy(out, (uint8_t[5]){0x3E, 0x41, 0x51, 0x21, 0x5E}, 5); break;
    case 'R': memcpy(out, (uint8_t[5]){0x7F, 0x09, 0x19, 0x29, 0x46}, 5); break;
    case 'S': memcpy(out, (uint8_t[5]){0x46, 0x49, 0x49, 0x49, 0x31}, 5); break;
    case 'T': memcpy(out, (uint8_t[5]){0x01, 0x01, 0x7F, 0x01, 0x01}, 5); break;
    case 'U': memcpy(out, (uint8_t[5]){0x3F, 0x40, 0x40, 0x40, 0x3F}, 5); break;
    case 'V': memcpy(out, (uint8_t[5]){0x1F, 0x20, 0x40, 0x20, 0x1F}, 5); break;
    case 'W': memcpy(out, (uint8_t[5]){0x3F, 0x40, 0x38, 0x40, 0x3F}, 5); break;
    case 'X': memcpy(out, (uint8_t[5]){0x63, 0x14, 0x08, 0x14, 0x63}, 5); break;
    case 'Y': memcpy(out, (uint8_t[5]){0x03, 0x04, 0x78, 0x04, 0x03}, 5); break;
    case 'Z': memcpy(out, (uint8_t[5]){0x61, 0x51, 0x49, 0x45, 0x43}, 5); break;
    case ':': memcpy(out, (uint8_t[5]){0x00, 0x36, 0x36, 0x00, 0x00}, 5); break;
    case '.': memcpy(out, (uint8_t[5]){0x00, 0x40, 0x60, 0x00, 0x00}, 5); break;
    case '%': memcpy(out, (uint8_t[5]){0x63, 0x13, 0x08, 0x64, 0x63}, 5); break;
    case '-': memcpy(out, (uint8_t[5]){0x08, 0x08, 0x08, 0x08, 0x08}, 5); break;
    case ' ': default: break;
    }
}

static void oled_draw_text(uint8_t *framebuffer, int x, int y, const char *text)
{
    if (!framebuffer || !text) {
        return;
    }

    for (const char *p = text; *p != '\0'; p++, x += 6) {
        if (x > (OLED_WIDTH - 6)) {
            break;
        }

        uint8_t glyph[5] = {0};
        oled_font5x7(*p, glyph);

        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if ((glyph[col] >> row) & 0x01) {
                    int px = x + col;
                    int py = y + row;
                    if (px < 0 || px >= OLED_WIDTH || py < 0 || py >= OLED_HEIGHT) {
                        continue;
                    }
                    framebuffer[px + (py / 8) * OLED_WIDTH] |= (uint8_t)(1U << (py % 8));
                }
            }
        }
    }
}

static esp_err_t oled_flush(const uint8_t *framebuffer)
{
    if (!s_oled_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!framebuffer) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[OLED_WIDTH + 1];
    tx[0] = 0x40;

    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++) {
        esp_err_t err = oled_send_command((uint8_t)(0xB0 + page));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set page failed: %s", esp_err_to_name(err));
            return err;
        }

        err = oled_send_command(0x00);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set low col failed: %s", esp_err_to_name(err));
            return err;
        }

        err = oled_send_command(0x10);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set high col failed: %s", esp_err_to_name(err));
            return err;
        }

        memcpy(&tx[1], &framebuffer[page * OLED_WIDTH], OLED_WIDTH);
        err = i2c_master_transmit(s_oled_dev, tx, sizeof(tx), -1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "frame transmit failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t oled_render_entry(const sensor_log_entry_t *entry)
{
    if (!entry) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t framebuffer[OLED_WIDTH * OLED_PAGE_COUNT];
    memset(framebuffer, 0, sizeof(framebuffer));

    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];
    char temp_code = '-';
    char rh_code = '-';
    char aqi_code = '-';
    char tvoc_code = '-';
    char eco2_code = '-';
    char validity_code = '-';

    time_t now = 0;
    time(&now);
    if (now > 1700000000) {
        struct tm tm_local;
        localtime_r(&now, &tm_local);
        strftime(line1, sizeof(line1), "%y-%m-%d %H:%M", &tm_local);
    } else {
        snprintf(line1, sizeof(line1), "--/-- --:--");
    }

    if (entry->has_aht) {
        if (entry->temperature_c_x100 >= TEMP_OK_MIN_X100 && entry->temperature_c_x100 <= TEMP_OK_MAX_X100) {
            temp_code = 'G';
        } else if (entry->temperature_c_x100 >= TEMP_WARN_MIN_X100 && entry->temperature_c_x100 <= TEMP_WARN_MAX_X100) {
            temp_code = 'Y';
        } else {
            temp_code = 'R';
        }

        if (entry->humidity_pct_x100 >= RH_OK_MIN_X100 && entry->humidity_pct_x100 <= RH_OK_MAX_X100) {
            rh_code = 'G';
        } else if (entry->humidity_pct_x100 >= RH_WARN_MIN_X100 && entry->humidity_pct_x100 <= RH_WARN_MAX_X100) {
            rh_code = 'Y';
        } else {
            rh_code = 'R';
        }

        snprintf(line2, sizeof(line2), "T:%.1f%c H:%.1f%c",
                 entry->temperature_c_x100 / 100.0f,
                 temp_code,
                 entry->humidity_pct_x100 / 100.0f,
                 rh_code);
    } else {
        snprintf(line2, sizeof(line2), "T:- H:-");
    }

    if (entry->ens_validity == 0) {
        validity_code = 'G';
    } else if (entry->ens_validity == 1) {
        validity_code = 'Y';
    } else {
        validity_code = 'R';
    }

    if (entry->has_ens) {
        if (entry->aqi <= 2) {
            aqi_code = 'G';
        } else if (entry->aqi == 3) {
            aqi_code = 'Y';
        } else {
            aqi_code = 'R';
        }

        if (entry->tvoc_ppb <= TVOC_OK_MAX) {
            tvoc_code = 'G';
        } else if (entry->tvoc_ppb <= TVOC_WARN_MAX) {
            tvoc_code = 'Y';
        } else {
            tvoc_code = 'R';
        }

        if (entry->eco2_ppm <= ECO2_OK_MAX) {
            eco2_code = 'G';
        } else if (entry->eco2_ppm <= ECO2_WARN_MAX) {
            eco2_code = 'Y';
        } else {
            eco2_code = 'R';
        }

        snprintf(line3, sizeof(line3), "A:%u%c T:%u%c", entry->aqi, aqi_code, entry->tvoc_ppb, tvoc_code);
        snprintf(line4, sizeof(line4), "C:%u%c V:%u%c", entry->eco2_ppm, eco2_code, entry->ens_validity, validity_code);
    } else {
        snprintf(line3, sizeof(line3), "A:- T:-");
        snprintf(line4, sizeof(line4), "C:- V:%u%c", entry->ens_validity, validity_code);
    }

    oled_draw_text(framebuffer, 0, 0, line1);
    oled_draw_text(framebuffer, 0, 16, line2);
    oled_draw_text(framebuffer, 0, 32, line3);
    oled_draw_text(framebuffer, 0, 48, line4);

    return oled_flush(framebuffer);
}

static esp_err_t oled_render_network(void)
{
    uint8_t framebuffer[OLED_WIDTH * OLED_PAGE_COUNT];
    memset(framebuffer, 0, sizeof(framebuffer));

    network_status_t status = {0};
    esp_err_t err = network_get_status(&status);
    if (err != ESP_OK) {
        return err;
    }

    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];

    snprintf(line1, sizeof(line1), "NET MODE:%s",
             status.mode == NETWORK_MODE_STA ? "STA" : "AP");
    snprintf(line2, sizeof(line2), "IP:%s", status.ip);
    snprintf(line3, sizeof(line3), "GW:%s", status.gateway);
    if (status.mode == NETWORK_MODE_STA) {
        if (status.has_rssi) {
            snprintf(line4, sizeof(line4), "RSSI:%d CH:%u", status.rssi, status.channel);
        } else {
            snprintf(line4, sizeof(line4), "RSSI:- CH:%u", status.channel);
        }
    } else {
        snprintf(line4, sizeof(line4), "CLIENTS:%u CH:%u", status.connected_clients, status.channel);
    }

    oled_draw_text(framebuffer, 0, 0, line1);
    oled_draw_text(framebuffer, 0, 16, line2);
    oled_draw_text(framebuffer, 0, 32, line3);
    oled_draw_text(framebuffer, 0, 48, line4);

    return oled_flush(framebuffer);
}

static void IRAM_ATTR oled_button_isr_handler(void *arg)
{
    (void)arg;
    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - s_button_last_tick) >= pdMS_TO_TICKS(200)) {
        s_button_last_tick = now;
        portENTER_CRITICAL_ISR(&s_button_lock);
        s_button_press_count++;
        portEXIT_CRITICAL_ISR(&s_button_lock);
    }
}

static esp_err_t oled_button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << OLED_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return gpio_isr_handler_add(OLED_BUTTON_GPIO, oled_button_isr_handler, NULL);
}

static void display_task(void *pvParameter)
{
    (void)pvParameter;

    while (1) {
        uint32_t button_presses = 0;
        portENTER_CRITICAL(&s_button_lock);
        button_presses = s_button_press_count;
        s_button_press_count = 0;
        portEXIT_CRITICAL(&s_button_lock);

        for (uint32_t i = 0; i < button_presses; i++) {
            oled_view_t previous_view = s_oled_view;
            s_oled_view = (oled_view_t)((s_oled_view + 1) % 3);

            if (previous_view == OLED_VIEW_OFF || s_oled_view == OLED_VIEW_OFF) {
                esp_err_t pwr_err = oled_set_power(s_oled_view != OLED_VIEW_OFF);
                if (pwr_err != ESP_OK) {
                    ESP_LOGW(TAG, "OLED power change failed: %s", esp_err_to_name(pwr_err));
                }
            }

            if (s_oled_view == OLED_VIEW_OFF) {
                oled_clear();
            }

            ESP_LOGI(TAG, "OLED view changed to %s",
                     s_oled_view == OLED_VIEW_AIR_QUALITY ? "air quality" :
                     s_oled_view == OLED_VIEW_NETWORK ? "network" : "off");
        }

        if (s_oled_available && s_oled_view == OLED_VIEW_NETWORK) {
            esp_err_t draw_err = oled_render_network();
            if (draw_err != ESP_OK) {
                ESP_LOGW(TAG, "OLED network render failed: %s", esp_err_to_name(draw_err));
            }
        } else if (s_oled_available && s_oled_view == OLED_VIEW_AIR_QUALITY) {
            esp_err_t pwr_err = oled_set_power(true);
            if (pwr_err != ESP_OK) {
                ESP_LOGW(TAG, "OLED power on failed: %s", esp_err_to_name(pwr_err));
            }

            sensor_log_entry_t snapshot = {0};
            bool has_data = false;
            if (s_latest_entry_mutex && xSemaphoreTake(s_latest_entry_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                snapshot = s_latest_entry;
                has_data = s_latest_entry_valid;
                xSemaphoreGive(s_latest_entry_mutex);
            }

            if (has_data) {
                esp_err_t draw_err = oled_render_entry(&snapshot);
                if (draw_err != ESP_OK) {
                    ESP_LOGW(TAG, "OLED render failed: %s", esp_err_to_name(draw_err));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t oled_display_init(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }

    s_latest_entry_mutex = xSemaphoreCreateMutex();
    if (!s_latest_entry_mutex) {
        ESP_LOGW(TAG, "Failed to create latest-entry mutex; OLED updates may be skipped");
    }

    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &oled_cfg, &s_oled_dev);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };

    for (size_t i = 0; i < sizeof(init_seq); i++) {
        err = oled_send_command(init_seq[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    err = oled_clear();
    if (err != ESP_OK) {
        return err;
    }

    s_oled_available = true;
    ESP_LOGI(TAG, "SSD1306 OLED initialized at 0x%02X", OLED_I2C_ADDR);

    err = oled_button_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Button init failed on GPIO %d: %s", OLED_BUTTON_GPIO, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Button configured on GPIO %d (active-low, press to cycle OLED views)", OLED_BUTTON_GPIO);
    }

    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);
    return ESP_OK;
}

void oled_display_publish_entry(const sensor_log_entry_t *entry)
{
    if (!entry || !s_latest_entry_mutex) {
        return;
    }

    if (xSemaphoreTake(s_latest_entry_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_latest_entry = *entry;
        s_latest_entry_valid = true;
        xSemaphoreGive(s_latest_entry_mutex);
    }
}

bool oled_display_is_available(void)
{
    return s_oled_available;
}
