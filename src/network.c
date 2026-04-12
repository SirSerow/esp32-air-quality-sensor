#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"

#include "app_config.h"
#include "secrets.h"

#include "network.h"

static const char *TAG = "NET";
static const char *TAG_TIME = "TIME";

static esp_err_t network_stack_init_once(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return ESP_OK;
}

static bool system_time_valid(void)
{
    time_t now = 0;
    time(&now);
    return now > 1700000000;
}

void network_configure_timezone_jst(void)
{
    setenv("TZ", "JST-9", 1);
    tzset();
}

esp_err_t network_sync_time_via_sta(void)
{
    if (strlen(WIFI_STA_SSID) == 0) {
        ESP_LOGI(TAG, "STA credentials not set, skip NTP sync");
        return ESP_OK;
    }

    esp_err_t err = network_stack_init_once();
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t sta_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASS, sizeof(sta_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    int waited_ms = 0;
    bool connected = false;
    while (waited_ms < STA_CONNECT_TIMEOUT_MS) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        waited_ms += 500;
    }

    if (!connected) {
        ESP_LOGW(TAG_TIME, "STA connect timeout, skip NTP sync");
        esp_wifi_stop();
        esp_wifi_deinit();
        esp_netif_destroy(sta_netif);
        return ESP_ERR_TIMEOUT;
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_init();

    waited_ms = 0;
    while (waited_ms < NTP_SYNC_TIMEOUT_MS && !system_time_valid()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        waited_ms += 500;
    }

    if (system_time_valid()) {
        time_t now;
        time(&now);
        ESP_LOGI(TAG_TIME, "NTP time synced: %lu", (unsigned long)now);
    } else {
        ESP_LOGW(TAG_TIME, "NTP sync timeout");
    }

    esp_sntp_stop();
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_netif_destroy(sta_netif);
    return ESP_OK;
}

esp_err_t network_start_softap(void)
{
    esp_err_t err = network_stack_init_once();
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (!ap_netif) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    strncpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char *)ap_config.ap.password, WIFI_AP_PASS, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);

    if (strlen(WIFI_AP_PASS) < 8) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
        ap_config.ap.password[0] = '\0';
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s CH=%d", WIFI_AP_SSID, WIFI_AP_CHANNEL);
    return ESP_OK;
}
