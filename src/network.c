#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "mdns.h"

#include "app_config.h"
#include "secrets.h"

#include "network.h"

static const char *TAG = "NET";
static const char *TAG_TIME = "TIME";
static const char *TAG_MDNS = "MDNS";

static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_sta_active = false;
static bool s_mdns_active = false;
static char s_access_url[64] = "http://192.168.4.1/";

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

static esp_err_t wifi_init_once(void)
{
    esp_err_t err = network_stack_init_once();
    if (err != ESP_OK) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return esp_wifi_set_storage(WIFI_STORAGE_RAM);
}

static bool system_time_valid(void)
{
    time_t now = 0;
    time(&now);
    return now > 1700000000;
}

static esp_err_t network_sync_time(void)
{
    int waited_ms = 0;

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, NTP_SERVER);
    esp_sntp_init();

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
    return ESP_OK;
}

static esp_err_t network_start_mdns(void)
{
    if (s_mdns_active) {
        mdns_free();
        s_mdns_active = false;
    }

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        return err;
    }

    err = mdns_hostname_set(MDNS_HOSTNAME);
    if (err != ESP_OK) {
        mdns_free();
        return err;
    }

    err = mdns_instance_name_set(MDNS_INSTANCE_NAME);
    if (err != ESP_OK) {
        mdns_free();
        return err;
    }

    err = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    if (err != ESP_OK) {
        mdns_free();
        return err;
    }

    s_mdns_active = true;
    snprintf(s_access_url, sizeof(s_access_url), "http://%s.local/", MDNS_HOSTNAME);
    ESP_LOGI(TAG_MDNS, "mDNS started: %s", s_access_url);
    return ESP_OK;
}

static esp_err_t network_try_start_sta(void)
{
    if (strlen(WIFI_STA_SSID) == 0) {
        ESP_LOGI(TAG, "STA credentials not set, skip external WiFi");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = wifi_init_once();
    if (err != ESP_OK) {
        return err;
    }

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif) {
            return ESP_FAIL;
        }
    }

    wifi_config_t sta_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)sta_cfg.sta.ssid, WIFI_STA_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_STA_PASS, sizeof(sta_cfg.sta.password) - 1);

    ESP_LOGI(TAG, "Connecting to external WiFi SSID=%s", WIFI_STA_SSID);
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
        ESP_LOGW(TAG, "STA connect timeout, falling back to SoftAP");
        esp_wifi_disconnect();
        esp_wifi_stop();
        return ESP_ERR_TIMEOUT;
    }

    esp_netif_ip_info_t ip_info = {0};
    err = esp_netif_get_ip_info(s_sta_netif, &ip_info);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "STA connected: IP=" IPSTR " GW=" IPSTR, IP2STR(&ip_info.ip), IP2STR(&ip_info.gw));
    }

    s_sta_active = true;
    network_sync_time();

    err = network_start_mdns();
    if (err != ESP_OK) {
        ESP_LOGW(TAG_MDNS, "mDNS start failed: %s", esp_err_to_name(err));
        if (ip_info.ip.addr != 0) {
            snprintf(s_access_url, sizeof(s_access_url), "http://" IPSTR "/", IP2STR(&ip_info.ip));
        }
    }

    return ESP_OK;
}

void network_configure_timezone_jst(void)
{
    setenv("TZ", "JST-9", 1);
    tzset();
}

esp_err_t network_start_softap(void)
{
    esp_err_t err = wifi_init_once();
    if (err != ESP_OK) {
        return err;
    }

    if (s_mdns_active) {
        mdns_free();
        s_mdns_active = false;
    }

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (!s_ap_netif) {
            return ESP_FAIL;
        }
    }

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

    s_sta_active = false;
    snprintf(s_access_url, sizeof(s_access_url), "http://192.168.4.1/");
    ESP_LOGI(TAG, "WiFi AP started: SSID=%s CH=%d", WIFI_AP_SSID, WIFI_AP_CHANNEL);
    return ESP_OK;
}

esp_err_t network_start(void)
{
    esp_err_t err = network_try_start_sta();
    if (err == ESP_OK) {
        return ESP_OK;
    }

    return network_start_softap();
}

bool network_is_sta_active(void)
{
    return s_sta_active;
}

const char *network_get_access_url(void)
{
    return s_access_url;
}
