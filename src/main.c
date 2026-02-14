#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Logging / errors
#include "esp_err.h"
#include "esp_log.h"

// ESP-IDF 5.x I2C master-bus API (IMPORTANT)
#include "driver/i2c_master.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

// K0I05 components
#include "ahtxx.h"
#include "ens160.h"
#include "secrets.h"

// EEPROM

static const char *TAG = "MAIN";
static httpd_handle_t s_http_server = NULL;

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    6
#define I2C_SCL_GPIO    7
#define I2C_FREQ_HZ     100000

#define NVS_NAMESPACE       "sensorlog"
#define NVS_SLOT_COUNT      180

#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4
#define NTP_SERVER          "pool.ntp.org"
#define STA_CONNECT_TIMEOUT_MS 20000
#define NTP_SYNC_TIMEOUT_MS    20000

#define SENSOR_READ_INTERVAL_MS 180000 // 3 minutes

typedef struct {
    ahtxx_handle_t aht;
    ens160_handle_t ens;
    nvs_handle_t nvs;
    bool nvs_ready;
} sensors_t;

typedef struct {
    uint32_t unix_time;
    int16_t temperature_c_x100;
    int16_t humidity_pct_x100;
    uint16_t eco2_ppm;
    uint16_t tvoc_ppb;
    uint8_t aqi;
    uint8_t ens_validity;
    uint8_t has_aht;
    uint8_t has_ens;
} sensor_log_entry_t;

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

static void configure_timezone_jst(void)
{
    setenv("TZ", "JST-9", 1);
    tzset();
}

static esp_err_t sync_time_via_sta(void)
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
        ESP_LOGW(TAG, "STA connect timeout, skip NTP sync");
        esp_wifi_stop();
        esp_wifi_deinit();
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
        ESP_LOGI(TAG, "NTP time synced: %lu", (unsigned long)now);
    } else {
        ESP_LOGW(TAG, "NTP sync timeout");
    }

    esp_sntp_stop();
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    return ESP_OK;
}

static esp_err_t wifi_start_softap(void)
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

static esp_err_t nvs_init_storage(nvs_handle_t *out_nvs)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    return nvs_open(NVS_NAMESPACE, NVS_READWRITE, out_nvs);
}

static esp_err_t nvs_log_sensor_sample(nvs_handle_t nvs, const sensor_log_entry_t *entry)
{
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
    if (err != ESP_OK) {
        return err;
    }

    wr_idx = (wr_idx + 1) % NVS_SLOT_COUNT;
    if (count < NVS_SLOT_COUNT) {
        count++;
    }

    err = nvs_set_u32(nvs, "wr_idx", wr_idx);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u32(nvs, "count", count);
    if (err != ESP_OK) {
        return err;
    }

    return nvs_commit(nvs);
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
        ESP_LOGI(TAG, "NVS stats: used=%u free=%u total=%u namespaces=%u",
                 (unsigned)stats.used_entries,
                 (unsigned)stats.free_entries,
                 (unsigned)stats.total_entries,
                 (unsigned)stats.namespace_count);
    } else {
        ESP_LOGW(TAG, "Failed to read NVS stats: %s", esp_err_to_name(err));
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

static void format_unix_time(uint32_t unix_time, char *buf, size_t buf_len)
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

static esp_err_t logs_page_get_handler(httpd_req_t *req)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "NVS unavailable");
        return ESP_FAIL;
    }

    uint32_t wr_idx = 0;
    uint32_t count = 0;
    err = nvs_get_u32_default(nvs, "wr_idx", 0, &wr_idx);
    if (err == ESP_OK) {
        err = nvs_get_u32_default(nvs, "count", 0, &count);
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "Failed to read log metadata");
        return ESP_FAIL;
    }

    if (count > NVS_SLOT_COUNT) {
        count = NVS_SLOT_COUNT;
    }

#define SEND_CHUNK_OR_EXIT(chunk_literal_or_buf)                     \
    do {                                                              \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf));  \
        if (err != ESP_OK) {                                          \
            goto page_exit;                                           \
        }                                                             \
    } while (0)

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    SEND_CHUNK_OR_EXIT(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Sensor Logs</title>"
        "<style>body{font-family:Arial,sans-serif;margin:12px}"
        ".charts{display:grid;grid-template-columns:1fr;gap:10px;max-width:900px}"
        "canvas{border:1px solid #999;background:#fff}</style></head><body>"
        "<h1>Sensor Logs</h1>"
        "<p>Live update: every 5s | JSON: <a href='/json'>/json</a></p>");

    SEND_CHUNK_OR_EXIT(
        "<div class='charts'>"
        "<div><b>Temperature (C)</b><br><canvas id='chart_temp' width='860' height='180'></canvas></div>"
        "<div><b>Humidity (%)</b><br><canvas id='chart_rh' width='860' height='180'></canvas></div>"
        "<div><b>AQI</b><br><canvas id='chart_aqi' width='860' height='180'></canvas></div>"
        "<div><b>TVOC (ppb)</b><br><canvas id='chart_tvoc' width='860' height='180'></canvas></div>"
        "<div><b>eCO2 (ppm)</b><br><canvas id='chart_eco2' width='860' height='180'></canvas></div>"
        "<div><b>ENS validity</b><br><canvas id='chart_validity' width='860' height='180'></canvas></div>"
        "</div><br>");

    char meta[160];
    snprintf(meta, sizeof(meta), "<p>Stored entries: %lu / %u</p>", (unsigned long)count, NVS_SLOT_COUNT);
    SEND_CHUNK_OR_EXIT(meta);

    SEND_CHUNK_OR_EXIT(
        "<table border='1' cellspacing='0' cellpadding='6'>"
        "<tr><th>#</th><th>Time</th><th>T(C)</th><th>RH(%)</th><th>AQI</th><th>TVOC(ppb)</th><th>eCO2(ppm)</th><th>ENS validity</th></tr>");

    uint32_t start = (wr_idx + NVS_SLOT_COUNT - count) % NVS_SLOT_COUNT;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (start + i) % NVS_SLOT_COUNT;
        char key[8];
        snprintf(key, sizeof(key), "r%03lu", (unsigned long)slot);

        sensor_log_entry_t entry = {0};
        if (!nvs_read_entry_compat(nvs, key, &entry)) {
            continue;
        }

        char row[400];
        char time_str[32];
        format_unix_time(entry.unix_time, time_str, sizeof(time_str));
        if (entry.has_aht) {
            if (entry.has_ens) {
                snprintf(row, sizeof(row),
                         "<tr><td>%lu</td><td>%s</td><td>%.2f</td><td>%.2f</td><td>%u</td><td>%u</td><td>%u</td><td>%u</td></tr>",
                         (unsigned long)(i + 1),
                         time_str,
                         entry.temperature_c_x100 / 100.0f,
                         entry.humidity_pct_x100 / 100.0f,
                         entry.aqi,
                         entry.tvoc_ppb,
                         entry.eco2_ppm,
                         entry.ens_validity);
            } else {
                snprintf(row, sizeof(row),
                         "<tr><td>%lu</td><td>%s</td><td>%.2f</td><td>%.2f</td><td>-</td><td>-</td><td>-</td><td>%u</td></tr>",
                         (unsigned long)(i + 1),
                         time_str,
                         entry.temperature_c_x100 / 100.0f,
                         entry.humidity_pct_x100 / 100.0f,
                         entry.ens_validity);
            }
        } else {
            if (entry.has_ens) {
                snprintf(row, sizeof(row),
                         "<tr><td>%lu</td><td>%s</td><td>-</td><td>-</td><td>%u</td><td>%u</td><td>%u</td><td>%u</td></tr>",
                         (unsigned long)(i + 1),
                         time_str,
                         entry.aqi,
                         entry.tvoc_ppb,
                         entry.eco2_ppm,
                         entry.ens_validity);
            } else {
                snprintf(row, sizeof(row),
                         "<tr><td>%lu</td><td>%s</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>%u</td></tr>",
                         (unsigned long)(i + 1),
                         time_str,
                         entry.ens_validity);
            }
        }
        SEND_CHUNK_OR_EXIT(row);
    }

    SEND_CHUNK_OR_EXIT(
        "</table>"
        "<script>"
        "function drawLine(canvasId,name,color,values,timeLabels){"
        "const c=document.getElementById(canvasId);if(!c)return;"
        "const ctx=c.getContext('2d');const w=c.width,h=c.height,p=32;"
        "ctx.clearRect(0,0,w,h);ctx.fillStyle='#fff';ctx.fillRect(0,0,w,h);"
        "ctx.strokeStyle='#bbb';ctx.strokeRect(p,p,w-2*p,h-2*p);"
        "let min=Infinity,max=-Infinity,maxN=values.length;"
        "for(const v of values){if(v==null)continue;min=Math.min(min,v);max=Math.max(max,v);}"
        "if(!isFinite(min)||!isFinite(max)||maxN<2){ctx.fillStyle='#444';ctx.fillText('Not enough data',p+8,p+18);return;}"
        "if(min===max){min=min-1;max=max+1;}"
        "const sx=(w-2*p)/(maxN-1);const sy=(h-2*p)/(max-min);"
        "ctx.fillStyle='#333';ctx.font='12px Arial';ctx.fillText(max.toFixed(2),4,p+8);ctx.fillText(min.toFixed(2),4,h-p);"
        "ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();let started=false;"
        "for(let i=0;i<values.length;i++){const v=values[i];if(v==null)continue;const x=p+i*sx;const y=h-p-(v-min)*sy;if(!started){ctx.moveTo(x,y);started=true;}else{ctx.lineTo(x,y);}}ctx.stroke();"
        "ctx.fillStyle='#666';ctx.font='10px Arial';"
        "const maxLabels=6;const step=Math.max(1,Math.ceil(maxN/maxLabels));"
        "for(let i=0;i<maxN;i+=step){const lbl=timeLabels&&timeLabels[i]?timeLabels[i]:null;if(!lbl)continue;const x=p+i*sx;ctx.fillText(lbl,Math.max(0,x-14),h-8);}"
        "if(timeLabels&&timeLabels[maxN-1]){const x=p+(maxN-1)*sx;ctx.fillText(timeLabels[maxN-1],Math.max(0,x-14),h-8);}"
        "ctx.fillStyle=color;ctx.fillRect(w-140,6,10,10);ctx.fillStyle='#222';ctx.fillText(name,w-125,15);"
        "}"
        "async function refreshCharts(){"
        "try{const r=await fetch('/json');const d=await r.json();const e=d.entries||[];"
        "const fmt=new Intl.DateTimeFormat('ja-JP',{hour:'2-digit',minute:'2-digit',hour12:false,timeZone:'Asia/Tokyo'});"
        "const timeLabels=e.map(x=>x.unix_time?fmt.format(new Date(x.unix_time*1000)):null);"
        "const temp=e.map(x=>x.has_aht?x.temperature_c:null);"
        "const rh=e.map(x=>x.has_aht?x.humidity_pct:null);"
        "const aqi=e.map(x=>x.has_ens?x.aqi:null);"
        "const tvoc=e.map(x=>x.has_ens?x.tvoc_ppb:null);"
        "const eco2=e.map(x=>x.has_ens?x.eco2_ppm:null);"
        "const validity=e.map(x=>x.ens_validity);"
        "drawLine('chart_temp','Temp C','#d14',temp,timeLabels);"
        "drawLine('chart_rh','RH %','#06c',rh,timeLabels);"
        "drawLine('chart_aqi','AQI','#1a7f37',aqi,timeLabels);"
        "drawLine('chart_tvoc','TVOC','#a36a00',tvoc,timeLabels);"
        "drawLine('chart_eco2','eCO2','#6f42c1',eco2,timeLabels);"
        "drawLine('chart_validity','ENS validity','#444',validity,timeLabels);"
        "}catch(_e){}}"
        "refreshCharts();setInterval(refreshCharts,5000);"
        "</script>"
        "</body></html>");

    err = ESP_OK;

page_exit:
    nvs_close(nvs);
    if (err == ESP_OK) {
        esp_err_t end_err = httpd_resp_sendstr_chunk(req, NULL);
        if (end_err != ESP_OK) {
            err = end_err;
        }
    } else {
        ESP_LOGW(TAG, "HTTP page send aborted: %s", esp_err_to_name(err));
    }
#undef SEND_CHUNK_OR_EXIT
    return err;
}

static esp_err_t logs_json_get_handler(httpd_req_t *req)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"NVS unavailable\"}");
        return ESP_FAIL;
    }

    uint32_t wr_idx = 0;
    uint32_t count = 0;
    err = nvs_get_u32_default(nvs, "wr_idx", 0, &wr_idx);
    if (err == ESP_OK) {
        err = nvs_get_u32_default(nvs, "count", 0, &count);
    }
    if (err != ESP_OK) {
        nvs_close(nvs);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"Failed to read log metadata\"}");
        return ESP_FAIL;
    }

    if (count > NVS_SLOT_COUNT) {
        count = NVS_SLOT_COUNT;
    }

#define SEND_JSON_CHUNK_OR_EXIT(chunk_literal_or_buf)                \
    do {                                                              \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf));  \
        if (err != ESP_OK) {                                          \
            goto json_exit;                                           \
        }                                                             \
    } while (0)

    httpd_resp_set_type(req, "application/json");

    char head[96];
    snprintf(head, sizeof(head), "{\"count\":%lu,\"capacity\":%u,\"entries\":[",
             (unsigned long)count, NVS_SLOT_COUNT);
    SEND_JSON_CHUNK_OR_EXIT(head);

    uint32_t start = (wr_idx + NVS_SLOT_COUNT - count) % NVS_SLOT_COUNT;
    bool first = true;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t slot = (start + i) % NVS_SLOT_COUNT;
        char key[8];
        snprintf(key, sizeof(key), "r%03lu", (unsigned long)slot);

        sensor_log_entry_t entry = {0};
        if (!nvs_read_entry_compat(nvs, key, &entry)) {
            continue;
        }

        if (!first) {
            SEND_JSON_CHUNK_OR_EXIT(",");
        }
        first = false;

        char item[320];
        snprintf(item, sizeof(item),
                 "{\"index\":%lu,\"unix_time\":%lu,\"has_aht\":%u,\"temperature_c\":%.2f,\"humidity_pct\":%.2f,"
                 "\"has_ens\":%u,\"aqi\":%u,\"tvoc_ppb\":%u,\"eco2_ppm\":%u,\"ens_validity\":%u}",
                 (unsigned long)(i + 1),
                 (unsigned long)entry.unix_time,
                 entry.has_aht,
                 entry.temperature_c_x100 / 100.0f,
                 entry.humidity_pct_x100 / 100.0f,
                 entry.has_ens,
                 entry.aqi,
                 entry.tvoc_ppb,
                 entry.eco2_ppm,
                 entry.ens_validity);
        SEND_JSON_CHUNK_OR_EXIT(item);
    }

    SEND_JSON_CHUNK_OR_EXIT("]}");

    err = ESP_OK;

json_exit:
    nvs_close(nvs);
    if (err == ESP_OK) {
        esp_err_t end_err = httpd_resp_sendstr_chunk(req, NULL);
        if (end_err != ESP_OK) {
            err = end_err;
        }
    } else {
        ESP_LOGW(TAG, "HTTP json send aborted: %s", esp_err_to_name(err));
    }
#undef SEND_JSON_CHUNK_OR_EXIT
    return err;
}

static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;

    esp_err_t err = httpd_start(&s_http_server, &config);
    if (err != ESP_OK) {
        return err;
    }

    httpd_uri_t logs_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = logs_page_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t logs_json_uri = {
        .uri = "/json",
        .method = HTTP_GET,
        .handler = logs_json_get_handler,
        .user_ctx = NULL,
    };

    err = httpd_register_uri_handler(s_http_server, &logs_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &logs_json_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Web server started on http://192.168.4.1/");
    return ESP_OK;
}

static void loop_task(void *pvParameter)
{
    sensors_t *s = (sensors_t *)pvParameter;

    while (1) {
        sensor_log_entry_t log_entry = {0};
        time_t now = 0;
        time(&now);
        if (now > 0) {
            log_entry.unix_time = (uint32_t)now;
        }

        // ---- AHT21 (temperature / humidity) ----
        float t_c = 0.0f, rh = 0.0f;
        esp_err_t err = ahtxx_get_measurement(s->aht, &t_c, &rh);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "AHT: T=%.2f C  RH=%.2f %%", t_c, rh);
            log_entry.has_aht = 1;
            log_entry.temperature_c_x100 = (int16_t)(t_c * 100.0f);
            log_entry.humidity_pct_x100 = (int16_t)(rh * 100.0f);
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
                log_entry.has_ens = 1;
                log_entry.ens_validity = (uint8_t)flag;
                log_entry.aqi = aq.uba_aqi;
                log_entry.tvoc_ppb = aq.tvoc;
                log_entry.eco2_ppm = aq.eco2;
            } else {
                ESP_LOGW(TAG, "ENS read failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGI(TAG, "ENS not ready yet (validity=%d)", (int)flag);
            log_entry.ens_validity = (uint8_t)flag;
        }

        if (s->nvs_ready) {
            err = nvs_log_sensor_sample(s->nvs, &log_entry);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "NVS log write failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C + AHT21 + ENS160");
    configure_timezone_jst();

    nvs_handle_t nvs = 0;
    bool nvs_ready = false;
    esp_err_t err = nvs_init_storage(&nvs);
    if (err == ESP_OK) {
        nvs_ready = true;
        ESP_LOGI(TAG, "NVS logging enabled (%s)", NVS_NAMESPACE);
        nvs_log_stats();
    } else {
        ESP_LOGW(TAG, "NVS init failed, continue without persistence: %s", esp_err_to_name(err));
    }

    err = sync_time_via_sta();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Time sync before AP failed: %s", esp_err_to_name(err));
    }

    err = wifi_start_softap();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi AP start failed: %s", esp_err_to_name(err));
    } else {
        err = start_webserver();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Web server start failed: %s", esp_err_to_name(err));
        }
    }

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
    sensors.nvs = nvs;
    sensors.nvs_ready = nvs_ready;

    xTaskCreate(loop_task, "loop_task", 4096, &sensors, 5, NULL);
}
