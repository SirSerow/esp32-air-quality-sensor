#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "app_config.h"

#include "network.h"
#include "storage.h"
#include "web_server.h"

static const char *TAG = "WEB";
static httpd_handle_t s_http_server = NULL;
static const storage_ctx_t *s_storage = NULL;

typedef enum {
    VALUE_SEVERITY_OK = 0,
    VALUE_SEVERITY_WARN,
    VALUE_SEVERITY_BAD,
    VALUE_SEVERITY_NA,
} value_severity_t;

static const char *severity_css_class(value_severity_t severity)
{
    switch (severity) {
    case VALUE_SEVERITY_OK: return "v-ok";
    case VALUE_SEVERITY_WARN: return "v-warn";
    case VALUE_SEVERITY_BAD: return "v-bad";
    case VALUE_SEVERITY_NA:
    default: return "v-na";
    }
}

static value_severity_t temperature_severity(int16_t temp_x100)
{
    if (temp_x100 >= TEMP_OK_MIN_X100 && temp_x100 <= TEMP_OK_MAX_X100) {
        return VALUE_SEVERITY_OK;
    }
    if (temp_x100 >= TEMP_WARN_MIN_X100 && temp_x100 <= TEMP_WARN_MAX_X100) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static value_severity_t humidity_severity(int16_t rh_x100)
{
    if (rh_x100 >= RH_OK_MIN_X100 && rh_x100 <= RH_OK_MAX_X100) {
        return VALUE_SEVERITY_OK;
    }
    if (rh_x100 >= RH_WARN_MIN_X100 && rh_x100 <= RH_WARN_MAX_X100) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static value_severity_t aqi_severity(uint8_t aqi)
{
    if (aqi <= 2) {
        return VALUE_SEVERITY_OK;
    }
    if (aqi == 3) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static value_severity_t tvoc_severity(uint16_t tvoc)
{
    if (tvoc <= TVOC_OK_MAX) {
        return VALUE_SEVERITY_OK;
    }
    if (tvoc <= TVOC_WARN_MAX) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static value_severity_t eco2_severity(uint16_t eco2)
{
    if (eco2 <= ECO2_OK_MAX) {
        return VALUE_SEVERITY_OK;
    }
    if (eco2 <= ECO2_WARN_MAX) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static value_severity_t ens_validity_severity(uint8_t validity)
{
    if (validity == 0) {
        return VALUE_SEVERITY_OK;
    }
    if (validity == 1) {
        return VALUE_SEVERITY_WARN;
    }
    return VALUE_SEVERITY_BAD;
}

static const char *storage_source_name(bool from_sd)
{
    return from_sd ? "sd" : "nvs";
}

static esp_err_t send_json_error(httpd_req_t *req, const char *status, const char *message)
{
    char body[160];
    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t send_json_entry_chunk(httpd_req_t *req, const sensor_log_entry_t *entry, size_t index)
{
    char item[320];
    snprintf(item, sizeof(item),
             "{\"index\":%lu,\"unix_time\":%lu,\"has_aht\":%u,\"temperature_c\":%.2f,\"humidity_pct\":%.2f,"
             "\"has_ens\":%u,\"aqi\":%u,\"tvoc_ppb\":%u,\"eco2_ppm\":%u,\"ens_validity\":%u}",
             (unsigned long)index,
             (unsigned long)entry->unix_time,
             entry->has_aht,
             entry->temperature_c_x100 / 100.0f,
             entry->humidity_pct_x100 / 100.0f,
             entry->has_ens,
             entry->aqi,
             entry->tvoc_ppb,
             entry->eco2_ppm,
             entry->ens_validity);
    return httpd_resp_sendstr_chunk(req, item);
}

static char *alloc_query_string(httpd_req_t *req)
{
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len == 0) {
        return NULL;
    }

    char *query = calloc(query_len + 1, sizeof(char));
    if (!query) {
        return NULL;
    }

    if (httpd_req_get_url_query_str(req, query, query_len + 1) != ESP_OK) {
        free(query);
        return NULL;
    }

    return query;
}

static bool parse_u32_string(const char *value, uint32_t *out_value)
{
    if (!value || !*value || !out_value) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (*end != '\0' || parsed > UINT32_MAX) {
        return false;
    }

    *out_value = (uint32_t)parsed;
    return true;
}

static bool parse_size_string(const char *value, size_t *out_value)
{
    if (!value || !*value || !out_value) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (*end != '\0') {
        return false;
    }

    *out_value = (size_t)parsed;
    return true;
}

static esp_err_t query_get_optional_u32(const char *query, const char *key, bool *present, uint32_t *value)
{
    if (present) {
        *present = false;
    }
    if (!query) {
        return ESP_OK;
    }

    char raw[24];
    if (httpd_query_key_value(query, key, raw, sizeof(raw)) != ESP_OK) {
        return ESP_OK;
    }

    if (!parse_u32_string(raw, value)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (present) {
        *present = true;
    }
    return ESP_OK;
}

static esp_err_t query_get_optional_limit(const char *query,
                                          const char *key,
                                          size_t default_value,
                                          size_t max_value,
                                          size_t *out_value)
{
    if (!out_value) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_value = default_value;
    if (!query) {
        return ESP_OK;
    }

    char raw[24];
    if (httpd_query_key_value(query, key, raw, sizeof(raw)) != ESP_OK) {
        return ESP_OK;
    }

    size_t parsed = 0;
    if (!parse_size_string(raw, &parsed) || parsed == 0 || parsed > max_value) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_value = parsed;
    return ESP_OK;
}

static esp_err_t api_status_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    sensor_log_entry_t latest = {0};
    bool from_sd = false;
    bool has_latest = storage_load_latest_entry(s_storage, &latest, true, &from_sd);

    time_t now = 0;
    time(&now);

    esp_err_t err = httpd_resp_sendstr_chunk(req, "{");
    if (err != ESP_OK) {
        return err;
    }

    char head[256];
    snprintf(head,
             sizeof(head),
             "\"device_time_unix\":%lu,\"sample_interval_sec\":%u,\"storage_source\":\"%s\","
             "\"sd_ready\":%s,\"nvs_ready\":%s,\"latest_record\":",
             (unsigned long)(now > 0 ? now : 0),
             SENSOR_READ_INTERVAL_MS / 1000U,
             has_latest ? storage_source_name(from_sd) : storage_source_name(s_storage->sd_ready),
             s_storage->sd_ready ? "true" : "false",
             s_storage->nvs_ready ? "true" : "false");
    err = httpd_resp_sendstr_chunk(req, head);
    if (err != ESP_OK) {
        return err;
    }

    if (has_latest) {
        err = send_json_entry_chunk(req, &latest, 1);
    } else {
        err = httpd_resp_sendstr_chunk(req, "null");
    }
    if (err != ESP_OK) {
        return err;
    }

    err = httpd_resp_sendstr_chunk(req, "}");
    if (err != ESP_OK) {
        return err;
    }

    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t api_records_get_handler(httpd_req_t *req)
{
    char *query = alloc_query_string(req);
    if (httpd_req_get_url_query_len(req) > 0 && !query) {
        return send_json_error(req, "500 Internal Server Error", "Query parsing failed");
    }

    bool has_from = false;
    bool has_to = false;
    uint32_t from_unix_time = 0;
    uint32_t to_unix_time = 0;
    size_t limit = API_DEFAULT_RECORD_LIMIT;

    esp_err_t err = query_get_optional_u32(query, "from", &has_from, &from_unix_time);
    if (err == ESP_OK) {
        err = query_get_optional_u32(query, "to", &has_to, &to_unix_time);
    }
    if (err == ESP_OK) {
        err = query_get_optional_limit(query, "limit", API_DEFAULT_RECORD_LIMIT, API_MAX_RECORDS_PER_RESPONSE, &limit);
    }
    if (err != ESP_OK) {
        free(query);
        return send_json_error(req, "400 Bad Request", "Invalid query parameters");
    }
    free(query);

    if (has_from && has_to && from_unix_time > to_unix_time) {
        return send_json_error(req, "400 Bad Request", "The from parameter must be <= to");
    }

    sensor_log_entry_t *entries = calloc(limit, sizeof(sensor_log_entry_t));
    if (!entries) {
        return send_json_error(req, "500 Internal Server Error", "Out of memory");
    }

    bool from_sd = false;
    bool has_more = false;
    bool newest_only = !has_from && !has_to;
    size_t count = storage_load_filtered_entries(s_storage,
                                                 entries,
                                                 limit,
                                                 newest_only,
                                                 true,
                                                 has_from,
                                                 from_unix_time,
                                                 has_to,
                                                 to_unix_time,
                                                 &from_sd,
                                                 &has_more);

    httpd_resp_set_type(req, "application/json");

#define SEND_API_JSON_OR_EXIT(chunk_literal_or_buf)                 \
    do {                                                            \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf));\
        if (err != ESP_OK) {                                        \
            goto records_exit;                                      \
        }                                                           \
    } while (0)

    char head[256];
    if (has_from && has_to) {
        snprintf(head,
                 sizeof(head),
                 "{\"count\":%lu,\"limit\":%lu,\"has_more\":%s,\"source\":\"%s\","
                 "\"from\":%lu,\"to\":%lu,\"entries\":[",
                 (unsigned long)count,
                 (unsigned long)limit,
                 has_more ? "true" : "false",
                 storage_source_name(from_sd),
                 (unsigned long)from_unix_time,
                 (unsigned long)to_unix_time);
    } else if (has_from) {
        snprintf(head,
                 sizeof(head),
                 "{\"count\":%lu,\"limit\":%lu,\"has_more\":%s,\"source\":\"%s\","
                 "\"from\":%lu,\"to\":null,\"entries\":[",
                 (unsigned long)count,
                 (unsigned long)limit,
                 has_more ? "true" : "false",
                 storage_source_name(from_sd),
                 (unsigned long)from_unix_time);
    } else if (has_to) {
        snprintf(head,
                 sizeof(head),
                 "{\"count\":%lu,\"limit\":%lu,\"has_more\":%s,\"source\":\"%s\","
                 "\"from\":null,\"to\":%lu,\"entries\":[",
                 (unsigned long)count,
                 (unsigned long)limit,
                 has_more ? "true" : "false",
                 storage_source_name(from_sd),
                 (unsigned long)to_unix_time);
    } else {
        snprintf(head,
                 sizeof(head),
                 "{\"count\":%lu,\"limit\":%lu,\"has_more\":%s,\"source\":\"%s\","
                 "\"from\":null,\"to\":null,\"entries\":[",
                 (unsigned long)count,
                 (unsigned long)limit,
                 has_more ? "true" : "false",
                 storage_source_name(from_sd));
    }
    SEND_API_JSON_OR_EXIT(head);

    for (size_t i = 0; i < count; i++) {
        if (i != 0) {
            SEND_API_JSON_OR_EXIT(",");
        }
        err = send_json_entry_chunk(req, &entries[i], i + 1);
        if (err != ESP_OK) {
            goto records_exit;
        }
    }

    SEND_API_JSON_OR_EXIT("]}");

records_exit:
    free(entries);
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, NULL);
    } else {
        ESP_LOGW(TAG, "HTTP records send aborted: %s", esp_err_to_name(err));
    }
#undef SEND_API_JSON_OR_EXIT
    return err;
}

static esp_err_t api_sync_get_handler(httpd_req_t *req)
{
    char *query = alloc_query_string(req);
    if (httpd_req_get_url_query_len(req) > 0 && !query) {
        return send_json_error(req, "500 Internal Server Error", "Query parsing failed");
    }

    bool has_since = false;
    uint32_t since_unix_time = 0;
    size_t limit = API_DEFAULT_SYNC_LIMIT;

    esp_err_t err = query_get_optional_u32(query, "since", &has_since, &since_unix_time);
    if (err == ESP_OK) {
        err = query_get_optional_limit(query, "limit", API_DEFAULT_SYNC_LIMIT, API_MAX_SYNC_BATCH_LIMIT, &limit);
    }
    if (err != ESP_OK) {
        free(query);
        return send_json_error(req, "400 Bad Request", "Invalid query parameters");
    }
    free(query);

    sensor_log_entry_t *entries = calloc(API_SYNC_BUFFER_CAPACITY, sizeof(sensor_log_entry_t));
    if (!entries) {
        return send_json_error(req, "500 Internal Server Error", "Out of memory");
    }

    bool from_sd = false;
    bool has_more = false;
    uint32_t next_since = since_unix_time;
    size_t count = storage_load_entries_after_time(s_storage,
                                                   entries,
                                                   API_SYNC_BUFFER_CAPACITY,
                                                   limit,
                                                   has_since ? since_unix_time : 0,
                                                   true,
                                                   &from_sd,
                                                   &has_more,
                                                   &next_since);

    httpd_resp_set_type(req, "application/json");

#define SEND_SYNC_JSON_OR_EXIT(chunk_literal_or_buf)                \
    do {                                                            \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf));\
        if (err != ESP_OK) {                                        \
            goto sync_exit;                                         \
        }                                                           \
    } while (0)

    char head[220];
    snprintf(head,
             sizeof(head),
             "{\"count\":%lu,\"limit\":%lu,\"has_more\":%s,\"next_since\":%lu,\"source\":\"%s\",\"entries\":[",
             (unsigned long)count,
             (unsigned long)limit,
             has_more ? "true" : "false",
             (unsigned long)next_since,
             storage_source_name(from_sd));
    SEND_SYNC_JSON_OR_EXIT(head);

    for (size_t i = 0; i < count; i++) {
        if (i != 0) {
            SEND_SYNC_JSON_OR_EXIT(",");
        }
        err = send_json_entry_chunk(req, &entries[i], i + 1);
        if (err != ESP_OK) {
            goto sync_exit;
        }
    }

    SEND_SYNC_JSON_OR_EXIT("]}");

sync_exit:
    free(entries);
    if (err == ESP_OK) {
        err = httpd_resp_sendstr_chunk(req, NULL);
    } else {
        ESP_LOGW(TAG, "HTTP sync send aborted: %s", esp_err_to_name(err));
    }
#undef SEND_SYNC_JSON_OR_EXIT
    return err;
}

static esp_err_t logs_page_get_handler(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;
    sensor_log_entry_t *entries = calloc(WEB_GRAPH_POINT_COUNT, sizeof(sensor_log_entry_t));
    if (!entries) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "Out of memory");
        return ESP_ERR_NO_MEM;
    }
    bool from_sd = false;
    size_t count = storage_load_recent_entries(s_storage, entries, WEB_GRAPH_POINT_COUNT, &from_sd);

#define SEND_CHUNK_OR_EXIT(chunk_literal_or_buf)                     \
    do {                                                             \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf)); \
        if (err != ESP_OK) {                                         \
            goto page_exit;                                          \
        }                                                            \
    } while (0)

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    SEND_CHUNK_OR_EXIT(
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Sensor Logs</title>"
        "<style>body{font-family:Arial,sans-serif;margin:12px}"
        ".charts{display:grid;grid-template-columns:1fr;gap:10px;max-width:900px}"
        "canvas{border:1px solid #999;background:#fff}"
        "table{border-collapse:collapse}th,td{padding:6px}"
        ".v-ok{color:#198754;font-weight:700}"
        ".v-warn{color:#b58900;font-weight:700}"
        ".v-bad{color:#d1242f;font-weight:700}"
        ".v-na{color:#666}</style></head><body>"
        "<h1>Sensor Logs</h1>"
        "<p>Live update: every 5s | JSON: <a href='/json'>/json</a> | CSV: <a href='/csv'>/csv</a></p>");

    SEND_CHUNK_OR_EXIT(
        "<div class='charts'>"
        "<div><b>Temperature (C)</b><br><canvas id='chart_temp' width='860' height='180'></canvas></div>"
        "<div><b>Humidity (%)</b><br><canvas id='chart_rh' width='860' height='180'></canvas></div>"
        "<div><b>AQI</b><br><canvas id='chart_aqi' width='860' height='180'></canvas></div>"
        "<div><b>TVOC (ppb)</b><br><canvas id='chart_tvoc' width='860' height='180'></canvas></div>"
        "<div><b>eCO2 (ppm)</b><br><canvas id='chart_eco2' width='860' height='180'></canvas></div>"
        "<div><b>ENS validity</b><br><canvas id='chart_validity' width='860' height='180'></canvas></div>"
        "</div><br>");

    char meta[200];
    snprintf(meta, sizeof(meta), "<p>Graph points: %lu / %u (%s)</p>",
             (unsigned long)count,
             WEB_GRAPH_POINT_COUNT,
             from_sd ? "SD card" : "NVS");
    SEND_CHUNK_OR_EXIT(meta);

    SEND_CHUNK_OR_EXIT(
        "<table border='1' cellspacing='0' cellpadding='6'>"
        "<tr><th>#</th><th>Time</th><th>T(C)</th><th>RH(%)</th><th>AQI</th><th>TVOC(ppb)</th><th>eCO2(ppm)</th><th>ENS validity</th></tr>");

    size_t table_count = count < WEB_TABLE_ROW_COUNT ? count : WEB_TABLE_ROW_COUNT;
    size_t table_start = count - table_count;

    for (size_t i = table_start; i < count; i++) {
        const sensor_log_entry_t *entry = &entries[i];

        char row[560];
        char time_str[32];
        char temp_str[16] = "-";
        char rh_str[16] = "-";
        char aqi_str[16] = "-";
        char tvoc_str[16] = "-";
        char eco2_str[16] = "-";
        char validity_str[16];
        const char *temp_class = "v-na";
        const char *rh_class = "v-na";
        const char *aqi_class = "v-na";
        const char *tvoc_class = "v-na";
        const char *eco2_class = "v-na";
        const char *validity_class = severity_css_class(ens_validity_severity(entry->ens_validity));

        storage_format_unix_time(entry->unix_time, time_str, sizeof(time_str));

        if (entry->has_aht) {
            snprintf(temp_str, sizeof(temp_str), "%.2f", entry->temperature_c_x100 / 100.0f);
            snprintf(rh_str, sizeof(rh_str), "%.2f", entry->humidity_pct_x100 / 100.0f);
            temp_class = severity_css_class(temperature_severity(entry->temperature_c_x100));
            rh_class = severity_css_class(humidity_severity(entry->humidity_pct_x100));
        }

        if (entry->has_ens) {
            snprintf(aqi_str, sizeof(aqi_str), "%u", entry->aqi);
            snprintf(tvoc_str, sizeof(tvoc_str), "%u", entry->tvoc_ppb);
            snprintf(eco2_str, sizeof(eco2_str), "%u", entry->eco2_ppm);
            aqi_class = severity_css_class(aqi_severity(entry->aqi));
            tvoc_class = severity_css_class(tvoc_severity(entry->tvoc_ppb));
            eco2_class = severity_css_class(eco2_severity(entry->eco2_ppm));
        }

        snprintf(validity_str, sizeof(validity_str), "%u", entry->ens_validity);

        snprintf(row, sizeof(row),
                 "<tr><td>%lu</td><td>%s</td>"
                 "<td class='%s'>%s</td><td class='%s'>%s</td>"
                 "<td class='%s'>%s</td><td class='%s'>%s</td><td class='%s'>%s</td>"
                 "<td class='%s'>%s</td></tr>",
                 (unsigned long)(i - table_start + 1),
                 time_str,
                 temp_class, temp_str,
                 rh_class, rh_str,
                 aqi_class, aqi_str,
                 tvoc_class, tvoc_str,
                 eco2_class, eco2_str,
                 validity_class, validity_str);
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
    free(entries);
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
    esp_err_t err = ESP_OK;
    sensor_log_entry_t *entries = calloc(WEB_GRAPH_POINT_COUNT, sizeof(sensor_log_entry_t));
    if (!entries) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "{\"error\":\"Out of memory\"}");
        return ESP_ERR_NO_MEM;
    }
    bool from_sd = false;
    size_t count = storage_load_recent_entries(s_storage, entries, WEB_GRAPH_POINT_COUNT, &from_sd);

#define SEND_JSON_CHUNK_OR_EXIT(chunk_literal_or_buf)                \
    do {                                                             \
        err = httpd_resp_sendstr_chunk(req, (chunk_literal_or_buf)); \
        if (err != ESP_OK) {                                         \
            goto json_exit;                                          \
        }                                                            \
    } while (0)

    httpd_resp_set_type(req, "application/json");

    char head[140];
    snprintf(head, sizeof(head), "{\"count\":%lu,\"capacity\":%u,\"source\":\"%s\",\"entries\":[",
             (unsigned long)count,
             WEB_GRAPH_POINT_COUNT,
             from_sd ? "sd" : "nvs");
    SEND_JSON_CHUNK_OR_EXIT(head);

    bool first = true;
    for (size_t i = 0; i < count; i++) {
        const sensor_log_entry_t *entry = &entries[i];

        if (!first) {
            SEND_JSON_CHUNK_OR_EXIT(",");
        }
        first = false;

        char item[320];
        snprintf(item, sizeof(item),
                 "{\"index\":%lu,\"unix_time\":%lu,\"has_aht\":%u,\"temperature_c\":%.2f,\"humidity_pct\":%.2f,"
                 "\"has_ens\":%u,\"aqi\":%u,\"tvoc_ppb\":%u,\"eco2_ppm\":%u,\"ens_validity\":%u}",
                 (unsigned long)(i + 1),
                 (unsigned long)entry->unix_time,
                 entry->has_aht,
                 entry->temperature_c_x100 / 100.0f,
                 entry->humidity_pct_x100 / 100.0f,
                 entry->has_ens,
                 entry->aqi,
                 entry->tvoc_ppb,
                 entry->eco2_ppm,
                 entry->ens_validity);
        SEND_JSON_CHUNK_OR_EXIT(item);
    }

    SEND_JSON_CHUNK_OR_EXIT("]}");

    err = ESP_OK;

json_exit:
    free(entries);
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

static esp_err_t logs_csv_get_handler(httpd_req_t *req)
{
    esp_err_t err = ESP_OK;

    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=logs.csv");

    if (storage_is_sd_ready(s_storage)) {
        FILE *f = fopen(storage_log_path(), "r");
        if (!f) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_sendstr(req, "Failed to open CSV file");
            return ESP_FAIL;
        }

        char buf[512];
        while (fgets(buf, sizeof(buf), f)) {
            err = httpd_resp_sendstr_chunk(req, buf);
            if (err != ESP_OK) {
                break;
            }
        }
        fclose(f);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HTTP csv send aborted: %s", esp_err_to_name(err));
            return err;
        }

        return httpd_resp_sendstr_chunk(req, NULL);
    }

    sensor_log_entry_t *entries = calloc(WEB_GRAPH_POINT_COUNT, sizeof(sensor_log_entry_t));
    if (!entries) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_sendstr(req, "Out of memory");
        return ESP_ERR_NO_MEM;
    }

    size_t count = storage_load_recent_entries_from_nvs(s_storage, entries, WEB_GRAPH_POINT_COUNT);

    err = httpd_resp_sendstr_chunk(req, "unix_time,temp_x100,humidity_x100,eco2_ppm,tvoc_ppb,aqi,ens_validity,has_aht,has_ens\n");
    if (err != ESP_OK) {
        free(entries);
        ESP_LOGW(TAG, "HTTP csv header send aborted: %s", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < count; i++) {
        const sensor_log_entry_t *entry = &entries[i];
        char row[200];
        snprintf(row, sizeof(row),
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

        err = httpd_resp_sendstr_chunk(req, row);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HTTP csv send aborted: %s", esp_err_to_name(err));
            free(entries);
            return err;
        }
    }

    free(entries);
    return httpd_resp_sendstr_chunk(req, NULL);
}

esp_err_t web_server_start(const storage_ctx_t *storage)
{
    if (!storage) {
        return ESP_ERR_INVALID_ARG;
    }

    s_storage = storage;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 7;
    config.stack_size = 8192;

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

    httpd_uri_t logs_csv_uri = {
        .uri = "/csv",
        .method = HTTP_GET,
        .handler = logs_csv_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t api_status_uri = {
        .uri = "/api/v1/status",
        .method = HTTP_GET,
        .handler = api_status_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t api_records_uri = {
        .uri = "/api/v1/records",
        .method = HTTP_GET,
        .handler = api_records_get_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t api_sync_uri = {
        .uri = "/api/v1/sync",
        .method = HTTP_GET,
        .handler = api_sync_get_handler,
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

    err = httpd_register_uri_handler(s_http_server, &logs_csv_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &api_status_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &api_records_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    err = httpd_register_uri_handler(s_http_server, &api_sync_uri);
    if (err != ESP_OK) {
        httpd_stop(s_http_server);
        s_http_server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Web server started on %s", network_get_access_url());
    return ESP_OK;
}
