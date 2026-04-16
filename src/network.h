#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    NETWORK_MODE_SOFTAP = 0,
    NETWORK_MODE_STA,
} network_mode_t;

typedef struct {
    network_mode_t mode;
    char ip[16];
    char gateway[16];
    char netmask[16];
    char ssid[33];
    int8_t rssi;
    uint8_t channel;
    uint8_t connected_clients;
    bool has_rssi;
} network_status_t;

void network_configure_timezone_jst(void);
esp_err_t network_start(void);
esp_err_t network_start_softap(void);
bool network_is_sta_active(void);
const char *network_get_access_url(void);
esp_err_t network_get_status(network_status_t *status);

#endif
