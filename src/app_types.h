#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "nvs.h"
#include "sdmmc_cmd.h"

#include "ahtxx.h"
#include "ens160.h"

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
    nvs_handle_t nvs;
    bool nvs_ready;
    bool sd_ready;
    sdmmc_card_t *sd_card;
} storage_ctx_t;

#endif
