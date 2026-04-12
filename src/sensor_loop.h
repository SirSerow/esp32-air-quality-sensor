#ifndef SENSOR_LOOP_H
#define SENSOR_LOOP_H

#include "esp_err.h"

#include "app_types.h"

esp_err_t sensor_loop_start(const sensors_t *sensors, storage_ctx_t *storage);

#endif
