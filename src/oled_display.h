#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "app_types.h"

esp_err_t oled_display_init(i2c_master_bus_handle_t bus);
void oled_display_publish_entry(const sensor_log_entry_t *entry);
bool oled_display_is_available(void);

#endif
