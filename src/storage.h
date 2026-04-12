#ifndef STORAGE_H
#define STORAGE_H

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "app_types.h"

esp_err_t storage_init(storage_ctx_t *ctx);
esp_err_t storage_append_entry(storage_ctx_t *ctx, const sensor_log_entry_t *entry);
size_t storage_load_recent_entries(const storage_ctx_t *ctx,
                                   sensor_log_entry_t *entries,
                                   size_t max_entries,
                                   bool *from_sd);
size_t storage_load_recent_entries_from_nvs(const storage_ctx_t *ctx,
                                            sensor_log_entry_t *entries,
                                            size_t max_entries);
bool storage_is_sd_ready(const storage_ctx_t *ctx);
const char *storage_log_path(void);
void storage_format_unix_time(uint32_t unix_time, char *buf, size_t buf_len);

#endif
