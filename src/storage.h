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
size_t storage_load_filtered_entries(const storage_ctx_t *ctx,
                                     sensor_log_entry_t *entries,
                                     size_t max_entries,
                                     bool newest_only,
                                     bool require_valid_time,
                                     bool has_from,
                                     uint32_t from_unix_time,
                                     bool has_to,
                                     uint32_t to_unix_time,
                                     bool *from_sd,
                                     bool *has_more);
size_t storage_load_entries_after_time(const storage_ctx_t *ctx,
                                       sensor_log_entry_t *entries,
                                       size_t entry_capacity,
                                       size_t batch_limit,
                                       uint32_t since_unix_time,
                                       bool require_valid_time,
                                       bool *from_sd,
                                       bool *has_more,
                                       uint32_t *next_since);
bool storage_load_latest_entry(const storage_ctx_t *ctx,
                               sensor_log_entry_t *entry,
                               bool require_valid_time,
                               bool *from_sd);
size_t storage_load_recent_entries_from_nvs(const storage_ctx_t *ctx,
                                            sensor_log_entry_t *entries,
                                            size_t max_entries);
bool storage_is_sd_ready(const storage_ctx_t *ctx);
const char *storage_log_path(void);
void storage_format_unix_time(uint32_t unix_time, char *buf, size_t buf_len);

#endif
