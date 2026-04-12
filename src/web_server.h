#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"

#include "app_types.h"

esp_err_t web_server_start(const storage_ctx_t *storage);

#endif
