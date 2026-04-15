#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>

#include "esp_err.h"

void network_configure_timezone_jst(void);
esp_err_t network_start(void);
esp_err_t network_start_softap(void);
bool network_is_sta_active(void);
const char *network_get_access_url(void);

#endif
