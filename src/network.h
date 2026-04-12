#ifndef NETWORK_H
#define NETWORK_H

#include "esp_err.h"

void network_configure_timezone_jst(void);
esp_err_t network_sync_time_via_sta(void);
esp_err_t network_start_softap(void);

#endif
