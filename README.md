# ESP32 Air Quality Sensor

ESP32-C6 firmware for reading AHT20 (temperature/humidity) and ENS160 (air quality) data, storing historical samples (SD card preferred, NVS fallback), showing live values on an SSD1306 OLED, and serving a built-in web dashboard over external Wi-Fi when available, with SoftAP fallback.

## Hardware
- ESP32-C6-DevKitC-1
- Sensor board with:
  - AHT20 temperature/humidity sensor
  - ENS160 air quality sensor

## Current Features
- Reads and logs:
  - Temperature (°C)
  - Humidity (%)
  - AQI
  - TVOC (ppb)
  - eCO2 (ppm)
  - ENS validity flag
- Stores sensor history to SD card (`/sdcard/logs.csv`) when available.
- Falls back to NVS circular buffer storage if SD card is unavailable.
- OLED interface (SSD1306, 128x64, I2C 0x3C):
  - 4-line live status view (time, T/RH, AQI/TVOC, eCO2/validity)
  - Health code markers per value (`G`/`Y`/`R`)
  - Button toggle on GPIO5 (press to turn OLED on/off)
- Tries external Wi-Fi first, then falls back to SoftAP only if STA is unavailable.
- Starts built-in HTTP server automatically on boot.
- Web dashboard at `/` with:
  - Live-updating table of stored values
  - Separate graph for each metric
- JSON API at `/json` for the built-in dashboard recent window.
- CSV download endpoint at `/csv`.
- Versioned server sync API under `/api/v1` for status, time-bounded history, and incremental polling.
- Syncs time over the STA connection when external Wi-Fi is available.
- Starts mDNS on STA so the dashboard can be reached via a stable `.local` name.

## Default Runtime Configuration
- STA dashboard URL: `http://co2-sensor.local/` when `WIFI_STA_SSID` / `WIFI_STA_PASS` are configured and reachable
- mDNS hostname: `co2-sensor`
- mDNS instance name: `CO2 Sensor`
- SoftAP SSID: `CO2-Sensor-AP`
- SoftAP password: `co2sensor123`
- SoftAP fallback URL: `http://192.168.4.1/`
- JSON endpoint: `/json`
- CSV endpoint: `/csv`
- Server status endpoint: `/api/v1/status`
- Server records endpoint: `/api/v1/records`
- Server sync endpoint: `/api/v1/sync`
- Sampling interval: 1 minute (`SENSOR_READ_INTERVAL_MS = 60000`)
- NVS ring-buffer capacity: 180 points (`NVS_SLOT_COUNT = 180`)
- SD log file path: `/sdcard/logs.csv`
- OLED I2C address: `0x3C` (SSD1306)
- OLED toggle button GPIO: `5`

## Project Structure
- `src/` main application code
- `include/` project headers
- `lib/` external/project libraries
- `test/` test files
- `platformio.ini` PlatformIO configuration
- `CMakeLists.txt` CMake/ESP-IDF project file
- `sdkconfig.esp32-c6-devkitc-1` board-specific ESP-IDF config

## Build, Flash, Monitor
From the project root:

```bash
/home/netlister/.platformio/penv/bin/platformio run -e esp32-c6-devkitc-1
/home/netlister/.platformio/penv/bin/platformio run -e esp32-c6-devkitc-1 -t upload
/home/netlister/.platformio/penv/bin/platformio device monitor -b 115200
```

Use the `~/.platformio/penv/bin/platformio` binary on this machine. `/usr/bin/platformio` resolves to an older incompatible core here.

## Wi-Fi Setup
1. Copy `src/secrets.h.example` to `src/secrets.h` if you have not created it yet.
2. Set `WIFI_STA_SSID` and `WIFI_STA_PASS` to the external Wi-Fi network you want the ESP32 to join.
3. Keep `WIFI_AP_SSID` and `WIFI_AP_PASS` set for fallback access when the external network is unavailable.
4. Rebuild and flash after changing credentials.

## Accessing the Dashboard
1. Power/flash the board and wait for boot logs.
2. If STA connects successfully, open:
   - Dashboard: `http://co2-sensor.local/`
   - JSON: `http://co2-sensor.local/json`
   - CSV: `http://co2-sensor.local/csv`
   - Server API status: `http://co2-sensor.local/api/v1/status`
3. If `.local` resolution is unreliable on your client, use the STA IP address printed in the boot log instead.
4. If STA is unavailable, connect your phone/laptop to Wi-Fi `CO2-Sensor-AP` and use:
   - Dashboard: `http://192.168.4.1/`
   - JSON: `http://192.168.4.1/json`
   - CSV: `http://192.168.4.1/csv`
   - Server API status: `http://192.168.4.1/api/v1/status`

## Network Startup Behavior
1. Boot initializes storage and sensors.
2. Firmware attempts STA connection using `WIFI_STA_SSID` and `WIFI_STA_PASS`.
3. On STA success, the device syncs time with `pool.ntp.org`, starts mDNS, and serves HTTP on the STA interface.
4. On STA failure or when credentials are blank, the device starts the fallback SoftAP and serves HTTP at `192.168.4.1`.

Dashboard preview:
![Final result web GUI](images/user_web_gui.jpg)

## OLED Display Layout
- Line 1: local time (`YY-MM-DD HH:MM`) when time is synced
- Line 2: temperature and humidity (`T` / `H`)
- Line 3: AQI and TVOC (`A` / `T`)
- Line 4: eCO2 and ENS validity (`C` / `V`)
- Marker legend: `G` = good, `Y` = warning, `R` = bad
- If data is unavailable for a sensor, value fields show `-`

## Data Source Behavior
- Startup tries SD card first (SPI pins: MISO=21, MOSI=22, CLK=19, CS=20).
- If SD mount succeeds, logs are appended to `/sdcard/logs.csv`.
- If SD mount fails, logging continues in NVS (`sensorlog` namespace).
- Web graph/table endpoints read from SD when available, otherwise from NVS.

## Server API
The server-facing API is separate from the dashboard endpoints and is intended for periodic polling by another service.

General behavior:
- All `/api/v1/*` endpoints return JSON.
- Historical results are ordered oldest to newest.
- Records with `unix_time = 0` are excluded from `/api/v1/*`.
- Storage source is reported as `"sd"` or `"nvs"`.

### `GET /api/v1/status`
Returns current device metadata and the newest valid-timestamp record.

Example fields:
- `device_time_unix`
- `sample_interval_sec`
- `storage_source`
- `sd_ready`
- `nvs_ready`
- `latest_record`

### `GET /api/v1/records`
Returns a bounded history window.

Query parameters:
- `from`: optional inclusive unix timestamp
- `to`: optional inclusive unix timestamp
- `limit`: optional number of rows, default `120`, max `240`

Behavior:
- With no `from` and no `to`, returns the newest `limit` records.
- With `from` and/or `to`, returns the matching chronological slice.
- Response includes `count`, `limit`, `has_more`, `source`, `from`, `to`, and `entries`.

Examples:
- Newest 60 rows: `/api/v1/records?limit=60`
- Last day starting at a timestamp: `/api/v1/records?from=1712800000`
- Bounded range: `/api/v1/records?from=1712800000&to=1712886400&limit=120`

### `GET /api/v1/sync`
Returns records newer than a server-held cursor for efficient incremental sync.

Query parameters:
- `since`: optional exclusive unix timestamp cursor, default `0`
- `limit`: optional batch size, default `120`, max `240`

Response fields:
- `count`
- `limit`
- `has_more`
- `next_since`
- `source`
- `entries`

Polling model:
1. Call `/api/v1/sync?since=0` for the initial import.
2. Store the returned `next_since` value after successfully ingesting the batch.
3. If `has_more` is `true`, call `/api/v1/sync?since=<next_since>` again immediately.
4. Otherwise wait until the next polling interval.

## KiCad Schematic & PCB Design
The hardware was designed in KiCad and includes:
- ESP32-C6 module connections (power, boot, and programming signals)
- I2C routing for AHT20 and ENS160 sensors
- GPIO breakout for OLED button and SPI SD card interface
- Compact 2-layer PCB layout optimized for sensor board assembly

### Schematic
![Sensor schematic](images/schematic.png)

### PCB Layout
![PCB layout](images/pcb_layout.png)

### 3D PCB Views
Front view:
![3D front view](images/3d_view_front.png)

Back view:
![3D back view](images/3d_view_back.png)

And yes, one design choice may look unnecessary - purely for "airflow optimization," obviously.

## Final Result Photos
Assembled/produced board (front):
![Production PCB front](images/production_pcb_front.jpg)

Assembled/produced board (back):
![Production PCB back](images/production_pcb_back.jpg)

## Example Serial Logs
```text
I (...) MAIN: NVS logging enabled (sensorlog)
I (...) MAIN: NVS stats: used=... free=... total=... namespaces=...
I (...) NET: Connecting to external WiFi SSID=...
I (...) NET: STA connected: IP=192.168.1.50 GW=192.168.1.1
I (...) MDNS: mDNS started: http://co2-sensor.local/
I (...) WEB: Web server started on http://co2-sensor.local/
I (...) MAIN: AHT: T=25.17 C  RH=47.53 %
I (...) MAIN: ENS: AQI=3  TVOC=331 ppb  eCO2=812 ppm
```
