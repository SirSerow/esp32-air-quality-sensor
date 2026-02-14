# ESP32 Air Quality Sensor

ESP32-C6 firmware for reading AHT20 (temperature/humidity) and ENS160 (air quality) data, storing historical samples in NVS, and serving a built-in web dashboard over Wi-Fi hotspot mode.

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
- Stores sensor history in NVS using a circular buffer.
- Starts Wi-Fi SoftAP automatically on boot.
- Starts built-in HTTP server automatically on boot.
- Web dashboard at `/` with:
  - Live-updating table of stored values
  - Separate graph for each metric
- JSON API at `/json` for easier parsing/integration.

## Default Runtime Configuration
- SoftAP SSID: `CO2-Sensor-AP`
- SoftAP password: `co2sensor123`
- AP URL: `http://192.168.4.1/`
- JSON endpoint: `http://192.168.4.1/json`
- Sampling interval: 3 minutes (`SENSOR_READ_INTERVAL_MS = 180000`)
- NVS ring-buffer capacity: 180 points (`NVS_SLOT_COUNT = 180`)

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
platformio run -e esp32-c6-devkitc-1
platformio run -e esp32-c6-devkitc-1 -t upload
platformio device monitor -b 115200
```

## Accessing the Dashboard
1. Power/flash the board and wait for boot logs.
2. Connect your phone/laptop to Wi-Fi `CO2-Sensor-AP`.
3. Open:
   - Dashboard: `http://192.168.4.1/`
   - JSON: `http://192.168.4.1/json`

## Example Serial Logs
```text
I (...) MAIN: NVS logging enabled (sensorlog)
I (...) MAIN: NVS stats: used=... free=... total=... namespaces=...
I (...) MAIN: WiFi AP started: SSID=CO2-Sensor-AP CH=1
I (...) MAIN: Web server started on http://192.168.4.1/
I (...) MAIN: AHT: T=25.17 C  RH=47.53 %
I (...) MAIN: ENS: AQI=3  TVOC=331 ppb  eCO2=812 ppm
```
