# ESP32 Air Quiality Sensor

This is a simple ESP32-based air quality sensor that uses the AHT20 temperature and humidity sensor and the ENS160 air quality sensor. 

## Hardware
- ESP32-C6-DevKitC-1
- Sensor board with AHT20 and ENS160 sensors:
  - AHT20 Temperature and Humidity Sensor
  - ENS160 Air Quality Sensor

## Software
The project is built using the ESP-IDF framework and PlatformIO. The code is organized into the following directories:
- `src`: Contains the main application code.
- `include`: Contains header files for the project.
- `lib`: Contains any external libraries used in the project.
- `test`: Contains unit tests for the project.
- platformio.ini: Configuration file for PlatformIO.
- cmake_lists.txt: CMake configuration file for ESP-IDF.
- sdkconfig.esp32-c6-devkitc-1: ESP-IDF configuration file for the ESP32-C6-DevKitC-1 board.


## Getting Started
1. Clone the repository to your local machine.
2. Open the project in your preferred IDE (e.g., Visual Studio Code with PlatformIO extension).
3. Connect the ESP32-C6-DevKitC-1 to your computer via USB.
4. Build and flash the firmware to the ESP32 using PlatformIO.
5. Monitor the serial output to see the sensor readings. Example output:
    ``` 
        I (11990224) MAIN: ENS: AQI=3  TVOC=331 ppb  eCO2=812 ppm
        I (11991314) MAIN: AHT: T=25.17 C  RH=47.53 %
        I (11992324) MAIN: ENS: AQI=3  TVOC=344 ppb  eCO2=816 ppm
        I (11993414) MAIN: AHT: T=25.18 C  RH=47.57 %
        I (11994424) MAIN: ENS: AQI=3  TVOC=345 ppb  eCO2=816 ppm
        I (11995514) MAIN: AHT: T=25.18 C  RH=47.58 %
        I (11996524) MAIN: ENS: AQI=3  TVOC=352 ppb  eCO2=819 ppm
    ```
