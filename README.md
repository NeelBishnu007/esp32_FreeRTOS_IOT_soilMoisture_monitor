# ESP32 Soil Moisture Monitor

This is a practical, end-to-end IoT soil moisture monitoring system built on ESP-IDF(ESP IoT Development Framework). It integrates real-time data acquisition, data processing, local visualization and cloud integration. 

It utilizes the ESP32 SOC (ESP32-D0WD-V3), which features a 32-bit Xtensa LX6 dual-core microprocessor. It has Bluetooth v4.2 and Wi-Fi 802.11 b/g/n connectivity and 40MHz crystal. Espressif's ESP-IDF has extensive documentation and examples that facilitate the development of applications that utilize the different modules of the SOC. ESP-IDF allows the exploitation of the FreeRTOS real-time operating system, which enables multitasking features of the ESP32. 


## 🛠️ Technical Highlights
- **FreeRTOS Architecture:** Implements a producer-consumer model using 3 tasks and 2 thread-safe Queues.
- **ADC Calibration:** Utilizes factory eFuse data and Line Fitting schemes for accurate voltage-to-moisture mapping.
- **I2C Integration:** Drives a SH1106 OLED display using the `u8g2` library.
- **IoT Connectivity:** WiFi stack for data integration with the ThingSpeak cloud account.

## 🏗️ Firmware Architecture
The system is divided into three primary tasks:
1. **Sensor Task:** High-priority task sampling ADC Channel 4 (GPIO 32) every 1s.
2. **ThingSpeak Task:** Performs 10-sample averaging and uploads to the cloud via HTTP GET.
3. **OLED Task:** Low-priority UI task for real-time local monitoring on the diplay. 
