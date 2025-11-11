# Proximity-Sync
An ESP32-sensor-display setup for maintaining equal and consistent distances between a speaker and a microphone.



## Hardware
The setup consists of the following components:
- [**Pololu VL53L4CD**](https://www.pololu.com/product/3692) Time-of-Flight sensor for distance measurement via I²C
- [**Waveshare 1.3" LCD (ST7789)**](https://www.waveshare.com/wiki/1.3inch_LCD_Module) for visual feedback via SPI
- **ESP32 NodeMCU** running on the **ESP-IDF SDK**

The LCD acts as a binary semaphore: it shows whether the speaker is at the correct distance from the microphone array.



## Software
- **Framework:** [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.2.5/esp32s3/index.html) (v5.x recommended)
- **Language:** C

### Dependencies
This project uses both built-in ESP-IDF components and one vendored driver:

| Component | Source | Purpose |
|------------|---------|----------|
| `driver` | Included in ESP-IDF | Base GPIO, I2C, and SPI drivers |
| `esp_lcd` | Included in ESP-IDF | LCD abstraction layer for ST7789 panels |
| `esp_driver_spi` | Included in ESP-IDF | SPI driver used by LCD and other peripherals |
| [`st_vl53l4cd`](https://github.com/LooUQ/st_vl53l4cd) | Vendored in `/components` | Time-of-Flight sensor driver |



## Build Instructions
1. Clone the repo:
   ```bash
   git clone https://github.com/user4-33/proximity-sync.git
   cd proximity-sync
   ```

2. Set up ESP-IDF (if not yet done):
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

3. Configure the target:
   ```bash
   idf.py set-target esp32
   ```

4. Build and flash:
   ```bash
   idf.py build flash monitor
   ```