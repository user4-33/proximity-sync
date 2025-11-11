# Proximity-Sync
An ESP32-sensor-display setup for maintaining equal and consistent distances between a speaker and a microphone.



## Hardware
The setup consists of the following components:
- [**Pololu VL53L4CD**](https://www.pololu.com/product/3692) Time-of-Flight sensor for distance measurement via I²C
- [**Waveshare 1.3" LCD (ST7789)**](https://www.waveshare.com/wiki/1.3inch_LCD_Module) for visual feedback via SPI
- **ESP32 NodeMCU** running on the **ESP-IDF SDK**

The LCD acts as a binary semaphore: it shows whether the speaker is at the correct distance from the microphone array.

### Pin Connections

#### Sensor ↔︎ ESP32
| Sensor Pin | ESP32 Pin | Notes |
|-------------|------------|-------|
| GND | GND | Ground |
| VIN | 3V3 | Power supply (3.3 V) |
| SDA | GPIO21 | I²C data line |
| SCL | GPIO22 | I²C clock line |
| XSHUT | GPIO33 | Shutdown / enable control |

#### Display ↔︎ ESP32
| Display Pin | ESP32 Pin | Notes |
|--------------|------------|-------|
| GND | GND | Ground |
| VCC | 3V3 | Power supply (3.3 V) |
| DIN | GPIO23 | SPI Data input: MOSI/SDO/COPI |
| CLK | GPIO18 | SPI Clock input: SCK/CLK/SCLK |
| CS  | GPIO5 | Chip select, Low active: SS/CS |
| DC  | GPIO16 | Data/Command select/Mode select |
| RST | GPIO17 | Hardware reset |
| BL | GPIO4 | LED backlight control |



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