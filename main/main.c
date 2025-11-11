// Basic includes
#include "freertos/FreeRTOS.h"     // FreeRTOS core types
#include "driver/gpio.h"           // GPIO driver
#include "esp_log.h"               // log driver

// VL53L4CD includes
#include "driver/i2c_master.h"     // new I²C driver (ESP‑IDF v5+)
#include "platform.h"              // VL53L4CD_SetDeviceHandle, TOF_ESP
#include "vl53l4cd_api.h"          // VL53L4CD API functions
#include "vl53l4cd_calibration.h"  // VL53L4CD calibration functions

// LCD includes
#include "driver/spi_master.h"     // SPI driver
#include "esp_lcd_io_spi.h"        // SPI-specific: esp_lcd_panel_io_spi_config_t, esp_lcd_new_panel_io_spi
#include "esp_lcd_types.h"
#include "hal/lcd_types.h"
#include "esp_lcd_panel_st7789.h"  // ST7789 specific header
#include "esp_lcd_panel_ops.h"     // LCD operations header
#include "esp_lcd_panel_io.h"      // generic panel IO declarations

// I2C pins
#define I2C_PORT       (I2C_NUM_0)
#define I2C_SDA_GPIO   (21)
#define I2C_SCL_GPIO   (22)
#define I2C_FREQ_HZ    (100000)      // start at 100 kHz
#define TOF_ADDR_7BIT  (0x29)        // VL53L4CD default 7-bit address (0x52 >> 1)

// SPI pins
#define LCD_HOST       (SPI2_HOST)
#define PIN_NUM_SCLK   (18)
#define PIN_NUM_MOSI   (23)
#define PIN_NUM_MISO   (-1)          // ST7789: no MISO since write only
#define PIN_NUM_CS     (5)
#define PIN_NUM_DC     (16)
#define PIN_NUM_RST    (17)
#define PIN_NUM_BL     (4)

// LCD resolution
#define LCD_HRES       (240)
#define LCD_VRES       (240)

// TAGs
static const char *TAG_SENSOR = "TOF_SENSOR";
static const char *TAG_LCD = "LCD";

// Shared variables for communication between tasks
static volatile uint16_t current_distance = (0);
static volatile uint8_t distance_status = (255);  // Initialize to error state



static void sensor_task(void *arg) {
    
    // VL53L4CD ranging variables
    Dev_t dev;
    uint8_t status, isReady;
    uint16_t sensor_id;
    VL53L4CD_ResultsData_t results;

    // Create I²C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, // enum constant provided by ESP-IDF
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // Add the VL53L4CD device on that bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // enum constant provided by ESP-IDF
        .device_address  = TOF_ADDR_7BIT,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
  
    // Set I2C handle for sensor driver
    VL53L4CD_SetDeviceHandle(dev_handle);

    // Check if sensor is present on I2C bus
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, TOF_ADDR_7BIT, 50));   

    // Check if there is a VL53L4CD sensor connected
    dev = TOF_ESP;
    status = VL53L4CD_GetSensorId(dev, &sensor_id);
    if(status || (sensor_id != 0xEBAA)) {
        ESP_LOGE(TAG_SENSOR, "VL53L4CD not detected at requested address (ID: 0x%04X, expected: 0xEBAA)", sensor_id);
        vTaskDelete(NULL);
        return;
    }

    // Init VL53L4CD sensor
    status = VL53L4CD_SensorInit(dev);
    if(status) {
        ESP_LOGE(TAG_SENSOR, "VL53L4CD ULD Loading failed (status: %u)", status);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_SENSOR, "VL53L4CD ULD ready!");


    // Offset calibration – to be performed once before production
    // Place a 17% reflective target at 100 mm distance in dark conditions
    /*int16_t offset_mm = 0;
    status = VL53L4CD_CalibrateOffset(dev, 100, &offset_mm, 20);
    if(status == 0) {
        ESP_LOGI(TAG_SENSOR, "Offset calibration complete, value = %d mm", offset_mm);
    } else {
        ESP_LOGE(TAG_SENSOR, "Offset calibration failed (status: %u)", status);
    }*/
    // Apply offset after successful calibration
    int16_t offset_mm = -23; // individual offset from your calibration printout
    VL53L4CD_SetOffset(dev, offset_mm);
    ESP_LOGI(TAG_SENSOR, "Applied stored offset: %d mm", offset_mm);

    // Start ranging measurements
    status = VL53L4CD_StartRanging(dev);
    if(status) {
        ESP_LOGE(TAG_SENSOR, "Failed to start ranging (status: %u)", status);
        vTaskDelete(NULL);
        return;
    }

    // Main ranging loop
    for(;;)
    {
        // Use polling function to know when a new measurement is ready.
        // (Another way can be to wait for HW interrupt raised on PIN 7 (GPIO 1) when a new measurement is ready)
        status = VL53L4CD_CheckForDataReady(dev, &isReady);

        if(isReady)
        {
            // (Mandatory) Clear HW interrupt to restart measurements
            VL53L4CD_ClearInterrupt(dev);

            // Read measured distance. RangeStatus = 0 means valid data
            VL53L4CD_GetResult(dev, &results);
            
            // Update shared variables (thread-safe for simple assignments on ESP32)
            current_distance = results.distance_mm;
            distance_status = results.range_status;
        }

        // Wait a few ms to avoid too high polling
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}



static void lcd_task(void *arg) {
    
    #define PIX_BUF_SIZE (240 * 20)
    static uint16_t linebuf[PIX_BUF_SIZE]; // small scratch buffer
    #define GREEN (0x001F)
    #define RED (0x07E0)
    
    // Create SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_HRES * 80 * sizeof(uint16_t), // ~80 lines per xfer
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Create LCD IO (DC/CS/clock/mode)
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 40 * 1000 * 1000, // 40 MHz
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // Create ST7789 panel
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16, // RGB565
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel));

    // Create Backlight pin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PIN_NUM_BL, 1); // backlight on

    // Various utilities
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x21, NULL, 0)); // Display Inversion On
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, false, false)); // no mirror
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, false)); // no swap
    
    // Clear entire screen to black initially
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    
    for (size_t i = 0; i < PIX_BUF_SIZE; ++i) {
        linebuf[i] = 0x0000; // black
    }
    for (int y = 0; y < LCD_VRES; y += 20) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_HRES, y + 20, linebuf);
    }

    ESP_LOGI(TAG_LCD, "ST7789 init complete, starting display loop");

    // Display update loop
    uint8_t last_status = 255; // Force initial update
    uint16_t last_distance = 0; // Track distance changes
    for (;;) {
        // Read current sensor values
        uint8_t status = distance_status;
        uint16_t distance = current_distance;
        
        // Update display when status or distance changes
        bool needs_update = (status != last_status) || (distance != last_distance);
        
        if (needs_update) {
            last_status = status;
            last_distance = distance;
            
            // Color based on measurement status and distance
            uint16_t color = RED;
            
            if (status == 0) {
                // Valid measurement
                if (distance >= 320 && distance <= 380) {
                    color = GREEN; // Green - distance is within desired range
                    ESP_LOGI(TAG_LCD, "Speaker within range: %.1f cm", distance / 10.0f);
                } else {
                    color = RED; // Red - distance is outside desired range
                    ESP_LOGI(TAG_LCD, "Speaker out of range: %.1f cm", distance / 10.0f);
                }
            } else if (status == 1 || status == 2 || status == 6) {
                // Same for warning statuses only
                // 1: Sigma is above the defined threshold (= standard deviation of the laser pulse measurement = noiser measurements)
                // 2: Signal is below the defined threshold
                // 6: Phase valid but no wrap around check performed
                if (distance >= 320 && distance <= 380) {
                    color = GREEN; // Green - distance is within desired range (but with warning)
                    ESP_LOGI(TAG_LCD, "Speaker within range: %.1f cm (warning: status %u)", distance / 10.0f, status);
                } else {
                    color = RED; // Red - distance is outside desired range (with warning)
                    ESP_LOGI(TAG_LCD, "Speaker out of range: %.1f cm (warning: status %u)", distance / 10.0f, status);
                }
            } else {
                // Error statuses (3-12, 255)
                color = RED; // Red - measurement error
                ESP_LOGI(TAG_LCD, "Measurement error (status: %u)", status);
            }
            
            // Draw colored rectangle
            for (int i = 0; i < PIX_BUF_SIZE; ++i) {
                linebuf[i] = color;
            }

            for (int y = 0; y < LCD_VRES; y += 20) {
                int y1 = y + 20;
                if (y1 > LCD_VRES) {
                    y1 = LCD_VRES;
                }
                ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_HRES, y1, linebuf));
            }
        }

        // Update at ~20 Hz
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}



void app_main(void) {
    ESP_LOGI("MAIN", "Starting sensor and LCD tasks...");
    
    // Create LCD task first
    xTaskCreate(lcd_task,    "lcd_task",    4096, NULL, 5, NULL);
    
    // Create sensor task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL); 
}