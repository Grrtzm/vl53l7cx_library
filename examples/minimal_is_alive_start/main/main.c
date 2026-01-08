#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "vl53l7cx_api.h"   // provided by the ULD you will copy in
#include "platform.h"

static const char *TAG = "vl53l7cx_min";

/* Pins as requested */
#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 21
#define SCL_GPIO 22

#define I2C_SCL_HZ 1000000
#define VL53_ADDR_7BIT 0x29

void app_main(void)
{
    uint8_t status = 0;
    uint8_t isAlive = 0;

    VL53L7CX_Configuration dev = {0};   // comes from ULD header

    /* 1) Create I2C bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    /* 2) Add VL53L7CX as device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53_ADDR_7BIT,
        .scl_speed_hz = I2C_SCL_HZ,
    };

    /* Platform object used by ST ULD */
    dev.platform.address = VL53_ADDR_7BIT;
    dev.platform.reset_gpio = GPIO_NUM_NC; // set if you wire reset/LPn

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev.platform.handle));

    ESP_LOGI(TAG, "Checking is_alive...");
    status = vl53l7cx_is_alive(&dev, &isAlive);
    if (status || !isAlive) {
        ESP_LOGE(TAG, "VL53L7CX not detected (status=%u, isAlive=%u)", status, isAlive);
        return;
    }
    ESP_LOGI(TAG, "VL53L7CX detected");

    ESP_LOGI(TAG, "Init...");
    status = vl53l7cx_init(&dev);
    if (status) {
        ESP_LOGE(TAG, "Init failed (status=%u)", status);
        return;
    }

    ESP_LOGI(TAG, "Start ranging...");
    status = vl53l7cx_start_ranging(&dev);
    if (status) {
        ESP_LOGE(TAG, "Start ranging failed (status=%u)", status);
        return;
    }

    ESP_LOGI(TAG, "Ranging started. Stopping after 500ms...");
    vTaskDelay(pdMS_TO_TICKS(500));
    (void)vl53l7cx_stop_ranging(&dev);
    ESP_LOGI(TAG, "Stopped.");
}
