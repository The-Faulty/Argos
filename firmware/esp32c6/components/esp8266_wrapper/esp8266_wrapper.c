/**
 * Wrapper module for source code compatibility with esp-open-rtos.
 */

#ifdef ESP_PLATFORM // ESP32 (ESP-IDF)

#include <sys/time.h>
#include <string.h>
#include "esp_log.h"

#include "hal/spi_types.h"
#include "driver/spi_common.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp8266_wrapper.h"

static const char *TAG = "i2c_wrapper";

// --- I2C ---

void i2c_init(int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq,
        // .clk_flags = 0,  // uncomment if you get a clk_flags compile error
    };

    esp_err_t err = i2c_param_config(bus, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
    }
}

int i2c_slave_write(uint8_t bus, uint8_t addr, const uint8_t *reg,
                    uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    if (reg)
        i2c_master_write_byte(cmd, *reg, true);
    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    // portTICK_PERIOD_MS replaces the removed portTICK_RATE_MS
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

int i2c_slave_read(uint8_t bus, uint8_t addr, const uint8_t *reg,
                   uint8_t *data, uint32_t len)
{
    if (len == 0)
        return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (reg)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, *reg, true);
        if (!data)
            i2c_master_stop(cmd);
    }

    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        if (len > 1)
            // I2C_MASTER_ACK replaces the removed I2C_ACK_VAL
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        // I2C_MASTER_NACK replaces the removed I2C_NACK_VAL
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
    }

    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// --- SPI --- (keep whatever was originally here)
bool spi_bus_init(spi_host_device_t host, uint8_t sclk, uint8_t miso, uint8_t mosi)
{
    spi_bus_config_t cfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    return spi_bus_initialize(host, &cfg, 1) == ESP_OK;
}

bool spi_device_init(uint8_t bus, uint8_t cs)
{
    // minimal stub — only needed if SPI is used
    return true;
}

size_t spi_transfer_pf(uint8_t bus, uint8_t cs,
                       const uint8_t *mosi, uint8_t *miso, uint16_t len)
{
    // minimal stub — only needed if SPI is used
    return 0;
}

uint32_t sdk_system_get_time(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000000 + tv.tv_usec);
}

#endif // ESP_PLATFORM