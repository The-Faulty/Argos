#include "pca9685.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pca9685";

#define PCA9685_REG_MODE1      0x00
#define PCA9685_REG_PRESCALE   0xFE
#define PCA9685_REG_LED0_ON_L  0x06

#define PCA9685_MODE1_SLEEP    (1 << 4)
#define PCA9685_MODE1_AI       (1 << 5)
#define PCA9685_MODE1_RESTART  (1 << 7)

#define PCA9685_INTERNAL_CLK_HZ 25000000UL

#define PCA9685_I2C_TIMEOUT_MS 50

static i2c_port_t s_port = I2C_NUM_0;
static uint8_t    s_addr = 0x40;
static uint16_t   s_freq_hz = PCA9685_DEFAULT_FREQ_HZ;
static bool       s_inited = false;

static esp_err_t pca9685_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(s_port, s_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(PCA9685_I2C_TIMEOUT_MS));
}

static esp_err_t pca9685_read_reg(uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(s_port, s_addr, &reg, 1, value, 1,
                                        pdMS_TO_TICKS(PCA9685_I2C_TIMEOUT_MS));
}

esp_err_t pca9685_init(i2c_port_t port, uint8_t addr, uint16_t freq_hz) {
    s_port = port;
    s_addr = addr;
    s_freq_hz = freq_hz ? freq_hz : PCA9685_DEFAULT_FREQ_HZ;

    // Sleep before changing the prescaler — the chip ignores PRE_SCALE writes
    // unless SLEEP=1 (datasheet 7.3.5).
    esp_err_t err = pca9685_write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sleep write failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t prescale = (uint8_t)((PCA9685_INTERNAL_CLK_HZ / (4096UL * s_freq_hz)) - 1);
    err = pca9685_write_reg(PCA9685_REG_PRESCALE, prescale);
    if (err != ESP_OK) return err;

    // Wake + enable register auto-increment so we can write all 4 LEDx_ON/OFF
    // bytes in one transaction. Then pulse RESTART to apply the new prescale.
    err = pca9685_write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_AI);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(1));  // oscillator wake settle (datasheet ≥500 µs)

    uint8_t mode1 = 0;
    if (pca9685_read_reg(PCA9685_REG_MODE1, &mode1) == ESP_OK) {
        pca9685_write_reg(PCA9685_REG_MODE1, mode1 | PCA9685_MODE1_RESTART);
    }

    pca9685_release_all();
    s_inited = true;
    ESP_LOGI(TAG, "init OK addr=0x%02X freq=%u Hz prescale=%u",
             s_addr, (unsigned)s_freq_hz, (unsigned)prescale);
    return ESP_OK;
}

esp_err_t pca9685_set_pulse_us(uint16_t pulse_us, uint8_t channel) {
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    if (channel >= PCA9685_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;

    uint8_t reg = PCA9685_REG_LED0_ON_L + 4 * channel;

    // Special encoding: setting bit 12 of OFF_H (= 0x10 in OFF_H byte) means
    // "full off". We use this for the limp/release case.
    if (pulse_us == 0) {
        uint8_t buf[5] = {reg, 0, 0, 0, 0x10};
        return i2c_master_write_to_device(s_port, s_addr, buf, sizeof(buf),
                                          pdMS_TO_TICKS(PCA9685_I2C_TIMEOUT_MS));
    }

    uint32_t period_us = 1000000UL / s_freq_hz;
    if (pulse_us > period_us) pulse_us = (uint16_t)period_us;

    uint16_t ticks = (uint16_t)((uint32_t)pulse_us * 4096UL / period_us);
    if (ticks > 4095) ticks = 4095;

    // ON edge at 0, OFF edge at `ticks` — front-aligned pulse.
    uint8_t buf[5] = {
        reg,
        0, 0,
        (uint8_t)(ticks & 0xFF),
        (uint8_t)((ticks >> 8) & 0x0F),
    };
    return i2c_master_write_to_device(s_port, s_addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(PCA9685_I2C_TIMEOUT_MS));
}

esp_err_t pca9685_release_all(void) {
    if (!s_inited) {
        // Allow release-all during init bring-up before s_inited flips true.
    }
    esp_err_t last = ESP_OK;
    for (uint8_t ch = 0; ch < PCA9685_NUM_CHANNELS; ++ch) {
        esp_err_t err = pca9685_set_pulse_us(0, ch);
        if (err != ESP_OK) last = err;
    }
    return last;
}
