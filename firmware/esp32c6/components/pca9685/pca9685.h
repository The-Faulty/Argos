// PCA9685 16-channel I2C PWM driver — minimal subset for hobby servos.
//
// Only what Argos needs: init, set frequency, write a per-channel pulse width
// in microseconds, and a "release" call that drops a channel to 0 duty so the
// attached servo goes limp (matches the watchdog behavior in
// web/leg_viz/server.py on the Pi side).

#ifndef ARGOS_PCA9685_H
#define ARGOS_PCA9685_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// PWM_FREQ matches the Pi-side bench tool. 50 Hz is the standard hobby-servo
// refresh; both endpoints have to agree or the µs→duty math drifts.
#define PCA9685_DEFAULT_FREQ_HZ 50

// All 16 channels. We use 12 for the legs; the rest are spare.
#define PCA9685_NUM_CHANNELS 16

esp_err_t pca9685_init(i2c_port_t port, uint8_t addr, uint16_t freq_hz);

// Set one channel's pulse width in microseconds. 0 releases the channel
// (full off, servo goes limp). Range-clamped internally.
esp_err_t pca9685_set_pulse_us(uint16_t pulse_us, uint8_t channel);

// Release every channel — used by the joint-command watchdog when the Pi
// stops talking, so the robot sags rather than freezing on the last command.
esp_err_t pca9685_release_all(void);

#ifdef __cplusplus
}
#endif

#endif  // ARGOS_PCA9685_H
