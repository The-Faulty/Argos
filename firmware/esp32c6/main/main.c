// Argos ESP32-C6 firmware. Owns the fast control loop:
//   * subscribes to /joint_command (sensor_msgs/JointState, 100 Hz from Pi)
//     and drives 12 PCA9685 channels with a watchdog that limps the servos
//     if commands stop coming in
//   * publishes /joint_states (sensor_msgs/JointState, 100 Hz) — echoes the
//     last command so RViz / downstream consumers see the MCU "feedback"
//   * publishes /imu/data_raw + /imu/mag (LSM9DS0, 100 Hz)
//   * publishes /gas (std_msgs/Float32, 10 Hz) — MQ-series ADC reading
//
// The Pi runs micro_ros_agent over UART (see ros2_ws/.../esp32_bridge.launch.py).

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float32.h>
#include <uros_network_interfaces.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "lsm9ds0.h"
#include "pca9685.h"

// ─── Pin / bus assignments ────────────────────────────────────────────────
//
// SCL/SDA values come from the original imu-branch firmware (GPIO22/23). The
// ESP32-C6 only exposes a subset of GPIOs as I2C-capable; verify against the
// schematic before flashing real hardware.
#define I2C_BUS         0
#define I2C_SCL_PIN     22
#define I2C_SDA_PIN     23
#define I2C_FREQ        I2C_FREQ_100K

#define PCA9685_I2C_PORT    I2C_NUM_0

// ─── Compile-time tunables (mirror Kconfig.projbuild) ─────────────────────

#ifndef CONFIG_ARGOS_PCA9685_ADDR
#define CONFIG_ARGOS_PCA9685_ADDR 0x00
#endif
#ifndef CONFIG_ARGOS_GAS_ADC_GPIO
#define CONFIG_ARGOS_GAS_ADC_GPIO 4
#endif
#ifndef CONFIG_ARGOS_JOINT_CMD_TIMEOUT_MS
#define CONFIG_ARGOS_JOINT_CMD_TIMEOUT_MS 250
#endif
#ifndef CONFIG_ARGOS_IMU_RATE_HZ
#define CONFIG_ARGOS_IMU_RATE_HZ 100
#endif
#ifndef CONFIG_ARGOS_GAS_RATE_HZ
#define CONFIG_ARGOS_GAS_RATE_HZ 10
#endif

// ─── Servo / joint constants (mirror ros2_ws/argos_control/Config.py) ─────

#define NUM_JOINTS 12

// Match web/leg_viz/server.py: 0..180° servo travel mapped to 370..2400 µs
// pulse, refreshed at 50 Hz. Both endpoints have to agree on these or the
// math drifts and bench calibration stops matching.
#define SERVO_PWM_FREQ_HZ   50
#define SERVO_PWM_MIN_US    370
#define SERVO_PWM_MAX_US    2400
#define SERVO_RANGE_DEG     180.0f
#define SERVO_CENTER_DEG    90.0f

#define DEG_PER_RAD (57.295779513f)
#define GRAVITY_MS2 9.80665f
#define DEG_TO_RAD  0.017453293f

// Per-joint calibration. Joint order MUST match JOINT_NAMES below
// (FR/FL/RR/RL × coxa/femur/tibia). Tune `direction` and `offset_deg` per
// joint after bench-testing the assembled robot — the Pi sends radians in
// "control space" (relative to neutral); this table is the only place that
// translates that into raw servo travel.
typedef struct {
    uint8_t channel;        // PCA9685 channel index 0..15
    int8_t  direction;      // +1 or -1, flips servo polarity per leg side
    float   offset_deg;     // mechanical zero offset added before clamping
    float   min_deg;        // SERVO_LIMITS_DEG[joint_type, 0]
    float   max_deg;        // SERVO_LIMITS_DEG[joint_type, 1]
} servo_cal_t;

static servo_cal_t s_servo_cal[NUM_JOINTS] = {
    // FR_coxa, FR_femur, FR_tibia
    { .channel = 0,  .direction = +1, .offset_deg = 0, .min_deg = 45, .max_deg = 135 },
    { .channel = 1,  .direction = +1, .offset_deg = 0, .min_deg = 50, .max_deg = 115 },
    { .channel = 2,  .direction = +1, .offset_deg = 0, .min_deg =  5, .max_deg = 180 },
    // FL_coxa, FL_femur, FL_tibia
    { .channel = 3,  .direction = +1, .offset_deg = 0, .min_deg = 45, .max_deg = 135 },
    { .channel = 4,  .direction = +1, .offset_deg = 0, .min_deg = 50, .max_deg = 115 },
    { .channel = 5,  .direction = +1, .offset_deg = 0, .min_deg =  5, .max_deg = 180 },
    // RR_coxa, RR_femur, RR_tibia — back-leg hip mirrored (commit f907ecc)
    { .channel = 6,  .direction = -1, .offset_deg = 0, .min_deg = 45, .max_deg = 135 },
    { .channel = 7,  .direction = +1, .offset_deg = 0, .min_deg = 50, .max_deg = 115 },
    { .channel = 8,  .direction = +1, .offset_deg = 0, .min_deg =  5, .max_deg = 180 },
    // RL_coxa, RL_femur, RL_tibia — back-leg hip mirrored
    { .channel = 9,  .direction = -1, .offset_deg = 0, .min_deg = 45, .max_deg = 135 },
    { .channel = 10, .direction = +1, .offset_deg = 0, .min_deg = 50, .max_deg = 115 },
    { .channel = 11, .direction = +1, .offset_deg = 0, .min_deg =  5, .max_deg = 180 },
};

static const char *const JOINT_NAMES[NUM_JOINTS] = {
    "FR_coxa_joint", "FR_femur_joint", "FR_tibia_joint",
    "FL_coxa_joint", "FL_femur_joint", "FL_tibia_joint",
    "RR_coxa_joint", "RR_femur_joint", "RR_tibia_joint",
    "RL_coxa_joint", "RL_femur_joint", "RL_tibia_joint",
};

// ─── micro-ROS plumbing ───────────────────────────────────────────────────

#define RCCHECK(fn) {                                                      \
    rcl_ret_t _rc = (fn);                                                  \
    if (_rc != RCL_RET_OK) {                                               \
        printf("RCCHECK fail line %d rc=%d\n", __LINE__, (int)_rc);        \
        vTaskDelete(NULL);                                                 \
    }                                                                      \
}
#define RCSOFTCHECK(fn) {                                                  \
    rcl_ret_t _rc = (fn);                                                  \
    if (_rc != RCL_RET_OK) {                                               \
        printf("RCSOFT fail line %d rc=%d\n", __LINE__, (int)_rc);         \
    }                                                                      \
}

static rcl_publisher_t imu_pub;
static rcl_publisher_t mag_pub;
static rcl_publisher_t joint_states_pub;
static rcl_publisher_t gas_pub;
static rcl_subscription_t joint_cmd_sub;

static sensor_msgs__msg__Imu              imu_msg;
static sensor_msgs__msg__MagneticField    mag_msg;
static sensor_msgs__msg__JointState       joint_states_msg;
static sensor_msgs__msg__JointState       joint_cmd_msg;
static std_msgs__msg__Float32             gas_msg;

static lsm9ds0_am_sensor_t *sensor_am;
static lsm9ds0_g_sensor_t  *sensor_g;

// Latest commanded joint positions (radians). Updated by the subscriber,
// re-published in /joint_states, and consumed by the watchdog. The watchdog
// stamp is bumped on every valid /joint_command message; if it goes stale,
// the servos are released so the robot sags rather than freezing.
static float    s_latest_cmd_rad[NUM_JOINTS] = {0};
static int64_t  s_last_cmd_us = 0;
static bool     s_watchdog_tripped = true;  // start tripped — wait for first cmd

// Gas sensor — single-pole IIR low-pass to smooth 60 Hz pickup before publish
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_adc_cali = NULL;
static adc_channel_t             s_adc_channel;
static float                     s_gas_filtered_mv = 0.0f;

// ─── Helpers ──────────────────────────────────────────────────────────────

static int find_joint_index(const char *name) {
    if (!name) return -1;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (strcmp(JOINT_NAMES[i], name) == 0) return i;
    }
    return -1;
}

static uint16_t joint_rad_to_pulse_us(int joint_index, float angle_rad) {
    const servo_cal_t *cal = &s_servo_cal[joint_index];
    float deg = SERVO_CENTER_DEG
              + (float)cal->direction * (angle_rad * DEG_PER_RAD)
              + cal->offset_deg;

    // Final clamp: even if the Pi-side safety node misbehaves, the servo
    // never gets driven outside its bench-tested mechanical envelope.
    if (deg < cal->min_deg) deg = cal->min_deg;
    if (deg > cal->max_deg) deg = cal->max_deg;

    float frac = deg / SERVO_RANGE_DEG;
    uint16_t us = (uint16_t)(SERVO_PWM_MIN_US
                  + frac * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
    return us;
}

static void apply_latest_to_servos(void) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        uint16_t us = joint_rad_to_pulse_us(i, s_latest_cmd_rad[i]);
        pca9685_set_pulse_us(us, s_servo_cal[i].channel);
    }
}

// ─── Subscriber callback ──────────────────────────────────────────────────

static void joint_cmd_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *m = (const sensor_msgs__msg__JointState *)msgin;
    if (!m || m->position.size == 0) return;

    bool name_ordered = (m->name.size == m->position.size);

    // Re-order by name so the Pi can publish in any order. If names are
    // missing we trust positional order — matches the Pi's
    // positions_from_joint_state() fallback.
    if (name_ordered) {
        for (size_t i = 0; i < m->position.size; ++i) {
            int j = find_joint_index(m->name.data[i].data);
            if (j < 0) continue;
            s_latest_cmd_rad[j] = (float)m->position.data[i];
        }
    } else {
        size_t n = m->position.size < NUM_JOINTS ? m->position.size : NUM_JOINTS;
        for (size_t i = 0; i < n; ++i) {
            s_latest_cmd_rad[i] = (float)m->position.data[i];
        }
    }

    s_last_cmd_us = esp_timer_get_time();
    if (s_watchdog_tripped) {
        ESP_LOGI("CMD", "joint command stream resumed");
        s_watchdog_tripped = false;
    }
}

// ─── Timer callbacks ──────────────────────────────────────────────────────

static void stamp_header_now(builtin_interfaces__msg__Time *stamp) {
    int64_t now_ns = rmw_uros_epoch_nanos();
    stamp->sec     = (int32_t)(now_ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(now_ns % 1000000000LL);
}

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (!timer) return;

    int64_t now_us = esp_timer_get_time();
    int64_t age_ms = (now_us - s_last_cmd_us) / 1000;

    // Watchdog: if the Pi has stopped talking, release every channel so the
    // robot sags rather than holding the last commanded pose forever. The
    // joint_states echo below keeps publishing the last command — that's
    // intentional, downstream consumers can see "MCU still alive, but
    // commands have stopped".
    if (age_ms > CONFIG_ARGOS_JOINT_CMD_TIMEOUT_MS) {
        if (!s_watchdog_tripped) {
            ESP_LOGW("WD", "joint command stale (%lld ms) — releasing servos", age_ms);
            pca9685_release_all();
            s_watchdog_tripped = true;
        }
    } else {
        apply_latest_to_servos();
    }

    // ── IMU read + publish ───────────────────────────────────────────────
    lsm9ds0_float_a_data_t a = {0};
    lsm9ds0_float_g_data_t g = {0};
    lsm9ds0_float_m_data_t m = {0};

    if (lsm9ds0_new_a_data(sensor_am) && lsm9ds0_get_float_a_data(sensor_am, &a)) {
        imu_msg.linear_acceleration.x = a.ax * GRAVITY_MS2;
        imu_msg.linear_acceleration.y = a.ay * GRAVITY_MS2;
        imu_msg.linear_acceleration.z = a.az * GRAVITY_MS2;
    }
    if (lsm9ds0_new_g_data(sensor_g) && lsm9ds0_get_float_g_data(sensor_g, &g)) {
        imu_msg.angular_velocity.x = g.x * DEG_TO_RAD;
        imu_msg.angular_velocity.y = g.y * DEG_TO_RAD;
        imu_msg.angular_velocity.z = g.z * DEG_TO_RAD;
    }
    if (lsm9ds0_new_m_data(sensor_am) && lsm9ds0_get_float_m_data(sensor_am, &m)) {
        mag_msg.magnetic_field.x = m.mx * 1e-4;
        mag_msg.magnetic_field.y = m.my * 1e-4;
        mag_msg.magnetic_field.z = m.mz * 1e-4;
    }
    imu_msg.orientation_covariance[0] = -1.0;  // signal "no orientation estimate"

    stamp_header_now(&imu_msg.header.stamp);
    stamp_header_now(&mag_msg.header.stamp);
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));

    // ── /joint_states echo ──────────────────────────────────────────────
    // Publishes the last commanded angles back, so the Pi can stop relying
    // on its preview publisher (see docs/pi_hardware_contract.md:88-89).
    stamp_header_now(&joint_states_msg.header.stamp);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joint_states_msg.position.data[i] = s_latest_cmd_rad[i];
    }
    RCSOFTCHECK(rcl_publish(&joint_states_pub, &joint_states_msg, NULL));
}

static void gas_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (!timer || !s_adc_handle) return;

    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, s_adc_channel, &raw) != ESP_OK) return;

    int mv = 0;
    if (s_adc_cali && adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK) {
        // calibrated reading available — use mV
    } else {
        // Fallback if cal scheme isn't supported: fake a linear mapping over
        // 0..3300 mV across the 12-bit raw range.
        mv = (raw * 3300) / 4095;
    }

    // Single-pole IIR low-pass — alpha=0.2 trades off settling (~5 samples ≈
    // 0.5 s at 10 Hz) against 60 Hz pickup rejection. Tune if needed.
    s_gas_filtered_mv = 0.8f * s_gas_filtered_mv + 0.2f * (float)mv;

    gas_msg.data = s_gas_filtered_mv;
    RCSOFTCHECK(rcl_publish(&gas_pub, &gas_msg, NULL));
}

// ─── Init helpers ─────────────────────────────────────────────────────────

static void init_imu(void) {
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    sensor_am = lsm9ds0_init_am_sensor(I2C_BUS, LSM9DS0_I2C_AM_ADDRESS_2, 0);
    sensor_g  = lsm9ds0_init_g_sensor(I2C_BUS, LSM9DS0_I2C_G_ADDRESS_2, 0);
    if (!sensor_am || !sensor_g) {
        ESP_LOGE("IMU", "LSM9DS0 init failed — check wiring/I2C address");
        return;
    }
    ESP_LOGI("IMU", "LSM9DS0 initialized");

    lsm9ds0_set_a_scale(sensor_am, lsm9ds0_a_scale_2_g);
    lsm9ds0_set_m_scale(sensor_am, lsm9ds0_m_scale_4_Gs);
    lsm9ds0_set_g_scale(sensor_g,  lsm9ds0_g_scale_245_dps);

    // Bumped to 100 Hz accel ODR to actually feed the new control loop rate.
    lsm9ds0_set_a_mode(sensor_am, lsm9ds0_a_odr_100, lsm9ds0_a_aaf_bw_773, true, true, true);
    lsm9ds0_set_m_mode(sensor_am, lsm9ds0_m_odr_100, lsm9ds0_m_low_res, lsm9ds0_m_continuous);
    lsm9ds0_set_g_mode(sensor_g,  lsm9ds0_g_odr_190, 3, true, true, true);
}

// PCA9685 shares I2C_BUS with the IMU. The esp8266_wrapper i2c_init above
// configures the bus; here we just re-use the standard ESP-IDF driver port,
// which is wired to the same pins via the wrapper's call to i2c_param_config.
static void init_servos(void) {
    esp_err_t err = pca9685_init(PCA9685_I2C_PORT,
                                 CONFIG_ARGOS_PCA9685_ADDR,
                                 SERVO_PWM_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGE("SERVO", "PCA9685 init failed: %s", esp_err_to_name(err));
        return;
    }
    pca9685_release_all();  // start limp; first /joint_command brings them up
}

static void init_gas_adc(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    if (adc_oneshot_new_unit(&init_cfg, &s_adc_handle) != ESP_OK) {
        ESP_LOGE("GAS", "adc_oneshot_new_unit failed");
        return;
    }

    // ESP32-C6 ADC1 maps GPIO0..GPIO6 to channel 0..6 directly.
    s_adc_channel = (adc_channel_t)CONFIG_ARGOS_GAS_ADC_GPIO;

    adc_oneshot_chan_cfg_t cfg = {
        .atten    = ADC_ATTEN_DB_11,    // ~0..3.1 V usable
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_oneshot_config_channel(s_adc_handle, s_adc_channel, &cfg);

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = s_adc_channel,
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) != ESP_OK) {
        ESP_LOGW("GAS", "ADC calibration unavailable, using linear fallback");
        s_adc_cali = NULL;
    }
}

static bool init_joint_state_msgs(void) {
    // /joint_states echo — preallocate name + position sequences so the
    // 100 Hz timer callback doesn't allocate.
    if (!rosidl_runtime_c__String__init(&joint_states_msg.header.frame_id)) return false;
    rosidl_runtime_c__String__assign(&joint_states_msg.header.frame_id, "base_link");

    if (!rosidl_runtime_c__String__Sequence__init(&joint_states_msg.name, NUM_JOINTS)) return false;
    if (!rosidl_runtime_c__double__Sequence__init(&joint_states_msg.position, NUM_JOINTS)) return false;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        rosidl_runtime_c__String__assign(&joint_states_msg.name.data[i], JOINT_NAMES[i]);
        joint_states_msg.position.data[i] = 0.0;
    }

    // /joint_command subscriber — preallocate up to NUM_JOINTS entries
    if (!rosidl_runtime_c__String__Sequence__init(&joint_cmd_msg.name, NUM_JOINTS)) return false;
    if (!rosidl_runtime_c__double__Sequence__init(&joint_cmd_msg.position, NUM_JOINTS)) return false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        // Pre-size each name string buffer so the rclc_executor can fill it
        // without dynamic allocation in the rt path.
        rosidl_runtime_c__String__init(&joint_cmd_msg.name.data[i]);
        rosidl_runtime_c__String__assign(&joint_cmd_msg.name.data[i], "");
    }

    return true;
}

// ─── micro-ROS task ───────────────────────────────────────────────────────

static void micro_ros_task(void *arg) {
    init_imu();
    init_servos();
    init_gas_adc();

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
    ESP_LOGI("UROS", "ping agent…");
    rmw_uros_ping_agent(1000, 3);
#endif

    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    if (rc != RCL_RET_OK) {
        ESP_LOGE("UROS", "support init failed rc=%d", (int)rc);
        vTaskDelete(NULL);
        return;
    }
    rmw_uros_sync_session(1000);

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "argos_esp32c6", "", &support));

    // ── IMU + mag ────────────────────────────────────────────────────────
    rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1.0;
    rosidl_runtime_c__String__init(&mag_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&mag_msg.header.frame_id, "imu_link");

    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw"));
    RCCHECK(rclc_publisher_init_default(
        &mag_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "/imu/mag"));

    // ── /joint_states + /joint_command ──────────────────────────────────
    if (!init_joint_state_msgs()) {
        ESP_LOGE("UROS", "joint state msg init failed");
        vTaskDelete(NULL);
        return;
    }
    RCCHECK(rclc_publisher_init_default(
        &joint_states_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"));
    RCCHECK(rclc_subscription_init_default(
        &joint_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_command"));

    // ── /gas ────────────────────────────────────────────────────────────
    RCCHECK(rclc_publisher_init_default(
        &gas_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/gas"));

    // ── timers ──────────────────────────────────────────────────────────
    rcl_timer_t control_timer, gas_timer;
    const uint32_t control_period_ms = 1000U / CONFIG_ARGOS_IMU_RATE_HZ;
    const uint32_t gas_period_ms     = 1000U / CONFIG_ARGOS_GAS_RATE_HZ;
    RCCHECK(rclc_timer_init_default2(&control_timer, &support,
                                     RCL_MS_TO_NS(control_period_ms),
                                     control_timer_callback, true));
    RCCHECK(rclc_timer_init_default2(&gas_timer, &support,
                                     RCL_MS_TO_NS(gas_period_ms),
                                     gas_timer_callback, true));

    // 1 sub + 2 timers = 3 handles
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &joint_cmd_sub, &joint_cmd_msg,
                                           &joint_cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &gas_timer));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        usleep(1000);
    }

    // (unreachable but tidy)
    rcl_subscription_fini(&joint_cmd_sub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_publisher_fini(&mag_pub, &node);
    rcl_publisher_fini(&joint_states_pub, &node);
    rcl_publisher_fini(&gas_pub, &node);
    rcl_node_fini(&node);
    vTaskDelete(NULL);
}

void app_main(void) {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}
