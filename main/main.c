#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "lsm9ds0.h"

// IMU I2C pin setup for ESP32C6
#define I2C_BUS 0
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 23
#define I2C_FREQ I2C_FREQ_100K

#define GRAVITY_MS2 9.80665f
#define DEG_TO_RAD 0.017453293f

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn)                                                                      \
	{                                                                                    \
		rcl_ret_t temp_rc = fn;                                                          \
		if ((temp_rc != RCL_RET_OK))                                                     \
		{                                                                                \
			printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
			vTaskDelete(NULL);                                                           \
		}                                                                                \
	}
#define RCSOFTCHECK(fn)                                                                    \
	{                                                                                      \
		rcl_ret_t temp_rc = fn;                                                            \
		if ((temp_rc != RCL_RET_OK))                                                       \
		{                                                                                  \
			printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
		}                                                                                  \
	}

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;

// IMU
static lsm9ds0_am_sensor_t *sensor_am;
static lsm9ds0_g_sensor_t *sensor_g;

/*
void imu_test_task(void *pvParameters)
{
	// Init I2C bus
	i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

	// Init both sub-sensors (I2C address 2 = SDO pulled high)
	sensor_am = lsm9ds0_init_am_sensor(I2C_BUS, LSM9DS0_I2C_AM_ADDRESS_2, 0);
	sensor_g = lsm9ds0_init_g_sensor(I2C_BUS, LSM9DS0_I2C_G_ADDRESS_2, 0);

	if (!sensor_am || !sensor_g)
	{
		ESP_LOGE("IMU", "LSM9DS0 init failed — check wiring and I2C address");
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI("IMU", "LSM9DS0 initialized OK");

	// Set scales and start measurements
	lsm9ds0_set_a_scale(sensor_am, lsm9ds0_a_scale_2_g);
	lsm9ds0_set_m_scale(sensor_am, lsm9ds0_m_scale_4_Gs);
	lsm9ds0_set_g_scale(sensor_g, lsm9ds0_g_scale_245_dps);

	lsm9ds0_set_a_mode(sensor_am, lsm9ds0_a_odr_12_5, lsm9ds0_a_aaf_bw_773, true, true, true);
	lsm9ds0_set_m_mode(sensor_am, lsm9ds0_m_odr_12_5, lsm9ds0_m_low_res, lsm9ds0_m_continuous);
	lsm9ds0_set_g_mode(sensor_g, lsm9ds0_g_odr_95, 3, true, true, true);

	while (1)
	{
		lsm9ds0_float_a_data_t a;
		lsm9ds0_float_m_data_t m;
		lsm9ds0_float_g_data_t g;

		if (lsm9ds0_new_a_data(sensor_am) && lsm9ds0_get_float_a_data(sensor_am, &a))
			ESP_LOGI("IMU", "Accel [g]  ax=%+6.3f ay=%+6.3f az=%+6.3f", a.ax, a.ay, a.az);

		if (lsm9ds0_new_m_data(sensor_am) && lsm9ds0_get_float_m_data(sensor_am, &m))
			ESP_LOGI("IMU", "Mag   [Gs] mx=%+6.3f my=%+6.3f mz=%+6.3f", m.mx, m.my, m.mz);

		if (lsm9ds0_new_g_data(sensor_g) && lsm9ds0_get_float_g_data(sensor_g, &g))
			ESP_LOGI("IMU", "Gyro [dps] gx=%+7.3f gy=%+7.3f gz=%+7.3f", g.x, g.y, g.z);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
*/

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		lsm9ds0_float_a_data_t a = {};
		lsm9ds0_float_g_data_t g = {};

		// accelerometer
		if (lsm9ds0_new_a_data(sensor_am) && lsm9ds0_get_float_a_data(sensor_am, &a))
		{
			msg.linear_acceleration.x = a.ax * GRAVITY_MS2;
			msg.linear_acceleration.y = a.ay * GRAVITY_MS2;
			msg.linear_acceleration.z = a.az * GRAVITY_MS2;
		}

		// gyroscope
		if (lsm9ds0_new_g_data(sensor_g) && lsm9ds0_get_float_g_data(sensor_g, &g))
		{
			msg.angular_velocity.x = g.x * DEG_TO_RAD;
			msg.angular_velocity.y = g.y * DEG_TO_RAD;
			msg.angular_velocity.z = g.z * DEG_TO_RAD;
		}

		// orientation unknown — set covariance[0] = -1 to signal this to ROS
		msg.orientation_covariance[0] = -1.0;

		// Stamp the header
		int64_t now_ns = rmw_uros_epoch_nanos();
		msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
		msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		ESP_LOGI("IMU", "a: x=%+.3f y=%+.3f z=%+.3f | g: x=%+.3f y=%+.3f z=%+.3f",
				 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
				 msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
	}
}

void micro_ros_task(void *arg)
{
	// --- Init IMU first ---
	i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

	sensor_am = lsm9ds0_init_am_sensor(I2C_BUS, LSM9DS0_I2C_AM_ADDRESS_2, 0);
	sensor_g = lsm9ds0_init_g_sensor(I2C_BUS, LSM9DS0_I2C_G_ADDRESS_2, 0);

	if (!sensor_am || !sensor_g)
	{
		ESP_LOGE("IMU", "LSM9DS0 init failed — check wiring and I2C address");
		vTaskDelete(NULL);
		return;
	}
	ESP_LOGI("IMU", "LSM9DS0 initialized OK");

	lsm9ds0_set_a_scale(sensor_am, lsm9ds0_a_scale_2_g);
	lsm9ds0_set_m_scale(sensor_am, lsm9ds0_m_scale_4_Gs);
	lsm9ds0_set_g_scale(sensor_g, lsm9ds0_g_scale_245_dps);

	lsm9ds0_set_a_mode(sensor_am, lsm9ds0_a_odr_12_5, lsm9ds0_a_aaf_bw_773, true, true, true);
	lsm9ds0_set_m_mode(sensor_am, lsm9ds0_m_odr_12_5, lsm9ds0_m_low_res, lsm9ds0_m_continuous);
	lsm9ds0_set_g_mode(sensor_g, lsm9ds0_g_odr_95, 3, true, true, true);

	// --- Init micro-ROS ---
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
											 CONFIG_MICRO_ROS_AGENT_PORT,
											 rmw_options));
	ESP_LOGI("UROS", "ping agent...");
	rmw_ret_t ping_rc = rmw_uros_ping_agent(1000, 3);
	ESP_LOGI("UROS", "ping rc = %d", (int)ping_rc);
#endif

	ESP_LOGI("UROS", "support init");
	// RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
	ESP_LOGI("UROS", "support init rc = %d", (int)rc);

	if (rc != RCL_RET_OK)
	{
		ESP_LOGE("UROS", "support init failed");
		vTaskDelay(pdMS_TO_TICKS(100));
		vTaskDelete(NULL);
		return;
	}

	// Sync time with agent so timestamps are meaningful
	rmw_uros_sync_session(1000);

	rcl_node_t node;
	ESP_LOGI("UROS", "node init");
	RCCHECK(rclc_node_init_default(&node, "imu_publisher", "", &support));

	ESP_LOGI("UROS", "imu msg init");
	if (!rosidl_runtime_c__String__init(&msg.header.frame_id))
	{
		ESP_LOGE("UROS", "frame_id init failed");
		vTaskDelete(NULL);
		return;
	}

	if (!rosidl_runtime_c__String__assign(&msg.header.frame_id, "imu_link"))
	{
		ESP_LOGE("UROS", "frame_id assign failed");
		vTaskDelete(NULL);
		return;
	}
	msg.orientation_covariance[0] = -1.0;

	ESP_LOGI("UROS", "publisher init");
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu/data_raw"));

	rcl_timer_t timer;
	ESP_LOGI("UROS", "timer init");
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(100), // 10 Hz publish rate
		timer_callback,
		true));

	rclc_executor_t executor;
	ESP_LOGI("UROS", "executor init");
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	ESP_LOGI("UROS", "executor add timer");
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while (1)
	{
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	rosidl_runtime_c__String__fini(&msg.header.frame_id);

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	// xTaskCreate(imu_test_task, "imu_test", 2048, NULL, 5, NULL);
	//  pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	xTaskCreate(micro_ros_task,
				"uros_task",
				CONFIG_MICRO_ROS_APP_STACK,
				NULL,
				CONFIG_MICRO_ROS_APP_TASK_PRIO,
				NULL);
}
