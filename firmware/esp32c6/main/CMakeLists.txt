idf_component_register(SRCS main.c esp32_serial_transport.c
                       INCLUDE_DIRS "."
                       REQUIRES lsm9ds0 esp8266_wrapper lsm303d l3gd20h
                                pca9685 esp_adc esp_timer driver
                                micro_ros_espidf_component)
