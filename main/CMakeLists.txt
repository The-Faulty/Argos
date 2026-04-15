idf_component_register(SRCS main.c  
                    INCLUDE_DIRS "."
                    REQUIRES lsm9ds0 esp8266_wrapper lsm303d l3gd20h micro_ros_espidf_component)