cmake_minimum_required(VERSION 3.5)

# Where to find micro_ros_espidf_component. Adjust if you keep it elsewhere.
# Default assumes a sibling layout:
#   <parent>/micro_ros_espidf_component/
#   <parent>/Argos/firmware/esp32c6/    <-- this project
set(EXTRA_COMPONENT_DIRS "./../../../micro_ros_espidf_component"
                        "./../../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(argos_esp32c6)

