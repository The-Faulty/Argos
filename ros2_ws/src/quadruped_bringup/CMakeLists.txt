cmake_minimum_required(VERSION 3.8)
project(quadruped_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/argos_component_check
  scripts/argos_joint_jog
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
