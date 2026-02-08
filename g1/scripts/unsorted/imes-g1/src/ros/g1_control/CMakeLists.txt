cmake_minimum_required(VERSION 3.5)
project(g1_control)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

set(AMENT_DEPENDENCIES rclpy tf2 tf2_ros)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()