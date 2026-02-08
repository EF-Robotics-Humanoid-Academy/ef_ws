cmake_minimum_required(VERSION 3.10)
project(g1_webserver)

set(AMENT_DEPENDENCIES rclpy tf2 tf2_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(DIRECTORY config launch g1_webserver navi_outdoor_waypoints navi_outdoor_nogozones
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS scripts/webserver
  scripts/add_host_name
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
