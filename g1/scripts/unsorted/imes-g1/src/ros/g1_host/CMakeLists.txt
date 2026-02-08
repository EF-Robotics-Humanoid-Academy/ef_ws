cmake_minimum_required(VERSION 3.5)
project(g1_host)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(AMENT_DEPENDENCIES rclpy tf2 tf2_ros)
# Core dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# -------------------------------------------------------- ROS2 
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/logitech_f710
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------- ROS2 
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
