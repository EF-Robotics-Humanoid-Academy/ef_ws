cmake_minimum_required(VERSION 3.5)
project(g1_obstacle_avoidance)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(g1_interface REQUIRED)

# C++ executable
add_executable(simple_obstacle_avoidance 
  src/obstacle_avoidance.cpp
)

ament_target_dependencies(simple_obstacle_avoidance
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  g1_interface
)

# Install C++ executable
install(TARGETS simple_obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()