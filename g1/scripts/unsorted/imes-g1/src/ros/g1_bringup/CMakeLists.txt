cmake_minimum_required(VERSION 3.5)
project(g1_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add Launch File
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Executables
install(PROGRAMS
  scripts/startup_installer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  hooks/auto.bash
  DESTINATION lib/${PROJECT_NAME}
)

# Auto source bash
install(
  DIRECTORY hooks/
  DESTINATION share/${PROJECT_NAME}/hooks
)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/auto.bash")

# Build Packages
ament_package()