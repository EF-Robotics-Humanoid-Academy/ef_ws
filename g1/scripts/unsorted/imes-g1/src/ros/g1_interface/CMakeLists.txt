cmake_minimum_required(VERSION 3.5)
project(g1_interface)
add_compile_options(-std=c++17)

set(AMENT_DEPENDENCIES std_srvs 
                       std_msgs
                       )

set(srv_files 
                srv/G1Modes.srv
                srv/G1Light.srv
                srv/G1Volume.srv
)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                        "srv/G1Modes.srv"
                        "srv/G1Light.srv"
                        "srv/G1Volume.srv"
)

ament_package()