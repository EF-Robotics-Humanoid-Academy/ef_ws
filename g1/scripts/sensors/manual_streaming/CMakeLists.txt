cmake_minimum_required(VERSION 3.0)
project(tx_lidar)

set(DEMO_NAME tx_lidar)
add_executable(${DEMO_NAME} main.cpp)

find_package(Threads REQUIRED)

target_link_libraries(${DEMO_NAME}
        PUBLIC
        livox_lidar_sdk_static
        Threads::Threads
                                )

