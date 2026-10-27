cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(g1_sonic_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(TensorRT REQUIRED COMPONENTS nvinfer nvonnxparser)

find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp REQUIRED)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)

set(BUILD_EXAMPLES OFF CACHE BOOL "Build Unitree SDK examples" FORCE)
add_subdirectory(thirdparty/unitree_sdk2)

add_library(TRTInference
  src/TRTInference/InferenceEngine.cpp
  src/TRTInference/InferenceEngine.h
  src/TRTInference/InferenceEngine.inl
  src/TRTInference/Utility.h
)
target_include_directories(TRTInference PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${TensorRT_INCLUDE_DIRS}"
  "${CUDAToolkit_INCLUDE_DIRS}"
)
target_link_libraries(TRTInference PRIVATE
  CUDA::cudart
  ${TensorRT_nvinfer_LIBRARY}
  ${TensorRT_nvonnxparser_LIBRARY}
)

add_library(audio_thread
  src/audio_thread/audio_thread.cpp
)
target_include_directories(audio_thread PUBLIC src/audio_thread)
target_link_libraries(audio_thread PRIVATE unitree_sdk2)

set(G1_DEPLOY_DIR src/g1/g1_deploy_onnx_ref)
add_library(g1_deploy_shared STATIC
  ${G1_DEPLOY_DIR}/src/cnpy.cpp
  ${G1_DEPLOY_DIR}/src/file_sink.cpp
  ${G1_DEPLOY_DIR}/src/fk.cpp
  ${G1_DEPLOY_DIR}/src/state_logger.cpp
)
target_include_directories(g1_deploy_shared PUBLIC
  ${G1_DEPLOY_DIR}/include
  ${CPPZMQ_INCLUDE_DIR}
  ${MSGPACK_INCLUDE_DIR}
  ${NLOHMANN_JSON_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(g1_deploy_shared PRIVATE ZLIB::ZLIB)

add_executable(g1_deploy_onnx_ref
  ${G1_DEPLOY_DIR}/src/g1_deploy_onnx_ref.cpp
)
target_compile_definitions(g1_deploy_onnx_ref PRIVATE HAS_ROS2=0)
target_compile_options(g1_deploy_onnx_ref PRIVATE -O3 -ffast-math)
target_include_directories(g1_deploy_onnx_ref PRIVATE
  ${G1_DEPLOY_DIR}/include
  src
  ${CPPZMQ_INCLUDE_DIR}
  ${MSGPACK_INCLUDE_DIR}
  ${NLOHMANN_JSON_INCLUDE_DIR}
  ${ZMQ_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(g1_deploy_onnx_ref PRIVATE
  g1_deploy_shared
  audio_thread
  unitree_sdk2
  ZLIB::ZLIB
  onnxruntime::onnxruntime
  TRTInference
  CUDA::cudart
  ${TensorRT_nvinfer_LIBRARY}
  ${TensorRT_nvonnxparser_LIBRARY}
  ${ZMQ_LIBRARY}
  Threads::Threads
)

set_target_properties(g1_deploy_onnx_ref PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
)
