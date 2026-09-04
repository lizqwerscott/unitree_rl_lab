cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_library(ZMQ_LIBRARY NAMES zmq)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
if(NOT ZMQ_LIBRARY OR NOT CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR "ZeroMQ development files are required (libzmq and zmq.hpp/cppzmq)")
endif()
find_package(tinyxml2 QUIET CONFIG)
if(NOT TARGET tinyxml2::tinyxml2)
  find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
  find_library(TINYXML2_LIBRARY NAMES tinyxml2)
  if(NOT TINYXML2_INCLUDE_DIR OR NOT TINYXML2_LIBRARY)
    message(FATAL_ERROR "tinyxml2 development files are required (tinyxml2.h and libtinyxml2)")
  endif()
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    IMPORTED_LOCATION "${TINYXML2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}")
endif()

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH)

message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Normalized architecture: ${ARCH}")

set(THIRDPARTY_DIR ${PROJECT_SOURCE_DIR}/../../thirdparty)

# 根据架构设置 ONNX Runtime 的目录名
if(ARCH MATCHES "x86_64|amd64")
    set(ONNXRUNTIME_ARCH_DIR "onnxruntime-linux-x64-1.22.0")
    message(STATUS "Using ONNX Runtime: ${ONNXRUNTIME_ARCH_DIR} (x86_64/amd64)")
elseif(ARCH MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_ARCH_DIR "onnxruntime-linux-aarch64-1.16.0-self")
    # set(ONNXRUNTIME_ARCH_DIR "onnxruntime-linux-aarch64-1.22.0")
    message(STATUS "Using ONNX Runtime: ${ONNXRUNTIME_ARCH_DIR} (aarch64/arm64)")
else()
    message(FATAL_ERROR "Unsupported architecture for ONNX Runtime: ${ARCH}")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  ${CPPZMQ_INCLUDE_DIR}
  include
  ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  tinyxml2::tinyxml2
  ${ZMQ_LIBRARY}
)

if(ARCH MATCHES "x86_64|amd64")
  link_libraries(
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime.so.1.22.0
  )
elseif(ARCH MATCHES "aarch64|arm64")
  link_libraries(
    # ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime.so.1.22.0
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime.so.1.16.0
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime_providers_cuda.so
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime_providers_cuda_ut.so
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime_providers_shared.so
    ${THIRDPARTY_DIR}/${ONNXRUNTIME_ARCH_DIR}/lib/libonnxruntime_providers_tensorrt.so
  )
else()
    message(FATAL_ERROR "Unsupported architecture for ONNX Runtime: ${ARCH}")
endif()

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)
target_link_libraries(g1_ctrl PRIVATE ${PROJECT_NAME}_lib)
