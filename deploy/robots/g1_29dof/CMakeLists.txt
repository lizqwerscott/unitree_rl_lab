cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)

set(LOCAL_INSTALL_DIR ${CMAKE_BINARY_DIR}/install)

add_subdirectory(thirdparty)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

include_directories(
  # /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_dependencies(${PROJECT_NAME}_lib pinocchio)

target_link_directories(${PROJECT_NAME}_lib PUBLIC ${LOCAL_INSTALL_DIR}/lib)
target_include_directories(
  ${PROJECT_NAME}_lib SYSTEM PUBLIC ${LOCAL_INSTALL_DIR}/include
                               ${LOCAL_INSTALL_DIR}/include/eigen3)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC pinocchio_default pinocchio_parsers)

add_executable(g1_ctrl main.cpp)
