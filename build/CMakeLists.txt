cmake_minimum_required(VERSION 3.16.3)
project(mujoco_control)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
# Compiler options
add_compile_options(-O2 -pthread)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/include
    # ${CMAKE_SOURCE_DIR}/../../../../include
    ${CMAKE_SOURCE_DIR}/../../../include
    ${CMAKE_SOURCE_DIR}/../../include
    ${CMAKE_SOURCE_DIR}/../src
    ${CMAKE_SOURCE_DIR}/../
)

# Link directories
link_directories(${CMAKE_SOURCE_DIR}/../../lib)

# Source files for mujoco_control executable
set(SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/../src/lodepng.cpp
    ${CMAKE_SOURCE_DIR}/../src/simulate.cc
    ${CMAKE_SOURCE_DIR}/../src/uitools.c
    ${CMAKE_SOURCE_DIR}/../src/glfw_adapter.cc
    ${CMAKE_SOURCE_DIR}/../src/glfw_dispatch.cc
    ${CMAKE_SOURCE_DIR}/../src/platform_ui_adapter.cc
    ${CMAKE_SOURCE_DIR}/../src/ControlSystem.cpp
    ${CMAKE_SOURCE_DIR}/../src/tool_kits.cpp
    ${CMAKE_SOURCE_DIR}/../include/franka_ik_He.hpp
    ${CMAKE_SOURCE_DIR}/../src/data_handler.cpp
    # ${CMAKE_SOURCE_DIR}/include/franka_ik_He.
)

add_library(tools_and_panda_ik
    ${CMAKE_SOURCE_DIR}/../src/tool_kits.cpp
    ${CMAKE_SOURCE_DIR}/../include/franka_ik_He.hpp  # Header-only? If so, it's not needed here.
    # Add other source files that make up the tools_and_panda_ik library
)
# src
# target_link_libraries(${PROJECT_NAME} PRIVATE tools_and_panda_ik)

# Executable for mujoco_control
add_executable(${PROJECT_NAME} ../apps/main.cc ${SOURCE_FILES})

add_executable(mujoco_control2 ../apps/main_panda_RH8D.cpp ../src/data_handler.cpp ../src/controller.cpp ../src/ControlSystem.cpp)


# Linking libraries
target_link_libraries(mujoco_control PRIVATE mujoco glfw Threads::Threads)

target_link_libraries(mujoco_control2 PRIVATE mujoco Eigen3::Eigen Threads::Threads glfw GL m tools_and_panda_ik)

# Set the output directory for the executables
# set_target_properties(mujoco_control  PROPERTIES
#                       RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
# 