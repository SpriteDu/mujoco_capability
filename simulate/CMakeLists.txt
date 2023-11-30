# cmake_minimum_required(VERSION 3.16.3)

# project(mujoco_control)

# add_library(
#     tools_and_panda_ik    
#     ${CMAKE_SOURCE_DIR}/include/tool_kits.cpp 
#     ${CMAKE_SOURCE_DIR}/include/franka_ik_He.hpp
#     # ${CMAKE_SOURCE_DIR}/include/data_handler.h
#     ${CMAKE_SOURCE_DIR}/glfw_adapter.cc 
#     ${CMAKE_SOURCE_DIR}/glfw_dispatch.cc 
#     ${CMAKE_SOURCE_DIR}/simulate.cc 
#     ${CMAKE_SOURCE_DIR}/platform_ui_adapter.cc
#     ${CMAKE_SOURCE_DIR}/uitools.c 
# )

# find_package(Eigen3 REQUIRED)


# LINK_LIBRARIES(${CMAKE_SOURCE_DIR}/../../lib)

# add_executable(${PROJECT_NAME} main.cc src/data_handler.cpp src/controller.cpp)
# #find_package(MUJOCO_LIB NAMES mujoco PATHS /home/jididu/Documents/mujoco-2.3.7/lib)

# #add_library(mylib src/add.cpp src/sub.cpp)


# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../bin)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../include)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../../lib)



# #target_link_libraries(${PROJECT_NAME}${MUJOCO_LIB})


# target_link_libraries(${PROJECT_NAME} PRIVATE mujoco)
# #target_link_libraries(${PROJECT_NAME} PRIVATE ${MUJOCO_LIB})
# target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# target_link_libraries(${PROJECT_NAME} PRIVATE -lglfw)
# target_link_libraries(${PROJECT_NAME} PRIVATE -lGL)
# target_link_libraries(${PROJECT_NAME} PRIVATE -lm)

# target_link_libraries(${PROJECT_NAME} PRIVATE tools_and_panda_ik)




# //////////////////

cmake_minimum_required(VERSION 3.0)
project(mujoco_control)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O2 -pthread -Wl,-no-as-needed)

# Include directories
include_directories(../include)

# Link directories
link_directories(../lib)

# Find packages
# If GLFW is not found automatically, you might need to use find_package(glfw3 REQUIRED)
# or specify the path manually

# Specify the source files
add_executable(mujoco_control
    lodepng.cpp
    simulate.cc
    uitools.c
    glfw_adapter.cc
    glfw_dispatch.cc
    platform_ui_adapter.cc
    main.cc
)

# Link with libraries
# Replace 'mujoco' with the actual library name if different
target_link_libraries(mujoco_control mujoco glfw)


# Uncomment if you have other libraries to link
# target_link_libraries(${PROJECT_NAME} PRIVATE ${MUJOCO_LIB})
