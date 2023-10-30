cmake_minimum_required(VERSION 3.16.3)

project(mujoco_control)

add_library(
    tools_and_panda_ik    
    ${CMAKE_SOURCE_DIR}/include/tool_kits.cpp 
    ${CMAKE_SOURCE_DIR}/include/franka_ik_He.hpp
    # ${CMAKE_SOURCE_DIR}/include/data_handler.h
)

find_package(Eigen3 REQUIRED)


LINK_LIBRARIES(${CMAKE_SOURCE_DIR}/../../lib)

add_executable(${PROJECT_NAME} main_panda_RH8D.cpp data_handler.cpp)
#find_package(MUJOCO_LIB NAMES mujoco PATHS /home/jididu/Documents/mujoco-2.3.7/lib)

#add_library(mylib src/add.cpp src/sub.cpp)


target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../bin)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../../../lib)



#target_link_libraries(${PROJECT_NAME}${MUJOCO_LIB})


target_link_libraries(${PROJECT_NAME} PRIVATE mujoco)
#target_link_libraries(${PROJECT_NAME} PRIVATE ${MUJOCO_LIB})
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME} PRIVATE -lglfw)
target_link_libraries(${PROJECT_NAME} PRIVATE -lGL)
target_link_libraries(${PROJECT_NAME} PRIVATE -lm)

target_link_libraries(${PROJECT_NAME} PRIVATE tools_and_panda_ik)
