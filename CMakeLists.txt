cmake_minimum_required(VERSION 3.0)
project(pure_ins)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 主程序
add_executable(${PROJECT_NAME} src/main.cpp src/imu.cpp src/imu_update.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# 进行零速修正的程序
add_executable(${PROJECT_NAME}_check src/main_check.cpp src/imu.cpp src/imu_update.cpp)
set_target_properties(${PROJECT_NAME}_check PROPERTIES CXX_STANDARD 14)
set_target_properties(${PROJECT_NAME}_check PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)