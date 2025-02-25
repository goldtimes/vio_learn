CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set(PROJECT_NAME "vio_data_simluation")
PROJECT(${PROJECT_NAME})
# 设置可执行文件的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# # 设置动态库的文件目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
# 设置c++的版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    src
)

LIST(APPEND target_lib
    ${Sophus_LIBRARIES}
)

add_executable(gen_imu src/gen_imu.cc)
target_link_libraries(gen_imu ${target_lib})





