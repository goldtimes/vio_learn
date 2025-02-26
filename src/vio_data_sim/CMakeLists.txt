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
