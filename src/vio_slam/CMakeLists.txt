find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    src
)

LIST(APPEND target_lib
    ${Sophus_LIBRARIES}
)

