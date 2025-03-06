find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    src/backend
)

add_library(${PROJECT_NAME}_backend 
        src/backend/vertex.cc
        src/backend/edge.cc
        src/backend/problem.cc
)

add_executable(curve_fitting src/curve_fitting/curve_fitting.cc)
target_link_libraries(curve_fitting ${PROJECT_NAME}_backend)

add_executable(nullspace_test src/nullspace/hessian_nullspace_test.cc)
target_link_libraries(nullspace_test ${PROJECT_NAME}_backend)