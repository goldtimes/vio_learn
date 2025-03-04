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