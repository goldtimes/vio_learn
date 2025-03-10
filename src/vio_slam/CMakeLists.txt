find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

find_package(GTest REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    src/backend
    src/thirdparty/sophus
    src/vins_code
    ${Boost_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

message(${PROJECT_SOURCE_DIR})
# sophus

add_library(${PROJECT_NAME}_backend 
        src/backend/vertex.cc
        src/backend/vertex_pose.cc
        src/backend/edge.cc
        src/backend/edge_reprojection.cc
        src/backend/problem.cc
        src/backend/problem_schur.cc
)

add_executable(curve_fitting src/curve_fitting/curve_fitting.cc)
target_link_libraries(curve_fitting ${PROJECT_NAME}_backend)

add_executable(nullspace_test src/nullspace/hessian_nullspace_test.cc)
target_link_libraries(nullspace_test ${PROJECT_NAME}_backend)

add_executable(monoBA src/ba_optimization/MonoBA.cc)
target_link_libraries(monoBA ${PROJECT_NAME}_backend)

add_executable(triangulation src/triangulation/triangulation.cc)
target_link_libraries(triangulation ${PROJECT_NAME}_backend)

add_executable(run_eurco 
    src/vins_code/app/run_eurco.cc
    src/vins_code/system.cc
    src/vins_code/system_config.cc

)
target_link_libraries(run_eurco 
    ${PROJECT_NAME}_backend
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBS}
    -lpthread
)

