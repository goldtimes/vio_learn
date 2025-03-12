find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Ceres REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    src/backend
    src/thirdparty/sophus
    src/vins_code
    src/vins_code/include
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
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


add_library(camera_model SHARED
    src/vins_code/camera_models/chessboard/Chessboard.cc
    src/vins_code/camera_models/calib/CameraCalibration.cc
    src/vins_code/camera_models/camera_models/Camera.cc
    src/vins_code/camera_models/camera_models/CameraFactory.cc
    src/vins_code/camera_models/camera_models/CostFunctionFactory.cc
    src/vins_code/camera_models/camera_models/PinholeCamera.cc
    src/vins_code/camera_models/camera_models/CataCamera.cc
    src/vins_code/camera_models/camera_models/EquidistantCamera.cc
    src/vins_code/camera_models/camera_models/ScaramuzzaCamera.cc
    src/vins_code/camera_models/sparse_graph/Transform.cc
    src/vins_code/camera_models/gpl/gpl.cc
    src/vins_code/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_library(vins_system_lib
    src/vins_code/system.cc
    src/vins_code/system_config.cc
    src/vins_code/feature_track.cc

    src/vins_code/backend/vertex.cc
    src/vins_code/backend/edge.cc
    src/vins_code/backend/problem.cc
    src/vins_code/backend/vertex_pose.cc
    src/vins_code/backend/edge_reprojection.cc
    src/vins_code/backend/edge_imu.cc
    src/vins_code/backend/edge_prior.cc
    # src/vins_code/backend/loss_function.cc
    # src/vins_code/factor/imu_integration.cc
)

target_link_libraries(vins_system_lib
    camera_model
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBS}
    -lpthread
)

add_executable( run_eurco  
    src/vins_code/app/run_eurco.cc
)
target_link_libraries( run_eurco 
    vins_system_lib
)

