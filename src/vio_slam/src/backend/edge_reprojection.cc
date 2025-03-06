/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-06 23:55:49
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-07 00:59:14
 * @FilePath: /vio_learn/src/vio_slam/src/backend/edge_reprojection.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "edge_reprojection.hh"
#include "sophus/se3.h"
#include "vertex_inverse_depth.hh"
#include "vertex_point_xyz.hh"

namespace vslam::backend {

// 查看第三章的pdf，逆深度的视觉重投影残差
void EdgeReprojection::ComputeResidual() {
    // 将i时刻的特征点通过姿态投影到j时刻，然后构建残差
    // i 时刻的逆深度
    double inv_dep_i = verticies_[0]->Parameters()[0];
    VecX pose_vec_i = verticies_[1]->Parameters();
    VecX pose_vec_j = verticies_[2]->Parameters();

    Eigen::Quaterniond Qwi(pose_vec_i[6], pose_vec_i[3], pose_vec_i[4], pose_vec_i[5]);
    Eigen::Vector3d twi(pose_vec_i[0], pose_vec_i[1], pose_vec_i[2]);

    Eigen::Quaterniond Qwj(pose_vec_j[6], pose_vec_j[3], pose_vec_j[4], pose_vec_j[5]);
    Eigen::Vector3d twj(pose_vec_j[0], pose_vec_j[1], pose_vec_j[2]);

    // 归一化的相机坐标到相机坐标 / inv_dep_i
    Eigen::Vector3d pt_in_camera_i = pts_i_ / inv_dep_i;
    Eigen::Vector3d pt_in_imu_i = qic_ * pt_in_camera_i + tic_;
    Eigen::Vector3d pt_in_world = Qwi * pt_in_imu_i + twi;
    Eigen::Vector3d pt_in_imu_j = Qwj.inverse() * (pt_in_world - twj);
    Eigen::Vector3d pt_in_camera_j = qic_.inverse() * (pt_in_imu_j - tic_);

    double depth_j = pt_in_camera_j[2];
    // 归一化相机
    residual_ = (pt_in_camera_j / depth_j).head<2>() - pts_j_.head<2>();
}

void EdgeReprojection::ComputeJacobians() {
    // 将i时刻的特征点通过姿态投影到j时刻，然后构建残差
    // i 时刻的逆深度
    double inv_dep_i = verticies_[0]->Parameters()[0];
    VecX pose_vec_i = verticies_[1]->Parameters();
    VecX pose_vec_j = verticies_[2]->Parameters();

    Eigen::Quaterniond Qwi(pose_vec_i[6], pose_vec_i[3], pose_vec_i[4], pose_vec_i[5]);
    Eigen::Vector3d twi(pose_vec_i[0], pose_vec_i[1], pose_vec_i[2]);

    Eigen::Quaterniond Qwj(pose_vec_j[6], pose_vec_j[3], pose_vec_j[4], pose_vec_j[5]);
    Eigen::Vector3d twj(pose_vec_j[0], pose_vec_j[1], pose_vec_j[2]);

    // 归一化的相机坐标到相机坐标 / inv_dep_i
    Eigen::Vector3d pt_in_camera_i = pts_i_ / inv_dep_i;
    Eigen::Vector3d pt_in_imu_i = qic_ * pt_in_camera_i + tic_;
    Eigen::Vector3d pt_in_world = Qwi * pt_in_imu_i + twi;
    Eigen::Vector3d pt_in_imu_j = Qwj.inverse() * (pt_in_world - twj);
    Eigen::Vector3d pt_in_camera_j = qic_.inverse() * (pt_in_imu_j - tic_);

    double depth_j = pt_in_camera_j[2];

    Mat33 Ri = Qwi.toRotationMatrix();
    Mat33 Rj = Qwj.toRotationMatrix();
    Mat33 ric = qic_.toRotationMatrix();

    // 先计算残差对fcj的雅可比
    Mat23 reduce(2, 3);
    reduce << 1. / depth_j, 0, -pt_in_camera_j(0) / (depth_j * depth_j), 0, 1. / depth_j,
        -pt_in_camera_j(1) / (depth_j * depth_j);
    // fcj对i时刻姿态的雅可比矩阵
    Eigen::Matrix<double, 2, 6> jacobian_pose_i;
    Eigen::Matrix<double, 3, 6> jaco_i;
    jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
    jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Sophus::SO3::hat(pt_in_imu_i);
    jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
    // fcj对i时刻姿态的雅可比矩阵

    Eigen::Matrix<double, 2, 6> jacobian_pose_j;
    Eigen::Matrix<double, 3, 6> jaco_j;
    jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
    jaco_j.rightCols<3>() = ric.transpose() * Sophus::SO3::hat(pt_in_imu_j);
    jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
    // fcj对逆深度的雅可比矩阵
    Eigen::Vector2d jacobian_feature;
    jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);

    jacobians_[0] = jacobian_feature;
    jacobians_[1] = jacobian_pose_i;
    jacobians_[2] = jacobian_pose_j;
}

void EdgeReprojection::SetTranslationImuFromCamera(Eigen::Quaterniond &qic, Vec3 &tic) {
    qic_ = qic;
    tic_ = tic;
}

void EdgeReprojectionXYZ::ComputeResidual() {
    // pt_w世界坐标下的点转换到imu坐标系在转到camera坐标
    Vec3 pt_w = verticies_[0]->Parameters();
    VecX pose_params = verticies_[1]->Parameters();
    // 所以这里顶点存放的就是imu到world的姿态
    Eigen::Quaterniond qwi(pose_params[6], pose_params[3], pose_params[4], pose_params[5]);
    Eigen::Vector3d twi(pose_params.head<3>());

    // 现将pt_w转到imu坐标系
    Vec3 pt_i = qwi.inverse() * (pt_w - twi);
    // 转到camera坐标
    Vec3 pt_c = qic_.inverse() * (pt_i - tic_);
    // 然后归一化深度
    double dep_i = pt_c.z();
    residual_ = (pt_c / dep_i).head<2>() - observation_.head<2>();
}

void EdgeReprojectionXYZ::ComputeJacobians() {
    // pt_w世界坐标下的点转换到imu坐标系在转到camera坐标
    Vec3 pt_w = verticies_[0]->Parameters();
    VecX pose_params = verticies_[1]->Parameters();
    // 所以这里顶点存放的就是imu到world的姿态
    Eigen::Quaterniond qwi(pose_params[6], pose_params[3], pose_params[4], pose_params[5]);
    Eigen::Vector3d twi(pose_params.head<3>());

    // 现将pt_w转到imu坐标系
    Vec3 pt_i = qwi.inverse() * (pt_w - twi);
    // 转到camera坐标
    Vec3 pt_c = qic_.inverse() * (pt_i - tic_);
    // 然后归一化深度
    double dep_i = pt_c.z();

    Mat33 Rwi = qwi.toRotationMatrix();
    Mat33 Ric = qic_.toRotationMatrix();

    // 残差对姿态微小量的雅可比矩阵，链式求导法
    Mat23 reduce;
    // clang-format off
    reduce << 1.0 / dep_i, 0 , -pt_c(0) / (dep_i * dep_i),
              0, 1.0 / dep_i , - pt_c(1) / (dep_i * dep_i);
    // clang-format on
    // 对位姿的雅可比矩阵
    Eigen::Matrix<double, 2, 6> jacobian_pose_i;
    Eigen::Matrix<double, 3, 6> jacobian_i;
    // 对Pi的雅可比
    jacobian_i.leftCols<3>() = -Ric.transpose() * Rwi.transpose();
    // 对Qwi的雅可比
    jacobian_i.rightCols<3>() = Ric.transpose() * Sophus::SO3::hat(pt_i);
    jacobian_pose_i.leftCols<6>() = reduce * jacobian_i;
    // 残差对路标点的雅可比矩阵 链式求导法
    Eigen::Matrix<double, 2, 3> jacobian_feature;
    jacobian_feature = reduce * Ric.transpose() * Rwi.transpose();
    // 注意顺序
    jacobians_[0] = jacobian_feature;
    jacobians_[1] = jacobian_pose_i;
}

void EdgeReprojectionXYZ::SetTranslationImuFromCamera(Eigen::Quaterniond &qic, Vec3 &tic) {
    qic = qic_;
    tic = tic_;
}

void EdgeReprojectionPoseOnly::ComputeResidual() {
    // 将landmark投影到像素平面
    auto pose_vertex = verticies_[0];
    VecX pose_vec = pose_vertex->Parameters();
    Sophus::SE3 pose(Eigen::Quaterniond(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5]),
                     Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[2]));
    Vec3 pc = pose.inverse() * landmark_world_;  // 相机坐标系
    // 归一化
    pc = pc / pc[2];
    // 乘上内参系数
    Vec2 pro_pixel = (K_ * pc);
    // 和观测像素做残差
    residual_ = pro_pixel - observation_;
}

void EdgeReprojectionPoseOnly::ComputeJacobians() {
}

}  // namespace vslam::backend