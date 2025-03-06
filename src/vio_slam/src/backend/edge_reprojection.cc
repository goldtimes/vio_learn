#include "edge_reprojection.hh"
#include "sophus/se3.h"
#include "vertex_inverse_depth.hh"
#include "vertex_point_xyz.hh"

namespace vslam::backend {

void EdgeReprojection::ComputeResidual() {
}

void EdgeReprojection::ComputeJacobians() {
}

void EdgeReprojection::SetTranslationImuFromCamera(Eigen::Quaterniond &qic, Vec3 &tic) {
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