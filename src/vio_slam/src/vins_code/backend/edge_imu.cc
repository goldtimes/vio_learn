#include "edge_imu.hh"

namespace vslam::vins {

Eigen::Vector3d EdgeImu::gravity_ = Eigen::Vector3d(0, 0, 0);

void EdgeImu::ComputeResidual() {
    // 取得顶点的值，传递给预积分类，完成残差的构建
    VecX pose_i = verticies_[0]->Parameters();
    VecX speedbias_i = verticies_[1]->Parameters();

    VecX pose_j = verticies_[2]->Parameters();
    VecX speedbias_j = verticies_[3]->Parameters();

    Eigen::Quaterniond Qi(pose_i[6], pose_i[5], pose_i[4], pose_i[3]);
    Eigen::Vector3d Pi(pose_i[0], pose_i[1], pose_i[2]);
    Vec3 Vi = speedbias_i.head<3>();
    Vec3 ba_i = speedbias_i.segment(3, 3);
    Vec3 bg_i = speedbias_i.tail<3>();

    Eigen::Quaterniond Qj(pose_j[6], pose_j[5], pose_j[4], pose_j[3]);
    Eigen::Vector3d Pj(pose_j[0], pose_j[1], pose_j[2]);
    Vec3 Vj = speedbias_j.head<3>();
    Vec3 ba_j = speedbias_j.segment(3, 3);
    Vec3 bg_j = speedbias_j.tail<3>();
    // 残差
    residual_ = pre_integration_->evalue(Pi, Qi, Vi, ba_i, bg_i, Pj, Qj, Vj, ba_j, bg_j);
    // 信息矩阵
    SetInformation(pre_integration_->covariance_.inverse());
}

void EdgeImu::ComputeJacobians() {
    VecX pose_i = verticies_[0]->Parameters();
    VecX speedbias_i = verticies_[1]->Parameters();

    VecX pose_j = verticies_[2]->Parameters();
    VecX speedbias_j = verticies_[3]->Parameters();

    Eigen::Quaterniond Qi(pose_i[6], pose_i[5], pose_i[4], pose_i[3]);
    Eigen::Vector3d Pi(pose_i[0], pose_i[1], pose_i[2]);
    Vec3 Vi = speedbias_i.head<3>();
    Vec3 ba_i = speedbias_i.segment(3, 3);
    Vec3 bg_i = speedbias_i.tail<3>();

    Eigen::Quaterniond Qj(pose_j[6], pose_j[5], pose_j[4], pose_j[3]);
    Eigen::Vector3d Pj(pose_j[0], pose_j[1], pose_j[2]);
    Vec3 Vj = speedbias_j.head<3>();
    Vec3 ba_j = speedbias_j.segment(3, 3);
    Vec3 bg_j = speedbias_j.tail<3>();

    double sum_dt = pre_integration_->dt;
    dp_dba_ = pre_integration_->jacobian_.template block<3, 3>(0, 9);
    dp_dbg_ = pre_integration_->jacobian_.template block<3, 3>(0, 12);
    dr_dbg_ = pre_integration_->jacobian_.template block<3, 3>(3, 12);
    dv_dba_ = pre_integration_->jacobian_.template block<3, 3>(6, 9);
    dv_dbg_ = pre_integration_->jacobian_.template block<3, 3>(6, 12);
    // 雅可比中的最大系数和最小系数
    if (pre_integration_->jacobian_.maxCoeff() > 1e8 || pre_integration_->jacobian_.minCoeff() < 1e8) {
    }

    //    if (jacobians[1])
    {
        Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_i;
        jacobian_speedbias_i.setZero();
        jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba_;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg_;

#if 0
        jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dr_dbg_;
#else
        // Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi -
        // pre_integration->linearized_bg)); jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
        // -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
        jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
            -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q).bottomRightCorner<3, 3>() * dr_dbg_;
#endif

        jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
        jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba_;
        jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg_;

        jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

        //        jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
        jacobians_[1] = jacobian_speedbias_i;
    }
    //    if (jacobians[2])
    {
        Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_j;
        jacobian_pose_j.setZero();

        jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();
#if 0
        jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q * Utility::deltaQ(dr_dbg_ * (bg_i - pre_integration_->linearized_bg));
        jacobian_pose_j.block<3, 3>(O_R, O_R) =
            Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

        //        jacobian_pose_j = sqrt_info * jacobian_pose_j;
        jacobians_[2] = jacobian_pose_j;
    }
    //    if (jacobians[3])
    {
        Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_j;
        jacobian_speedbias_j.setZero();

        jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

        jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

        jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

        //        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
        jacobians_[3] = jacobian_speedbias_j;
    }
}

}  // namespace vslam::vins