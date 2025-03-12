/**
 * imu预积分类
 */

#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "utility.hh"

namespace vslam::vins {

class IntegrationBase {
   public:
    using Mat1515 = Eigen::Matrix<double, 15, 15>;

    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_linearized_ba,
                    const Eigen::Vector3d &_linearized_bg)
        : acc_0{_acc_0},
          gyr_0{_gyr_0},
          linearized_acc{_acc_0},
          linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba},
          linearized_bg{_linearized_bg},
          jacobian_{Eigen::Matrix<double, 15, 15>::Identity()},
          covariance_{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0},
          delta_p{Eigen::Vector3d::Zero()},
          delta_q{Eigen::Quaterniond::Identity()},
          delta_v{Eigen::Vector3d::Zero()}

    {
        noise_ = Eigen::Matrix<double, 18, 18>::Zero();
        noise_.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_.block<3, 3>(6, 6) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise_.block<3, 3>(9, 9) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise_.block<3, 3>(12, 12) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise_.block<3, 3>(15, 15) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }
    // 初始化IntegrationBase时传入imu的初始状态，然后每来一帧数据，使用中值积分一次
    void push_back(double _dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro) {
        dt_buff_.push_back(dt);
        acc_buff_.push_back(acc);
        gyro_buff_.push_back(gyro);
        propagate(dt, acc, gyro);
    }

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1) {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        // 存放预积分的量
        Eigen::Vector3d result_delta_p;
        Eigen::Quaterniond result_delta_q;
        Eigen::Vector3d result_delta_v;
        Eigen::Vector3d result_linearized_ba;
        Eigen::Vector3d result_linearized_bg;
        // 中值积分
        midPointIntegration(dt, acc_0, gyr_0, _acc_1, gyr_1, delta_p, delta_q, delta_v, linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v, result_linearized_ba, result_linearized_bg,
                            true);
        // 修改acc_0,acc_1
        sum_dt += dt;
        delta_p = result_delta_p;
        delta_v = result_delta_v;
        delta_q = result_delta_q;
        delta_q.normalize();

        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;

        acc_0 = acc_1;
        gyr_0 = gyr_1;
    }

    void midPointIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                             const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
                             const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
                             Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                             bool update_jacobian) {
        // 中值积分
        Eigen::Vector3d acc_w_0 = delta_q * (acc_0 - linearized_ba);
        Eigen::Vector3d gyro_mid = 0.5 * (gyr_0 + gyr_1) - linearized_bg;
        // 计算旋转
        result_delta_q =
            delta_q * Eigen::Quaterniond(1, gyro_mid(0) * dt / 2, gyro_mid(1) * dt / 2, gyro_mid(2) * dt / 2);
        Eigen::Vector3d acc_w_1 = delta_q * (acc_1 - linearized_ba);
        Eigen::Vector3d acc_mid = 0.5 * (acc_w_0 + acc_w_1);
        result_delta_p = delta_p + delta_v * dt + 0.5 * acc_mid * dt * dt;
        result_delta_v = delta_v + acc_mid * dt;
        // 短时间内认为零偏是不变的
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
        // 计算系统的jacobian和误差的传播
        if (update_jacobian) {
            // 暂时没有推导
            using namespace Eigen;
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
            R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
            R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1), a_1_x(0), 0;

            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) =
                -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(6, 3) =
                -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            // cout<<"A"<<endl<<A<<endl;

            MatrixXd V = MatrixXd::Zero(15, 18);
            V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * _dt;

            // step_jacobian = F;
            // step_V = V;
            jacobian_ = F * jacobian_;
            covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
        }
    }

    // 计算系统的残差,传入的是相机两帧的姿态
    Eigen::Matrix<double, 15, 1> evalue(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                                        const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai,
                                        const Eigen::Vector3d &Bgi, const Eigen::Vector3d &Pj,
                                        const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
                                        const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj) {
        Eigen::Matrix<double, 15, 1> residuals;
        // 根据jacobian矩阵进行零偏的更新，将零偏一阶线性化
        // pvq对零偏的雅可比矩阵
        Eigen::Matrix3d dp_dba = jacobian_.block<3, 3>(0, 9);
        Eigen::Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 12);
        Eigen::Matrix3d dq_dbg = jacobian_.block<3, 3>(3, 9);
        Eigen::Matrix3d dv_dba = jacobian_.block<3, 3>(6, 9);
        Eigen::Matrix3d dv_dbg = jacobian_.block<3, 3>(6, 12);
        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;
        // 零偏更新完之后更新正确的p,v,q增量

        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
        // 残差的计算
        residuals.block<3, 1>(0, 0) =
            Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(6, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(9, 0) = Baj - Bai;
        residuals.block<3, 1>(12, 0) = Bgj - Bgi;
        return residuals;
    }

    // 零偏更新之后的重新预积分
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian_.setIdentity();
        covariance_.setZero();
        for (int i = 0; i < static_cast<int>(dt_buff_.size()); i++) {
            propagate(dt_buff_[i], acc_buff_[i], gyro_buff_[i]);
        }
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    //    存储数据
    std::vector<double> dt_buff_;
    std::vector<Eigen::Vector3d> acc_buff_;
    std::vector<Eigen::Vector3d> gyro_buff_;
    // 15x15的jacobian
    Mat1515 jacobian_;
    // 15维度的残差
    Mat1515 residuals_;
    // 18x18的noise
    Eigen::Matrix<double, 18, 18> noise_;
    // 协方差
    Mat1515 covariance_;

    // 噪声
    double ACC_N = 0.08;
    double GYR_N = 0.004;
    double ACC_W = 0.00004;
    double GYR_W = 2.0e-6;

    Eigen::Vector3d G{0.0, 0.0, 9.8};
};
}  // namespace vslam::vins
