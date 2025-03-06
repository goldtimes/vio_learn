/**
    参见视觉slam14讲中的BA问题描述和重投影误差对位姿和特征点的雅可比矩阵
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <random>
#include <vector>

// 存放相机在世界坐标系下的pose
struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        Rwc = R;
        twc = t;
        Qwc = Eigen::Quaterniond(R);
    }

    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond Qwc;
    Eigen::Vector3d twc;
};

int main(int argc, char** argv) {
    // 我们生成20个特征点和10个相机位姿
    int featureNums = 20;
    int poseNums = 10;
    // Hessian的矩阵维度 10 * 6 + 20 * 3 = 120
    int dim = poseNums * 6 + featureNums * 3;
    Eigen::MatrixXd H(dim, dim);
    H.setZero();
    // 焦距
    int fx = 1;
    int fy = 1;

    double radius = 8;
    std::vector<Pose> camera_pose;

    for (int i = 0; i < poseNums; ++i) {
        // 10个相机位姿，相机只在1/4上的圆弧运动
        double theta = i * 2 * M_PI / (poseNums * 4);  // 1/4 圆弧
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.emplace_back(R, t);
    }
    // 三维点的生成
    std::default_random_engine generator;

    std::vector<Eigen::Vector3d> points_in_world;

    for (int j = 0; j < featureNums; ++j) {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(8., 10.);
        double x_world = xy_rand(generator);
        double y_world = xy_rand(generator);
        double z_world = z_rand(generator);
        Eigen::Vector3d Pw(x_world, y_world, z_world);
        points_in_world.push_back(Pw);
        // 开始构造H矩阵
        for (int i = 0; i < poseNums; ++i) {
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z * z;

            // 误差项对Pc的雅可比矩阵
            Eigen::Matrix<double, 2, 3> jacobian_uv_Pc;
            // clang-format off
            jacobian_uv_Pc << fx / z, 0, -fx * x / z_2,
                              0, fy / z, -fy * y / z_2;
            // clang-format on

            // 对三维点的雅可比矩阵的 jacobian_uv_Pc * R
            Eigen::Matrix<double, 2, 3> jacobian_Pwj = jacobian_uv_Pc * Rcw;

            // 对位姿的雅可比矩阵2x6
            Eigen::Matrix<double, 2, 6> jacobian_Ti;

            // clang-format off
            jacobian_Ti << -x * y * fx / z_2, (1 + x * x / z_2) * fx, -y / z * fx, fx / z, 0, -x * fx / z_2,
                           -(1 + y * y / z_2) * fy, x * y / z_2 * fy, x / z * fy, 0, fy / z, -y * fy / z_2;

            // clang-format on

            // /** hessian矩阵的形式
            //     x1 x2 x3 .... x10, f1, f2, .....f20
            //   x1
            //   x2
            //   x3
            //   .
            //   .
            //   .
            //   x10
            //   f1
            //   f2
            //   f3
            //   .
            //   .
            //   .
            //   f20

            //  */
            //  位姿jacobian * 位姿 jacobian
            H.block(i * 6, i * 6, 6, 6) += jacobian_Ti.transpose() * jacobian_Ti;
            // 特征点对特征点
            H.block(j * 3 + 6 * poseNums, j * 3 + 6 * poseNums, 3, 3) += jacobian_Pwj.transpose() * jacobian_Pwj;
            H.block(i * 6, j * 3 + 6 * poseNums, 6, 3) += jacobian_Ti.transpose() * jacobian_Pwj;
            H.block(j * 3 + 6 * poseNums, i * 6, 3, 6) += jacobian_Pwj.transpose() * jacobian_Ti;
        }
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << svd.singularValues() << std::endl;

    return 0;
}