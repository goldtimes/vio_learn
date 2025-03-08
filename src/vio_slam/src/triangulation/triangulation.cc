/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-08 16:12:05
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-08 16:47:33
 * @FilePath: /vio_learn/src/vio_slam/src/triangulation/triangulation.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
   三角化的代码
*/

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <random>
#include <vector>

struct Frame {
    // 关键帧的位姿和观察到的像素坐标
    Frame(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {
    }

    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;
};

int main(int argc, char** argv) {
    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Frame> keyframes;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4);  // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        keyframes.push_back(Frame(R, t));
    }
    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    std::normal_distribution<double> noise_pdf(0, 1.);

    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);
    Eigen::Vector3d Pw(tx, ty, tz);
    // 假设关键点被第3帧才开始观察到
    int start_frame_id = 3;
    int end_frame_id = poseNums;

    for (int i = start_frame_id; i < poseNums; ++i) {
        // 计算landmark到归一化相机坐标
        Eigen::Vector3d Pc = keyframes[i].Rwc.transpose() * (Pw - keyframes[i].twc);
        double depth = Pc[2];
        Pc = Pc / depth;
        keyframes[i].uv = Pc.head<2>();
    }

    // 开始三角化
    Eigen::Vector3d P_est = Eigen::Vector3d::Zero();
    // 被观测到的帧数
    int num_observer = end_frame_id - start_frame_id;
    // 方程的维度
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2 * num_observer, 4);

    for (int i = 0; i < num_observer; ++i) {
        int idx = i + start_frame_id;
        // 有噪声之后 三角化的误差很大
        // double u = keyframes[idx].uv[0] + noise_pdf(generator);
        // double v = keyframes[idx].uv[1] + noise_pdf(generator);
        double u = keyframes[idx].uv[0];
        double v = keyframes[idx].uv[1];

        Eigen::MatrixXd Tcw(3, 4);
        Tcw.block(0, 0, 3, 3) = keyframes[idx].Rwc.transpose();
        Tcw.block(0, 3, 3, 1) = -keyframes[idx].Rwc.transpose() * keyframes[idx].twc;

        D.row(2 * i + 0) = u * Tcw.row(2) - Tcw.row(0);
        D.row(2 * i + 1) = v * Tcw.row(2) - Tcw.row(1);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix4d mv = svd.matrixV();
    Eigen::Vector4d v_last = mv.col(3);
    if (v_last[3] < 1e-8) {
        v_last[3] = 0.00001;
    }
    P_est = v_last.hnormalized();
    std::cout << "ground truth: \n" << Pw.transpose() << std::endl;
    std::cout << "your result: \n" << P_est.transpose() << std::endl;

    return 0;
}