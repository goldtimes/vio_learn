#include <iostream>
#include <random>
#include "edge_reprojection.hh"
#include "problem_schur.hh"
#include "vertex_inverse_depth.hh"
#include "vertex_pose.hh"

struct Frame {
    Frame(const Eigen::Matrix3d& R, const Eigen::Vector3d t) : Rwc(R), Qwc(R), twc(t) {
    }
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond Qwc;
    Eigen::Vector3d twc;

    std::unordered_map<int, Eigen::Vector3d> features;
};

void GenerateSimData(std::vector<Frame>& camera_poses, std::vector<Eigen::Vector3d>& features) {
    int featureNums = 20;
    int poseNums = 3;
    double radius = 8;
    // 相机位姿
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4);  // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_poses.push_back(Frame(R, t));
    }
    // 特征点
    std::default_random_engine engine;
    std::normal_distribution<double> noise_pdf(0., 1. / 1000.);  // 2pixel / focal
    for (int i = 0; i < featureNums; ++i) {
        std::uniform_real_distribution<double> xy_rand(-4, 4);
        std::uniform_real_distribution<double> z_rand(4, 8);
        Eigen::Vector3d Pw(xy_rand(engine), xy_rand(engine), z_rand(engine));
        features.push_back(Pw);
        // 投影每个相机平面
        for (int j = 0; j < poseNums; ++j) {
            Eigen::Vector3d Pc = camera_poses[j].Rwc.transpose() * (Pw - camera_poses[j].twc);
            // 归一化坐标
            Pc = Pc / Pc[2];
            // 加上噪声
            Pc[0] = Pc[0] + noise_pdf(engine);
            Pc[1] = Pc[1] + noise_pdf(engine);
            camera_poses[j].features.insert(std::make_pair(i, Pc));
        }
    }
}

int main(int argc, char** argv) {
    using namespace vslam::backend;
    std::vector<Frame> cameraPoses;
    std::vector<Eigen::Vector3d> points_3d;
    GenerateSimData(cameraPoses, points_3d);
    // 相机到imu的外参
    Eigen::Quaterniond qic(1, 0, 0, 0);
    Eigen::Vector3d tic(0, 0, 0);

    vslam::backend::ProblemSchur problem(vslam::backend::ProblemSchur::ProblemType::SLAM_PROBLEM);
    std::vector<std::shared_ptr<VertexPose>> pose_vertices_;
    // 构建顶点
    for (int i = 0; i < cameraPoses.size(); ++i) {
        std::shared_ptr<VertexPose> camera_pose(new VertexPose());
        Eigen::VectorXd camera_params(7);
        camera_params << cameraPoses[i].twc, cameraPoses[i].Qwc.x(), cameraPoses[i].Qwc.y(), cameraPoses[i].Qwc.z(),
            cameraPoses[i].Qwc.w();
        camera_pose->SetParameters(camera_params);
        problem.AddVertex(camera_pose);
        pose_vertices_.push_back(camera_pose);
    }
    // 逆深度顶点
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0, 1.);
    double noise = 0;
    std::vector<double> noise_invd;
    std::vector<std::shared_ptr<VertexInverseDepth>> allPoints;
    for (int i = 0; i < points_3d.size(); ++i) {
        //假设所有特征点的起始帧为第0帧， 逆深度容易得到
        Eigen::Vector3d Pw = points_3d[i];
        // 在第一帧的相机坐标
        Eigen::Vector3d Pc = cameraPoses[0].Rwc.transpose() * (Pw - cameraPoses[0].twc);
        // 逆深度
        noise = noise_pdf(generator);
        double inverse_depth = 1. / (Pc.z() + noise);
        noise_invd.push_back(inverse_depth);
        // 构建landmark点
        std::shared_ptr<VertexInverseDepth> inverse_vertex(new VertexInverseDepth());
        VecX inv_d(1);
        inv_d << inverse_depth;
        inverse_vertex->SetParameters(inv_d);
        problem.AddVertex(inverse_vertex);
        allPoints.push_back(inverse_vertex);
        // 构建边
        for (int j = 1; j < cameraPoses.size(); ++j) {
            // 先取出对应的特征点
            Eigen::Vector3d pt_i = cameraPoses[0].features.find(i)->second;
            Eigen::Vector3d pt_j = cameraPoses[j].features.find(i)->second;
            // 构建边
            std::shared_ptr<EdgeReprojection> edge_rep(new EdgeReprojection(pt_i, pt_j));

            std::vector<std::shared_ptr<Vertex>> edge_vertices;
            edge_vertices.push_back(inverse_vertex);
            edge_vertices.push_back(pose_vertices_[0]);
            edge_vertices.push_back(pose_vertices_[j]);

            edge_rep->SetVertex(edge_vertices);
            problem.AddEdge(edge_rep);
        }
    }

    problem.Solve(5);

    std::cout << "\nCompare MonoBA results after opt..." << std::endl;
    for (size_t k = 0; k < allPoints.size(); k += 1) {
        std::cout << "after opt, point " << k << " : gt " << 1. / points_3d[k].z() << " ,noise " << noise_invd[k]
                  << " ,opt " << allPoints[k]->Parameters() << std::endl;
    }
    std::cout << "------------ pose translation ----------------" << std::endl;
    for (int i = 0; i < pose_vertices_.size(); ++i) {
        std::cout << "translation after opt: " << i << " :" << pose_vertices_[i]->Parameters().head(3).transpose()
                  << " || gt: " << cameraPoses[i].twc.transpose() << std::endl;
    }
    /// 优化完成后，第一帧相机的 pose 平移（x,y,z）不再是原点 0,0,0. 说明向零空间发生了漂移。
    /// 解决办法： fix 第一帧和第二帧，固定 7 自由度。 或者加上非常大的先验值。

    return 0;
}