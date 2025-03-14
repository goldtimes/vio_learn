#pragma once
#include <Eigen/Eigen>
#include <list>
#include <vector>
namespace vslam::vins {

class FeaturePerFrame {
   public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& _point, double td) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
        cur_td = td;
    }

    double cur_td;
    Eigen::Vector3d point;
    Eigen::Vector2d uv;
    Eigen::Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    double dep_gradient;
};

class FeaturePerId {
   public:
    // 特征点的唯一id
    const int feature_id;
    // 被观测的起始帧数
    int start_frame;
    // 特征点是否被估计了深度
    double estimated_depth;
    // 每一帧中的像素值
    std::vector<FeaturePerFrame> feature_per_frame;
    FeaturePerId(const int id, int s_frame) : feature_id(id), start_frame(s_frame), estimated_depth(0.0) {
    }

    int endFrame();
};

/**
    特征点管理类
    维护一个列表，存储了有多少个feature点。因为每个feature都有自己id,相同id的feature再滑动窗口内被追踪到的帧数
 */
class FeatureManager {
   public:
    std::list<FeaturePerId> feature;

   private:
};
}  // namespace vslam::vins