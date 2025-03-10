#pragma once
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <condition_variable>
#include <iostream>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include "feature_track.hh"
#include "system_config.hh"
#include "tic_toc.hh"

namespace vslam::vins {

// 前置声明

struct ImuMsg {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamped;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
using ImuConstPtr = std::shared_ptr<ImuMsg const>;

struct ImageMsg {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamped;
    // 相机坐标的特征点
    std::vector<Eigen::Vector3d> points;
    // 相机特征点的ids
    std::vector<int> id_of_points;
    // 归一化的像素坐标
    std::vector<float> u_of_points;
    std::vector<float> v_of_points;
    std::vector<float> velocity_x_of_point;
    std::vector<float> velocity_y_of_point;
};
using ImageConstPtr = std::shared_ptr<ImageMsg const>;

class System {
   public:
    System(const std::string config_path);
    ~System();

    void AddImage(double sensor_time, const cv::Mat& image);

    void AddImu(double sensor_time, const Eigen::Vector3d& imu_acc, const Eigen::Vector3d& imu_gyro);

    // 计算里程计
    void ProcessBackend();

    // 绘制轨迹
    void Draw();

   private:
    std::vector<std::pair<std::vector<ImuConstPtr>, ImageConstPtr>> getMeasurements();

   private:
    SystemConfg config_;

    // 我们将数据放到队列中，并通知获取后端检查数据是否同步了，获取到测量数据，后端开始运行
    std::mutex data_mtx;
    std::condition_variable cond;

    std::queue<ImuConstPtr> imu_queue_;
    std::queue<ImageConstPtr> image_queu_;

    double last_imu_time;

    bool init_feature = false;
    bool first_image_flag = true;

    double first_image_time;
    double last_image_time;

    int pub_count = 0;
    bool PUB_THIS_FRAME = false;
    TicToc track_record;

    std::shared_ptr<FeatureTrack> feature_track;
};
}  // namespace vslam::vins