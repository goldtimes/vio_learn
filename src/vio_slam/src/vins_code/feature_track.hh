#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "system_config.hh"

namespace vslam::vins {
class FeatureTrack {
   public:
    FeatureTrack(const SystemConfg& config);
    ~FeatureTrack();

    // 读取相机内参
    void ReadIntrinsicParameter(const std::string& config_file);

    void trackImage(const cv::Mat& img, double img_time, bool track_this_frame);
    // 设置mask
    void setMask();
    // 去畸变
    void undistoredPoints();
    // outlier的滤除
    void rejectWithF();
    // 处理新的特征点
    void addPoints();
    template <typename T>
    void reduceVector(T datas, std::vector<uchar> status);

    bool updateID(unsigned int i) {
        if (i < ids.size()) {
            if (ids[i] == -1) {
                ids[i] = n_id++;
            }
            return true;
        } else {
            return false;
        }
    }

    //    相机模型
    camodocal::CameraPtr camera_ptr;

    double cur_time;
    double prev_time;

    // 掩码 一般是鱼眼相机
    cv::Mat mask;
    // 前一帧的图像
    cv::Mat prev_img;
    // 这一帧的图像
    cv::Mat cur_img;
    cv::Mat forw_img;
    // 上一帧的关键帧
    std::vector<cv::Point2f> prev_kps;
    // 这一帧的关键帧
    std::vector<cv::Point2f> cur_kps;
    std::vector<cv::Point2f> forw_kps;

    // 去畸变的点
    std::vector<cv::Point2f> pre_un_kps;
    std::vector<cv::Point2f> cur_un_kps;

    std::map<int, cv::Point2f> cur_un_kps_map;
    std::map<int, cv::Point2f> pre_un_kps_map;
    // 跟踪的id
    std::vector<int> ids;
    // 特征点的跟踪次数
    std::vector<int> track_cnt;

    std::vector<cv::Point2f> kpts;

    // 特征点的速度
    std::vector<cv::Point2f> pts_velocity;

    // 全局的feature id
    static int n_id;

    SystemConfg config_;

   private:
    bool inBorder(const cv::Point2f& point);
};
}  // namespace vslam::vins