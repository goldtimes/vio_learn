#pragma once
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <string>
#include <unordered_map>
#include <vector>

namespace vslam::vins {
struct SystemConfg {
    SystemConfg() {
    }
    const int NUM_OF_CAM = 1;
    std::string imu_topic;
    std::string image_topic;
    int focal_length;

    std::string FISHEYE_MASK;
    std::vector<std::string> CAM_NAMES;
    int MAX_CNT;
    int MIN_DIST;
    //  int WINDOW_SIZE;
    int FREQ;
    double F_THRESHOLD;
    int SHOW_TRACK;
    bool STEREO_TRACK;
    int EQUALIZE;
    int FISHEYE;
    bool PUB_THIS_FRAME;

    // estimator
    const int WINDOW_SIZE = 10;
    const int NUM_OF_F = 1000;

    double INIT_DEPTH;
    double MIN_PARALLAX;
    int ESTIMATE_EXTRINSIC;

    double ACC_N, ACC_W;
    double GYR_N, GYR_W;

    std::vector<Eigen::Matrix3d> RIC;
    std::vector<Eigen::Vector3d> TIC;
    Eigen::Vector3d G;

    double BIAS_ACC_THRESHOLD;
    double BIAS_GYR_THRESHOLD;
    double SOLVER_TIME;
    int NUM_ITERATIONS;
    std::string EX_CALIB_RESULT_PATH;
    std::string VINS_RESULT_PATH;
    std::string IMU_TOPIC;
    double TD;
    double TR;
    int ESTIMATE_TD;
    int ROLLING_SHUTTER;
    double ROW, COL;
};

void readParameters(std::string config_file, SystemConfg& config);

enum SIZE_PARAMETERIZATION { SIZE_POSE = 7, SIZE_SPEEDBIAS = 9, SIZE_FEATURE = 1 };

enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

}  // namespace vslam::vins