#include "system_config.hh"
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace vslam::vins {
using namespace std;

void readParameters(std::string config_file, SystemConfg& config) {
    // 借助cv的方式读取文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "1 readParameters ERROR: Wrong path to settings:" << config_file << endl;
        return;
    }
    fsSettings["imu_topic"] >> config.imu_topic;

    config.focal_length = 460;
    // 后端的求解
    config.SOLVER_TIME = fsSettings["max_solver_time"];
    config.NUM_ITERATIONS = fsSettings["max_num_iterations"];
    // 关键帧的判断
    config.MIN_PARALLAX = fsSettings["keyframe_parallax"];
    config.MIN_PARALLAX = config.MIN_PARALLAX / config.focal_length;

    string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    config.VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.txt";
    // cout << "result path " << VINS_RESULT_PATH << endl;
    // ofstream fout(VINS_RESULT_PATH, ios::out);
    // fout.close();
    // imu噪声
    config.ACC_N = fsSettings["acc_n"];
    config.ACC_W = fsSettings["acc_w"];
    config.GYR_N = fsSettings["gyr_n"];
    config.GYR_W = fsSettings["gyr_w"];
    config.G.z() = fsSettings["g_norm"];
    // 图像参数
    config.ROW = fsSettings["image_height"];
    config.COL = fsSettings["image_width"];
    // ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    config.ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (config.ESTIMATE_EXTRINSIC == 2) {
        // ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        config.RIC.push_back(Eigen::Matrix3d::Identity());
        config.TIC.push_back(Eigen::Vector3d::Zero());
        config.EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    } else {
        if (config.ESTIMATE_EXTRINSIC == 1) {
            // ROS_WARN(" Optimize extrinsic param around initial guess!");
            config.EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (config.ESTIMATE_EXTRINSIC == 0) {
            cout << " fix extrinsic param " << endl;
        }
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        config.RIC.push_back(eigen_R);
        config.TIC.push_back(eigen_T);
        // ROS_INFO_STREAM("Extrinsic_R : " << endl
        //                                  << RIC[0]);
        // ROS_INFO_STREAM("Extrinsic_T : " << endl
        //                                  << TIC[0].transpose());
    }

    config.INIT_DEPTH = 5.0;
    config.BIAS_ACC_THRESHOLD = 0.1;
    config.BIAS_GYR_THRESHOLD = 0.1;

    config.TD = fsSettings["td"];
    config.ESTIMATE_TD = fsSettings["estimate_td"];
    // if (ESTIMATE_TD)
    // ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    // else
    //     ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    config.ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (config.ROLLING_SHUTTER) {
        config.TR = fsSettings["rolling_shutter_tr"];
        // ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    } else {
        config.TR = 0;
    }

    // string VINS_FOLDER_PATH = readParam<string>(n, "vins_folder");

    fsSettings["image_topic"] >> config.image_topic;
    fsSettings["imu_topic"] >> config.imu_topic;
    config.MAX_CNT = fsSettings["max_cnt"];
    config.MIN_DIST = fsSettings["min_dist"];
    config.ROW = fsSettings["image_height"];
    config.COL = fsSettings["image_width"];
    config.FREQ = fsSettings["freq"];
    config.F_THRESHOLD = fsSettings["F_threshold"];
    config.SHOW_TRACK = fsSettings["show_track"];
    config.EQUALIZE = fsSettings["equalize"];
    config.FISHEYE = fsSettings["fisheye"];
    // if (FISHEYE == 1)
    //     FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    config.CAM_NAMES.push_back(config_file);

    // WINDOW_SIZE = 20;
    config.STEREO_TRACK = false;
    config.PUB_THIS_FRAME = false;

    if (config.FREQ == 0) {
        config.FREQ = 10;
    }
    fsSettings.release();

    cout << "1 readParameters:  "
         << "\n  INIT_DEPTH: " << config.INIT_DEPTH << "\n  MIN_PARALLAX: " << config.MIN_PARALLAX
         << "\n  ACC_N: " << config.ACC_N << "\n  ACC_W: " << config.ACC_W << "\n  GYR_N: " << config.GYR_N
         << "\n  GYR_W: " << config.GYR_W << "\n  RIC:   " << config.RIC[0]
         << "\n  TIC:   " << config.TIC[0].transpose() << "\n  G:     " << config.G.transpose()
         << "\n  BIAS_ACC_THRESHOLD:" << config.BIAS_ACC_THRESHOLD
         << "\n  BIAS_GYR_THRESHOLD:" << config.BIAS_GYR_THRESHOLD << "\n  SOLVER_TIME:" << config.SOLVER_TIME
         << "\n  NUM_ITERATIONS:" << config.NUM_ITERATIONS << "\n  ESTIMATE_EXTRINSIC:" << config.ESTIMATE_EXTRINSIC
         << "\n  ESTIMATE_TD:" << config.ESTIMATE_TD << "\n  ROLLING_SHUTTER:" << config.ROLLING_SHUTTER
         << "\n  ROW:" << config.ROW << "\n  COL:" << config.COL << "\n  TD:" << config.TD << "\n  TR:" << config.TR
         << "\n  FOCAL_LENGTH:" << config.focal_length << "\n  IMAGE_TOPIC:" << config.image_topic
         << "\n  IMU_TOPIC:" << config.imu_topic << "\n  FISHEYE_MASK:" << config.FISHEYE_MASK
         << "\n  CAM_NAMES[0]:" << config.CAM_NAMES[0] << "\n  MAX_CNT:" << config.MAX_CNT
         << "\n  MIN_DIST:" << config.MIN_DIST << "\n  FREQ:" << config.FREQ << "\n  F_THRESHOLD:" << config.F_THRESHOLD
         << "\n  SHOW_TRACK:" << config.SHOW_TRACK << "\n  STEREO_TRACK:" << config.STEREO_TRACK
         << "\n  EQUALIZE:" << config.EQUALIZE << "\n  FISHEYE:" << config.FISHEYE
         << "\n  PUB_THIS_FRAME:" << config.PUB_THIS_FRAME << endl;
}
}  // namespace vslam::vins