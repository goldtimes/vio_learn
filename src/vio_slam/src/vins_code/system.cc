#include "system.hh"
#include <set>
#include "tic_toc.hh"

namespace vslam::vins {

System::System(const std::string config_dir) {
    std::cout << "vins system staring............." << std::endl;
    std::string config_file = config_dir + "/euroc_config.yaml";
    readParameters(config_file, config_);

    // tracker
    feature_track = std::make_shared<FeatureTrack>(config_);
    // estimator

    // record pose
}
System::~System() {
}

void System::AddImage(double sensor_time, const cv::Mat& image) {
    // 第一帧的时候丢弃
    if (!init_feature) {
        std::cout << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed"
                  << std::endl;
        init_feature = true;
        return;
    }
    if (first_image_flag) {
        std::cout << "first image.........." << std::endl;
        first_image_flag = false;
        last_image_time = sensor_time;
        return;
    }
    // 如果两帧间隔太久
    if (sensor_time - last_image_time > 1.0) {
        std::cerr << "image data is discontinue. reset the feature track" << std::endl;
        first_image_flag = true;
        last_image_time = sensor_time;
        pub_count = 0;

        return;
    }

    // 如果检测到时间倒回
    if (sensor_time < last_image_time) {
        std::cerr << "image data is disorder, reset the feature track" << std::endl;
        first_image_flag = true;
        last_image_time = sensor_time;
        pub_count = 0;
        return;
    }
    last_image_time = sensor_time;
    // 对特征的追踪是每帧都需要做的.但是我们不需要每帧的特征点都放到后端去。所以参数里面的频率是10HZ
    if (std::round(1.0 * pub_count / (sensor_time - first_image_time)) <= config_.FREQ) {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (sensor_time - first_image_time) - config_.FREQ) < 0.01 * config_.FREQ) {
            std::cout << "reset frequency control" << std::endl;
            first_image_time = sensor_time;
            pub_count = 0;
        }
    } else {
        PUB_THIS_FRAME = false;
    }

    track_record.tic();
    // track image
    feature_track->trackImage(image, sensor_time, PUB_THIS_FRAME);
    // update feature id;

    if (PUB_THIS_FRAME) {
        pub_count++;
        // 将提取到的特征放到image_msg队列中
        std::shared_ptr<ImageMsg> feature_points(new ImageMsg);
        feature_points->timestamped = sensor_time;
        std::set<int> has_ids;
        auto& undistored_pts = feature_track->cur_un_kps;
        auto& cur_pts = feature_track->cur_kps;
        auto& ids = feature_track->ids;
        auto pts_velocity = feature_track->pts_velocity;

        for (int i = 0; i < ids.size(); ++i) {
            if (feature_track->track_cnt[i] > 1) {
                int p_id = ids[i];
                has_ids.insert(p_id);
                double x = undistored_pts[i].x;
                double y = undistored_pts[i].y;
                double z = 1.;
                feature_points->points.push_back(Eigen::Vector3d(x, y, z));
                feature_points->id_of_points.push_back(p_id * config_.NUM_OF_CAM + i);
                feature_points->u_of_points.push_back(cur_pts[i].x);
                feature_points->v_of_points.push_back(cur_pts[i].y);
                feature_points->velocity_x_of_point.push_back(pts_velocity[i].x);
                feature_points->velocity_y_of_point.push_back(pts_velocity[i].y);
            }
        }

        data_mtx.lock();
        image_queu_.push(feature_points);
        data_mtx.unlock();
        cond.notify_one();
    }
    auto timeused = track_record.toc();
    std::cout << "track image  used time:" << timeused << " ms" << std::endl;
    cv::Mat show_img;
    cv::cvtColor(image, show_img, CV_GRAY2RGB);
    if (config_.SHOW_TRACK) {
        for (unsigned int j = 0; j < feature_track->cur_kps.size(); j++) {
            double len = std::min(1.0, 1.0 * feature_track->track_cnt[j] / config_.WINDOW_SIZE);
            cv::circle(show_img, feature_track->cur_kps[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }

        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
        cv::imshow("IMAGE", show_img);
        cv::waitKey(1);
    }
}

void System::AddImu(double sensor_time, const Eigen::Vector3d& imu_acc, const Eigen::Vector3d& imu_gyro) {
    std::shared_ptr<ImuMsg> imu_data(new ImuMsg());
    imu_data->timestamped = sensor_time;
    imu_data->acc = imu_acc;
    imu_data->gyro = imu_gyro;
    if (sensor_time <= last_imu_time) {
        std::cout << "imu data in disorder" << std::endl;
        return;
    }
    last_imu_time = sensor_time;
    {
        std::lock_guard<std::mutex> lk(data_mtx);
        imu_queue_.push(imu_data);
    }
    // 释放锁并通知其他线程
    cond.notify_one();
}

void System::Draw() {
}

void System::ProcessBackend() {
    std::cout << "ProcessBackend Start" << std::endl;
    while (true) {
        // 获取锁
        std::unique_lock<std::mutex> lk(data_mtx);
        // img1-img2 内的imu数据
        std::vector<std::pair<std::vector<ImuConstPtr>, ImageConstPtr>> measurements;
        // 被唤醒后，检查条件变量是否为true/false. false的情况下会放弃锁资源，继续等待
        // true的条件下，获取锁，往下执行

        cond.wait(lk, [&]() {
            measurements = getMeasurements();
            return !measurements.empty();
        });
    }
}

std::vector<std::pair<std::vector<ImuConstPtr>, ImageConstPtr>> System::getMeasurements() {
    std::vector<std::pair<std::vector<ImuConstPtr>, ImageConstPtr>> measurements;
    return measurements;
}
}  // namespace vslam::vins