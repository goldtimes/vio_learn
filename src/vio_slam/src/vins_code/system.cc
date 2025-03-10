#include "system.hh"
#include "tic_toc.hh"

namespace vslam::vins {

System::System(const std::string config_dir) {
    std::cout << "vins system staring............." << std::endl;
    std::string config_file = config_dir + "/euroc_config.yaml";
    readParameters(config_file, config_);

    // tracker

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
        first_image_flag = sensor_time;
        last_image_time = sensor_time;
        return;
    }
    // 如果两帧间隔太久
    if (sensor_time - last_image_time > 1.0) {
        std::cerr << "image data is discontinue. reset the feature track" << std::endl;
        first_image_flag = true;
        first_image_flag = sensor_time;
        last_image_time = sensor_time;
        pub_count = 0;

        return;
    }

    // 如果检测到时间倒回
    if (sensor_time < last_image_time) {
        std::cerr << "image data is disorder, reset the feature track" << std::endl;
        first_image_flag = true;
        first_image_flag = sensor_time;
        last_image_time = sensor_time;
        pub_count = 0;

        return;
    }

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
    // update feature id;

    if (PUB_THIS_FRAME) {
        pub_count++;
        // 将提取到的特征放到image_msg队列中
    }
    auto timeused = track_record.toc();
    std::cout << "track image  used time:" << timeused << " ms" << std::endl;
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