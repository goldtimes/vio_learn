/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-09 22:39:37
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-10 23:54:46
 * @FilePath: /vio_learn/src/vio_slam/src/vins_code/app/run_eurco.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include "system.hh"

std::string euro_data_dir;
std::string config_dir;

const int nDelayTimes = 2;

std::shared_ptr<vslam::vins::System> system_;

void PubImageThread() {
    auto cam_config_file = boost::filesystem::path(config_dir) / "MH_05_cam0.txt";
    if (boost::filesystem::exists(cam_config_file)) {
        std::cout << "PubImageThread Start reading cam file:" << cam_config_file.string() << std::endl;
        std::ifstream cam_fin;
        cam_fin.open(cam_config_file);

        std::string cam_line;
        double cam_timestamped;
        std::string image_name;

        while (std::getline(cam_fin, cam_line) && !cam_line.empty()) {
            std::istringstream ss_cam(cam_line);
            ss_cam >> cam_timestamped >> image_name;
            // std::cout << std::fixed << "image_name:" << image_name << ",image time: " << cam_timestamped <<
            // std::endl;
            auto image_path = euro_data_dir + "/cam0/data/" + image_name;
            cv::Mat image = cv::imread(image_path, 0);
            if (image.empty()) {
                std::cerr << "image is empty! path: " << image_path << std::endl;
                return;
            }
            // add to system
            system_->AddImage(cam_timestamped / 1e9, image);
            usleep(50000 * nDelayTimes);
        }
        cam_fin.close();
    }
}

void PubImuThread() {
    auto imu_file = boost::filesystem::path(config_dir) / "MH_05_imu0.txt";
    if (boost::filesystem::exists(imu_file)) {
        std::cout << "ImuPubThread Start reading imu file:" << imu_file.string() << std::endl;
    }

    std::ifstream imu_fin;
    imu_fin.open(imu_file);
    std::string imu_line;
    double imu_stamped = 0.0;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    // 能正取读取行并且行的内容不为空
    while (std::getline(imu_fin, imu_line) && !imu_line.empty()) {
        std::istringstream ss_imu(imu_line);
        ss_imu >> imu_stamped >> gyro.x() >> gyro.y() >> gyro.z() >> acc.x() >> acc.y() >> acc.z();
        // add imu data to system
        system_->AddImu(imu_stamped / 1e9, acc, gyro);
        usleep(5000 * nDelayTimes);
    }
    imu_fin.close();
}

int main(int argc, char** argv) {
    std::cout << "Run Eurco Data.........." << std::endl;
    euro_data_dir = "/home/kilox/vslam_ws/src/vio_learn/MH_05_difficult/mav0";
    config_dir = "/home/kilox/vslam_ws/src/vio_learn/config";

    system_ = std::make_shared<vslam::vins::System>(config_dir);

    // 先运行后台的线程
    std::thread backend_trhead(&vslam::vins::System::ProcessBackend, system_);

    // 创建线程并运行起来了
    std::thread pub_image_thread(PubImageThread);
    std::thread pub_imu_thread(PubImuThread);

    pub_image_thread.join();
    pub_imu_thread.join();

    std::cout << "Main thread exits........" << std::endl;

    return 0;
}