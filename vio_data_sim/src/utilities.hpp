#pragma once
#include <fstream>
#include <vector>
#include <iostream>
#include "imu.hpp"

void savePose(std::string filename, std::vector<MotionData>& pose){
    std::ofstream save_points;
        save_points.open(filename.c_str());

        for (int i = 0; i < pose.size(); ++i) {
            MotionData data = pose[i];
            double time = data.timestamped;
            Eigen::Quaterniond q(data.Rwb);
            Eigen::Vector3d t = data.twb;
            Eigen::Vector3d gyro = data.imu_gyro;
            Eigen::Vector3d acc = data.imu_acc;

            save_points<<time<<" "
                    <<q.w()<<" "
                    <<q.x()<<" "
                    <<q.y()<<" "
                    <<q.z()<<" "
                    <<t(0)<<" "
                    <<t(1)<<" "
                    <<t(2)<<" "
                    <<gyro(0)<<" "
                    <<gyro(1)<<" "
                    <<gyro(2)<<" "
                    <<acc(0)<<" "
                    <<acc(1)<<" "
                    <<acc(2)<<" "
                    <<std::endl;
        }
}