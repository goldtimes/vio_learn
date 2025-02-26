#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include "imu.hpp"


void loadPose(std::string filename, std::vector<MotionData>& pose){
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()){
        std::cerr << "can't open load pose file" << std::endl;
        return ;
    }

    while (!f.eof()){
        std::string s;
        std::getline(f, s);
        if(!s.empty()){
            std::stringstream ss;
            ss << s;

            MotionData data;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss>>time;
            ss>>q.w();
            ss>>q.x();
            ss>>q.y();
            ss>>q.z();
            ss>>t(0);
            ss>>t(1);
            ss>>t(2);
            ss>>gyro(0);
            ss>>gyro(1);
            ss>>gyro(2);
            ss>>acc(0);
            ss>>acc(1);
            ss>>acc(2);

            data.timestamped = time;
            data.imu_gyro = gyro;
            data.imu_acc = acc;
            data.Rwb = Eigen::Matrix3d(q);
            data.twb = t;
            pose.push_back(data);
        }
    }
}

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