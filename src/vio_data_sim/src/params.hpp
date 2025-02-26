#pragma  once

#include <Eigen/Eigen>


class Params{
    public:
        Params(){

        }
    public:
        int imu_freq = 200;
        double imu_timestamped = 1.0 / imu_freq;
        double time_start = 0.0;
        double time_end = 20.0;

        // 设置imu零偏随机游走噪声
        double gyro_bias_sigma = 1.0e-5;
        double acc_bias_sigma = 1e-4;
        // 高斯白噪声
        double gyro_noise_sigma = 0.015;
        double acc_noise_sigma = 0.019;

        
};