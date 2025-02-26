#pragma  once
#include "params.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>


Eigen::Matrix3d euler2Rotation(const Eigen::Vector3d& eulerAngles){
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}


// 视频中的推导，世界坐标系的速度到imu的角速度
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}

struct MotionData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamped;
    // 世界坐标系的姿态
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d Vw;

    // 由imu的噪声推导零偏随时间的变换
    Eigen::Vector3d imu_acc_bias;
    Eigen::Vector3d imu_gyro_bias;


    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;
    
};

class Imu{
    public:
        Imu(const Params& params);

        MotionData get_imudata(double t);

        void addNoise(MotionData& data);

        void integImu(const std::vector<MotionData>& input_data,  std::string out_file, bool use_mid_integ = false);

    public:
        Params params_;
        // 假设已知imu的运动方程
        double ellipse_x = 15;
        double ellipse_y = 20;
        double z = 1;
        double K1 = 10; //z轴的运动频率更快
        double K = 2 * M_PI / 20; // 20s刚好旋转一圈

        // 旋转方向上的运动方程
        double k_roll = 0.1;
        double k_pitch = 0.2;

        Eigen::Vector3d gyro_bias_;
        Eigen::Vector3d acc_bias_;

        Eigen::Vector3d init_velocity_;
        Eigen::Vector3d init_twb_;
        Eigen::Matrix3d init_Rwb_;
        // enu坐标系下的初始重力加速度
        Eigen::Vector3d g_{0,0,-9.81};
        // 随机噪声
        std::default_random_engine generator_(std::random_device());


};

Imu::Imu(const Params& params):params_(params){
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();   
}

MotionData Imu::get_imudata(double t){
    MotionData data;

    // 生成世界坐标系的位移
    Eigen::Vector3d position(ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    // 对位置一阶导/二阶导
    Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));
    double K2 = K * K;
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数
    // 生成世界坐标系的旋转
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

    // 由欧拉角到旋转矩阵
    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);
    // imu测量的角速度
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;
    // imu测量的加速度
    Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - g_);

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.Vw = dp;
    data.timestamped = t;
    return data;

}

// 对生成的imu数据模拟添加噪声
void Imu::addNoise(MotionData& data){

    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

     Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = params_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( params_.imu_timestamped ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = params_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( params_.imu_timestamped ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += params_.gyro_bias_sigma * sqrt(params_.imu_timestamped ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += params_.acc_bias_sigma * sqrt(params_.imu_timestamped ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

void Imu::integImu(const std::vector<MotionData>& input_data,  std::string out_file, bool use_mid_integ){
    std::ofstream save_point;
    save_point.open(out_file);

    double dt = params_.imu_timestamped;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);                // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for(int i = 1; i < input_data.size(); ++i){
        MotionData imu_data = input_data[i];
        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d half_dtheta;
        if (!use_mid_integ){
            half_dtheta = imu_data.imu_gyro * dt / 2.0;

        }else{
            half_dtheta = (input_data[i - 1].imu_gyro + input_data[i].imu_gyro) / 2 * dt / 2.0;
        }   

        dq.w() = 1;
        dq.x() = half_dtheta[0];
        dq.y() = half_dtheta[1];
        dq.z() = half_dtheta[2];
        dq.normalize();
        // 欧拉积分
        if (!use_mid_integ){
            Eigen::Vector3d acc_w = Qwb * imu_data.imu_acc + gw;
            Qwb = Qwb * dq;
            Pwb = Pwb + Vw * dt + 0.5 * acc_w * dt * dt;
            Vw = Vw + acc_w * dt;
        }
        // 中值积分
        else {
            Eigen::Vector3d acc_w = (Qwb * input_data[i-1].imu_acc + gw + Qwb * input_data[i].imu_acc + gw) / 2;
            Qwb = Qwb * dq;
            Pwb = Pwb + Vw * dt + 0.5 * acc_w * dt * dt;
            Vw = Vw + acc_w * dt;
        }

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_point<<imu_data.timestamped <<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;
    }
}




