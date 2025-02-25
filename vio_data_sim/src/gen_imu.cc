#include <iostream>
#include <vector>
// #include "imu.hpp"
#include "params.hpp"
#include "utilities.hpp"

int main(int argc, char** argv){

    Params imu_param;
    Imu imu_gen(imu_param);
    std::vector<MotionData> imu_datas;
    std::vector<MotionData> imu_with_noise;
    for(double t = imu_param.time_start; t < imu_param.time_end;){
        MotionData imu_data = imu_gen.get_imudata(t);
        imu_datas.push_back(imu_data);

        // add noise
        MotionData data_noise = imu_data;
        imu_gen.addNoise(data_noise);
        imu_with_noise.push_back(data_noise);

        t += 1.0 / imu_param.imu_freq;   
    }

    imu_gen.init_velocity_ = imu_datas[0].Vw;
    imu_gen.init_Rwb_ = imu_datas[0].Rwb;
    imu_gen.init_twb_ = imu_datas[0].twb;
    // save pose
    savePose("../output/imu_pose.txt", imu_datas);
    savePose("../output/imu_pose_noise.txt", imu_with_noise);

    return 0;
}