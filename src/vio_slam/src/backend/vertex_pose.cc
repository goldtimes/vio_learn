#include "vertex_pose.hh"
#include "so3.hpp"

namespace vslam::backend {

void VertexPose::Plus(const VecX& delta) {
    VecX& origin_value = Parameters();
    origin_value.head<3>() = delta.head<3>();
    Eigen::Quaterniond origin_q(origin_value[6], origin_value[3], origin_value[4], origin_value[5]);
    Eigen::Quaterniond update_q =
        origin_q * Sophus::SO3d::exp(Eigen::Vector3d(delta[3], delta[4], delta[5])).unit_quaternion();
    update_q.normalized();
    origin_value[3] = update_q.x();
    origin_value[4] = update_q.y();
    origin_value[5] = update_q.z();
    origin_value[6] = update_q.w();
}
}  // namespace vslam::backend