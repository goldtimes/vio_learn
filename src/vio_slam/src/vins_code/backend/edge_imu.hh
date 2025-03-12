#pragma once
#include "edge.hh"
#include "factor/integration_base.hh"
#include "vertex_pose.hh"
#include "vertex_speedbias.hh"

namespace vslam::vins {

class EdgeImu : public Edge {
   public:
    EdgeImu(IntegrationBase* integ)
        : pre_integration_(integ),
          Edge(15, 4, std::vector<std::string>{"VertexPose", "VertexSpeedBias", "VertexPose", "VertexSpeedBias"}) {
    }

    std::string TypeInfo() const override {
        return "EdgeImu";
    }

    void ComputeJacobians() override;

    void ComputeResidual() override;

   private:
    //    预积分传递给边
    IntegrationBase* pre_integration_;
    static Eigen::Vector3d gravity_;

    enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

    Mat33 dp_dba_ = Mat33::Zero();
    Mat33 dp_dbg_ = Mat33::Zero();
    Mat33 dr_dbg_ = Mat33::Zero();
    Mat33 dv_dba_ = Mat33::Zero();
    Mat33 dv_dbg_ = Mat33::Zero();
};
}  // namespace vslam::vins