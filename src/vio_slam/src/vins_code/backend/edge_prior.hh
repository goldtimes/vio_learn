#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "edge.hh"
#include "eigen_types.hh"

namespace vslam {
namespace vins {

/**
 * EdgeSE3Prior，此边为 1 元边，与之相连的顶点有：Ti
 */
class EdgeSE3Prior : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3Prior(const Vec3 &p, const Qd &q) : Edge(6, 1, std::vector<std::string>{"VertexPose"}), Pp_(p), Qp_(q) {
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "EdgeSE3Prior";
    }

    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

   private:
    Vec3 Pp_;  // pose prior
    Qd Qp_;    // Rotation prior
};

}  // namespace vins
}  // namespace vslam
