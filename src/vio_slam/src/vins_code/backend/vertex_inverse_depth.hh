#pragma once

#include "vertex.hh"

// 以xyz形式存储的特征点形式
namespace vslam::vins {
class VertexInverseDepth : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexInverseDepth() : Vertex(1) {
    }
    // 更新方式默认的+法

    virtual std::string TypeInfo() const override {
        return "VertexInverseDepth";
    }
};
}  // namespace vslam::vins