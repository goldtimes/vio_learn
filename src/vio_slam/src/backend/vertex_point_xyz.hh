#pragma once

#include "vertex.hh"

// 以xyz形式存储的特征点形式
namespace vslam::backend {
class VertexPointXYZ : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointXYZ() : Vertex(3) {
    }
    // 更新方式默认的+法

    virtual std::string TypeInfo() const override {
        return "VertexPointXYZ";
    }
};
}  // namespace vslam::backend