#pragma once

#include "vertex.hh"

namespace vslam::vins {
class VertexSpeedBias : public Vertex {
   public:
    VertexSpeedBias() : Vertex(9) {
    }

    std::string TypeInfo() const override {
        return "VertexSpeedBias";
    }
};
}  // namespace vslam::vins