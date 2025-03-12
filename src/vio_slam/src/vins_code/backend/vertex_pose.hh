
#include <memory>
#include "vertex.hh"

namespace vslam::vins {
class VertexPose : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() : Vertex(7, 6) {
    }

    virtual void Plus(const VecX &delta) override;

    std::string TypeInfo() const override {
        return "VertexPose";
    }
};
}  // namespace vslam::vins