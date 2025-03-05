#include "edge.hh"
#include <iostream>
#include "vertex.hh"

namespace vslam::backend {
unsigned long global_edge_id = 0;
Edge::Edge(int residual_dimension, int num_verticies, const std::vector<std::string> &verticies_types) {
    // 残差的维度
    residual_.resize(residual_dimension);
    if (!verticies_types.empty()) {
        verticies_types_ = verticies_types;
    }
    // jacobians的个数
    jacobians_.resize(num_verticies);
    // id
    id_ = global_edge_id++;
    // 信息矩阵
    Eigen::MatrixXd information(residual_dimension, residual_dimension);
    information.setIdentity();
    information_ = information;
}

Edge::~Edge() {
}

double Edge::Chi2() {
    return residual_.transpose() * information_ * residual_;
}

bool Edge::CheckValid() {
    if (!verticies_types_.empty()) {
        for (size_t i = 0; i < verticies_.size(); ++i) {
            if (verticies_types_[i] != verticies_[i]->TypeInfo()) {
                std::cout << "Vertex type does not match, should be " << verticies_types_[i] << ", but set to "
                          << verticies_[i]->TypeInfo() << std::endl;
                return false;
            }
        }
    }
    return true;
}
}  // namespace vslam::backend