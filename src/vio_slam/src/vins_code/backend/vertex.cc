#include "vertex.hh"

namespace vslam::vins {
// 全局变量
unsigned long global_vertex_id_ = 0;
Vertex::Vertex(int num_dimension, int local_dimension) {
    values_.resize(num_dimension, 1);
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = global_vertex_id_++;
}

Vertex::~Vertex() {
}

int Vertex::Dimension() const {
    return values_.rows();
}

int Vertex::LocalDimension() const {
    return local_dimension_;
}

void Vertex::Plus(const VecX &delta) {
    values_ += delta;
}
}  // namespace vslam::backend