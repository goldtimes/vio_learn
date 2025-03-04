#include "problem.hh"
#include <algorithm>
#include "vertex.hh"

namespace vslam::backend {
Problem::Problem(ProblemType type) : problem_type_(type) {
}

Problem::~Problem() {
}

bool Problem::AddVertex(std::shared_ptr<Vertex> v) {
    // 存在该顶点
    if (verticies_.find(v->Id()) != verticies_.end()) {
        return false;
    } else {
        verticies_.insert(std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
    }
    return true;
}

bool Problem::AddEdge(std::shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) == edges_.end()) {
        edges_.insert(std::pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
    } else {
        // LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
        return false;
    }
    // 边添加完之后遍历该边的所有顶点
    for (const auto& vertex : edge->Verticies()) {
        vertexsIdToEdge.insert(std::pair(vertex->Id(), edge));
    }
    return true;
}

// 求解的流程
bool Problem::Solve(int iterations) {
    return true;
}

}  // namespace vslam::backend