#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include "edge.hh"
#include "eigen_types.hh"
#include "vertex.hh"

namespace vslam::backend {
class Problem {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // slam的稀疏问题或者通用的最小二乘问题
    enum class ProblemType { SLAM_PROBLEM, GENERIC_PROBLEM };
    using ulong = unsigned long;

    // 存储vertex
    using HashVertex = std::map<unsigned long, std::shared_ptr<Vertex>>;
    // 存储边
    using HashEdge = std::unordered_map<unsigned long, std::shared_ptr<Edge>>;
    // 顶点和边的关系
    using HashVertexIdToEdge = std::unordered_multimap<unsigned long, std::shared_ptr<Edge>>;

    Problem(ProblemType type);
    ~Problem();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool Solve(int iterations);

   private:
    ProblemType problem_type_;
    //    所有的顶点
    HashVertex verticies_;
    // 所有的边
    HashEdge edges_;
    // 顶点和边的关系
    HashVertexIdToEdge vertexsIdToEdge;
    Eigen::Matrix3d mat;
};
}  // namespace vslam::backend