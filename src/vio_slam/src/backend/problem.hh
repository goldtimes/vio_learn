/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-04 23:12:57
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-05 00:16:40
 * @FilePath: /vslam_ws/src/vio_learn/src/vio_slam/src/backend/problem.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <fstream>
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

    std::ofstream outfile_;
    // 记录迭代的次数
    int n_iter_;

    // 位姿的维度
    ulong ordering_pose_;
    // landmark的维度
    ulong ordering_landmark_;
    // 总的维度
    ulong ordering_total_;

    // Hessian矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x;

   private:
    //    统计优化变量的维度
    void SetOrdering();
    // J^T * information * J
    void MakeHessian();
};
}  // namespace vslam::backend