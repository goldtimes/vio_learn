/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-04 23:12:57
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-05 00:50:23
 * @FilePath: /vio_learn/src/vio_slam/src/backend/problem.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "problem.hh"
#include <algorithm>
#include <chrono>
#include <iostream>
#include "edge.hh"
#include "eigen_types.hh"
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

void Problem::SetOrdering() {
    ordering_pose_ = 0;
    ordering_landmark_ = 0;
    ordering_total_ = 0;

    for (const auto vertex : verticies_) {
        ordering_total_ += vertex.second->LocalDimension();
    }
}

void Problem::MakeHessian() {
    auto start = std::chrono::system_clock::now();
    ulong size = ordering_total_;
    MatXX H(MatXX::Zero(size, size));
    VecX b(VecX::Zero(size, 1));

    // 遍历每条边，计算边的残差和雅可比矩阵
    for (auto& edge : edges_) {
        // 计算残差
        edge.second->ComputeResidual();
        edge.second->ComputeJacobians();
        // 雅可比矩阵
        auto jacobians = edge.second->Jacobians();
        // 顶点
        auto vertices = edge.second->Verticies();
        // 遍历顶点
        for (size_t i = 0; i < vertices.size(); ++i) {
            auto vertex = vertices[i];
            // 顶点被固定了
            if (vertex->IsFixed()) {
                // 雅可比矩阵为0
                continue;
            }
            // 拿到当前的jacobian
            auto jacobian = jacobians[i];
            // 放在H的起始位置
            ulong index_i = vertex->OrderingId();
            // 维度
            ulong dim_i = vertex->LocalDimension();

            MatXX JtW = jacobian.transpose() * edge.second->Information();
            /**
                 a b
               a
               b
            */
            for (int j = i; j < vertices.size(); ++j) {
                auto vertex = vertices[j];
                // 顶点被固定了
                if (vertex->IsFixed()) {
                    // 雅可比矩阵为0
                    continue;
                }
                // 拿到当前的jacobian
                auto jacobian = jacobians[j];
                ulong index_j = vertex->OrderingId();
                ulong dim_j = vertex->LocalDimension();

                MatXX hessian = JtW * jacobian;
                // 累加起来
                H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
                if (j != i) {
                    // 对称的下三角
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
                }
            }
            //
            b.segment(index_i, dim_i).noalias() += -JtW * edge.second->Residual();
        }
    }

    Hessian_ = H;
    b_ = b;
    // 求解的结果存放在deltax
    delta_x = VecX::Zero(size, 1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "MakeHessian time used:" << elapsed_seconds.count() << " ms" << std::endl;
}

// 求解的流程
bool Problem::Solve(int iterations) {
    // 没有添加顶点或者边
    if (edges_.size() == 0 || verticies_.size() == 0) {
        std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
        return false;
    }
    // 存放一些输出的数据
    outfile_.open("data_mu.txt");
    n_iter_ = -1;
    auto solver_start = std::chrono::system_clock::now();
    // 计算优化的变量维度
    SetOrdering();

    return true;
}

}  // namespace vslam::backend