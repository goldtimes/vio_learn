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
#include <math.h>
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

// 由最初的残差如果下降了1e-6倍之后，就停止迭代
// 取出hessian矩阵中对角线最大的值 * 1e-5为初始的lamda值
void Problem::ComputeLambdaInitLM() {
    ni_ = 2;
    currentLambda_ = -1;
    currentChi_ = 0.0;
    // 遍历所有边,累加所有的误差项
    for (const auto& edge : edges_) {
        currentChi_ += edge.second->Chi2();
    }
    if (err_prior_.rows() > 0) {
        currentChi_ += err_prior_.norm();
    }
    // 停止迭代的阈值
    stopThresholdLM_ = 1e-6 * currentChi_;

    // 计算初始化的lambda
    double maxDiagonal = 0.0;
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (ulong i = 0; i < Hessian_.rows(); ++i) {
        maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
    }
    double tau = 1e-5;
    currentLambda_ = 1e-5 * maxDiagonal;
}

void Problem::AddLambdatoHessianLM() {
    // 保存变量
    Hessian_tmp_ = Hessian_;
    for (ulong i = 0; i < Hessian_.cols(); ++i) {
        Hessian_(i, i) += currentLambda_;
    }
}

void Problem::RemoveLambdaHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    // for (ulong i = 0; i < size; ++i) {
    //     Hessian_(i, i) -= currentLambda_;
    // }
    Hessian_ = Hessian_tmp_;
}

void Problem::UpdateStates() {
    for (auto& v : verticies_) {
        ulong idx = v.first;
        ulong v_dim = v.second->LocalDimension();
        VecX delta = delta_x.segment(idx, v_dim);
        v.second->Plus(delta);
    }
}

bool Problem::IsGoodStepInLM() {
    double scale = 0;
    scale = delta_x.transpose() * (currentLambda_ * delta_x + b_);
    scale += 1e-3;  // make sure it's non-zero :)

    // recompute residuals after update state
    // 统计所有的残差
    double tempChi = 0.0;
    for (auto edge : edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && std::isfinite(tempChi))  // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        n_iter_++;
        outfile_ << n_iter_ << " " << currentLambda_ << std::endl;
        std::cout << n_iter_ << " " << currentLambda_ << std::endl;
        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        n_iter_++;
        outfile_ << n_iter_ << " " << currentLambda_ << std::endl;
        std::cout << n_iter_ << " " << currentLambda_ << "  false" << std::endl;
        return false;
    }
}

void Problem::RollbackStates() {
    for (auto vertex : verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x.segment(idx, dim);

        // 之前的增量加了后使得损失函数增加了，我们应该不要这次迭代结果，所以把之前加上的量减去。
        vertex.second->Plus(-delta);
    }
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
    // 计算H矩阵和b矩阵
    MakeHessian();
    // 计算停止迭代的条件和 lamda值, lm初始化
    ComputeLambdaInitLM();

    // lm迭代流程
    bool stop = false;
    int iter = 0;

    n_iter_++;
    outfile_ << n_iter_ << " " << currentLambda_ << std::endl;
    std::cout << n_iter_ << " " << currentLambda_ << std::endl;

    while (!stop && (iter < iterations)) {
        std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << std::endl;

        // 迭代，不断尝试lamda，如果发现残差变小则成功
        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess) {
            // 将lamda加入hessian_矩阵
            AddLambdatoHessianLM();
            // 求解线性方程
            delta_x = Hessian_.inverse() * b_;
            //  delta_x_ = H.ldlt().solve(b_);
            RemoveLambdaHessianLM();
            // 如果增量很小,或者一直失败
            if (delta_x.norm() <= 1e-6 || false_cnt > 10) {
                stop = true;
                break;
            }
            // 更新变量
            UpdateStates();

            // 如果误差下降lamda值减小，然后再新的线性化点构建Hessian矩阵
            // 如果误差增加，那么lamda值也要增加
            oneStepSuccess = IsGoodStepInLM();
            if (oneStepSuccess) {
                MakeHessian();
                false_cnt = 0;
            } else {
                false_cnt++;
                RollbackStates();  // 误差没下降，回滚
            }
        }
        iter++;
        if (sqrt(currentChi_) <= stopThresholdLM_) {
            stop = true;
        }
    }
    outfile_.close();
    auto t_solve = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = t_solve - solver_start;
    std::cout << "problem solve cost: " << elapsed_seconds.count() << " ms" << std::endl;
    return true;
}

}  // namespace vslam::backend