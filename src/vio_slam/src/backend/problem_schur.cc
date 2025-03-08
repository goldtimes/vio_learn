/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-04 23:12:57
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-08 10:10:06
 * @FilePath: /vio_learn/src/vio_slam/src/backend/problem.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "problem_schur.hh"
#include <math.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include "edge.hh"
#include "eigen_types.hh"
#include "vertex.hh"

namespace vslam::backend {
ProblemSchur::ProblemSchur(ProblemType type) : problem_type_(type) {
}

ProblemSchur::~ProblemSchur() {
}

bool ProblemSchur::AddVertex(std::shared_ptr<Vertex> v) {
    // 存在该顶点
    if (verticies_.find(v->Id()) != verticies_.end()) {
        return false;
    } else {
        verticies_.insert(std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
    }
    // 不同于之前曲线拟合的方式，添加入vertexpose, hessian的矩阵需要重新调整下
    if (problem_type_ == ProblemType::SLAM_PROBLEM) {
        if (IsPoseVertex(v)) {
            ResizePoseHessiansWhenAddingPose(v);
        }
    }
    return true;
}

bool ProblemSchur::IsPoseVertex(const std::shared_ptr<Vertex>& v) {
    if (v->TypeInfo() == "VertexPose") {
        return true;
    } else {
        return false;
    }
}

bool ProblemSchur::IsLandmarkVertex(const std::shared_ptr<Vertex>& v) {
    std::string type = v->TypeInfo();
    return type == std::string("VertexPointXYZ") || type == std::string("VertexInverseDepth");
}

void ProblemSchur::ResizePoseHessiansWhenAddingPose(const std::shared_ptr<Vertex>& v) {
    // 新的维度
    int size = H_prior_.rows() + v->LocalDimension();
    // 保持原来的数据并且扩大
    H_prior_.conservativeResize(size, size);
    b_prior_.conservativeResize(size);

    // 新增的部分赋值为0
    b_prior_.tail(v->LocalDimension()).setZero();
    H_prior_.rightCols(v->LocalDimension()).setZero();
    H_prior_.bottomRows(v->LocalDimension()).setZero();
}

bool ProblemSchur::AddEdge(std::shared_ptr<Edge> edge) {
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

void ProblemSchur::SetOrdering() {
    ordering_pose_ = 0;
    ordering_landmark_ = 0;
    ordering_total_ = 0;
    int debug;
    // std::cout << "verticies_ size:" << verticies_.size() << std::endl;
    // Note:: verticies_ 是 map 类型的, 顺序是按照 id 号排序的
    for (const auto vertex : verticies_) {
        ordering_total_ += vertex.second->LocalDimension();
        // std::cout << "ordering_total_ :" << ordering_total_ << std::endl;
        if (IsPoseVertex(vertex.second)) {
            //  统计位姿的维度
            debug += vertex.second->LocalDimension();
        }
        // slam问题，我们要统计pose的维度和landmark的维度
        if (problem_type_ == ProblemType::SLAM_PROBLEM) {
            // 设置了顶点的ordering_id
            AddOrderingSLAM(vertex.second);
        }
        if (IsPoseVertex(vertex.second)) {
            std::cout << "pose vertex id:" << vertex.second->Id() << " order: " << vertex.second->OrderingId()
                      << std::endl;
        }
    }
    std::cout << "\n ordered_landmark_vertices_ size : " << idx_landmark_vertices_.size() << std::endl;
    if (problem_type_ == ProblemType::SLAM_PROBLEM) {
        ulong all_pose_dim = ordering_pose_;
        // 这里要把 landmark 的 ordering 加上 pose 的数量，就保持了 landmark 在后,而 pose 在前
        for (const auto& idx_landmark_vertex : idx_landmark_vertices_) {
            idx_landmark_vertex.second->SetOrderingId(idx_landmark_vertex.second->OrderingId() + all_pose_dim);
        }
    }
}

void ProblemSchur::AddOrderingSLAM(const std::shared_ptr<Vertex>& vertex) {
    if (IsPoseVertex(vertex)) {
        vertex->SetOrderingId(ordering_pose_);
        idx_pose_vertices_.insert(std::pair(vertex->Id(), vertex));
        ordering_pose_ += vertex->LocalDimension();

    } else if (IsLandmarkVertex(vertex)) {
        vertex->SetOrderingId(ordering_landmark_);
        ordering_landmark_ += vertex->LocalDimension();
        idx_landmark_vertices_.insert(std::pair(vertex->Id(), vertex));
    }
}

void ProblemSchur::MakeHessian() {
    auto start = std::chrono::system_clock::now();
    ulong size = ordering_total_;
    MatXX H(MatXX::Zero(size, size));
    std::cout << "hessian rows:" << H.rows() << ", cols:" << H.cols() << std::endl;

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
            auto vertex_i = vertices[i];
            // 顶点被固定了
            if (vertex_i->IsFixed()) {
                // 雅可比矩阵为0
                continue;
            }
            // 拿到当前的jacobian
            auto jacobian_i = jacobians[i];
            // 放在H的起始位置
            ulong index_i = vertex_i->OrderingId();
            // 维度
            ulong dim_i = vertex_i->LocalDimension();

            MatXX JtW = jacobian_i.transpose() * edge.second->Information();
            /**
                 a b
               a
               b
            */
            for (size_t j = i; j < vertices.size(); ++j) {
                auto vertex_j = vertices[j];
                // 顶点被固定了
                if (vertex_j->IsFixed()) {
                    // 雅可比矩阵为0
                    continue;
                }
                // 拿到当前的jacobian
                auto jacobian_j = jacobians[j];
                ulong index_j = vertex_j->OrderingId();
                ulong dim_j = vertex_j->LocalDimension();

                MatXX hessian = JtW * jacobian_j;
                // std::cout << "hessian rows:" << hessian.rows() << ", cols:" << hessian.cols() << std::endl;
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
    // 先验的信息
    if (err_prior_.rows() > 0) {
        b_prior_ = H_prior_ * delta_x.head(ordering_pose_);
    }
    // 这里为什么只加上了位姿的先验矩阵
    Hessian_.topLeftCorner(ordering_pose_, ordering_pose_) += H_prior_;
    b_.head(ordering_pose_) += b_prior_;

    // 求解的结果存放在deltax
    delta_x = VecX::Zero(size, 1);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "MakeHessian time used:" << elapsed_seconds.count() << " ms" << std::endl;
}

// 由最初的残差如果下降了1e-6倍之后，就停止迭代
// 取出hessian矩阵中对角线最大的值 * 1e-5为初始的lamda值
void ProblemSchur::ComputeLambdaInitLM() {
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

void ProblemSchur::AddLambdatoHessianLM() {
    // 保存变量
    Hessian_tmp_ = Hessian_;
    for (ulong i = 0; i < Hessian_.cols(); ++i) {
        Hessian_(i, i) += currentLambda_;
    }
}

void ProblemSchur::RemoveLambdaHessianLM() {
    ulong size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    // for (ulong i = 0; i < size; ++i) {
    //     Hessian_(i, i) -= currentLambda_;
    // }
    Hessian_ = Hessian_tmp_;
}

void ProblemSchur::UpdateStates() {
    for (auto vertex : verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x.segment(idx, dim);
        vertex.second->Plus(delta);
    }
    if (err_prior_.rows() > 0) {
        b_prior_ -= H_prior_ * delta_x.head(ordering_pose_);  // update the error_prior
        // err_prior_ = Jt_prior_inv_ * b_prior_.head(ordering_pose_ - 6);
    }
    // if (err_prior_.rows() > 0) {
    //     b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);  // update the error_prior
    //     err_prior_ = Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 6);
    // }
}

bool ProblemSchur::IsGoodStepInLM() {
    double scale = 0;
    scale = delta_x.transpose() * (currentLambda_ * delta_x + b_);
    scale += 1e-3;  // make sure it's non-zero :)

    // recompute residuals after update state
    // TODO:: get robustChi2() instead of Chi2()
    double tempChi = 0.0;
    for (auto edge : edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }
    if (err_prior_.size() > 0) tempChi += err_prior_.norm();

    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && isfinite(tempChi))  // last step was good, 误差在下降
    {
        double alpha = 1. - pow((2 * rho - 1), 3);
        alpha = std::min(alpha, 2. / 3.);
        double scaleFactor = (std::max)(1. / 3., alpha);
        currentLambda_ *= scaleFactor;
        ni_ = 2;
        currentChi_ = tempChi;
        return true;
    } else {
        currentLambda_ *= ni_;
        ni_ *= 2;
        return false;
    }
}

void ProblemSchur::RollbackStates() {
    for (auto vertex : verticies_) {
        ulong idx = vertex.second->OrderingId();
        ulong dim = vertex.second->LocalDimension();
        VecX delta = delta_x.segment(idx, dim);

        // 之前的增量加了后使得损失函数增加了，我们应该不要这次迭代结果，所以把之前加上的量减去。
        vertex.second->Plus(-delta);
    }
}

void ProblemSchur::SolveLinearSystem() {
    if (problem_type_ == ProblemType::GENERIC_PROBLEM) {
        MatXX H = Hessian_;
        // 非 SLAM 问题直接求解
        // PCG solver
        for (ulong i = 0; i < H.rows(); ++i) {
            H(i, i) += currentLambda_;
        }
        delta_x = H.inverse() * b_;
    } else {
        // SLAM 问题采用舒尔补的计算方式
        // step1: schur marginalization --> Hpp, bpp
        // TODO:: home work [cg]. 完成矩阵块取值，Hmm，Hpm，Hmp，bpp，bmm
        // MatXX Hmm = Hessian_.block(?,?, ?, ?);
        // MatXX Hpm = Hessian_.block(?,?, ?, ?);
        // MatXX Hmp = Hessian_.block(?,?, ?, ?);
        // VecX bpp = b_.segment(?,?);
        // VecX bmm = b_.segment(?,?);
        /**
            Hpp Hpm
            Hmp Hmm
        */
        int reserve_size = ordering_pose_;
        int marg_size = ordering_landmark_;
        MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
        MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
        MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);

        VecX bpp = b_.segment(0, reserve_size);
        VecX bmm = b_.segment(reserve_size, marg_size);

        // 求Hpp的舒尔补
        MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
        // Hmm
        // 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
        for (const auto& idx_landmark_vertex : idx_landmark_vertices_) {
            int idx = idx_landmark_vertex.second->OrderingId() - reserve_size;
            int size = idx_landmark_vertex.second->LocalDimension();
            Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
        }
        MatXX tmpH = Hpm * Hmm_inv;
        H_pp_schur_ = Hessian_.block(0, 0, reserve_size, reserve_size) - tmpH * Hmp;
        b_pp_schur_ = bpp - tmpH * bmm;

        VecX delta_x_pp(VecX::Zero(reserve_size));
        // PCG Solver
        for (ulong i = 0; i < ordering_pose_; ++i) {
            H_pp_schur_(i, i) += currentLambda_;
        }

        int n = H_pp_schur_.rows() * 2;                       // 迭代次数
        delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, n);  // 哈哈，小规模问题，搞 pcg 花里胡哨
        delta_x.head(reserve_size) = delta_x_pp;
        //        std::cout << delta_x_pp.transpose() << std::endl;

        // TODO:: home work [cg]. step3: solve landmark
        VecX delta_x_ll(marg_size);
        // delta_x_ll = ???;
        delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
        delta_x.tail(marg_size) = delta_x_ll;
    }
}

/** @brief conjugate gradient with perconditioning
 *
 *  the jacobi PCG method
 *
 */
VecX ProblemSchur::PCGSolver(const MatXX& A, const VecX& b, int maxIter) {
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VecX x(VecX::Zero(rows));
    MatXX M_inv = A.diagonal().asDiagonal().inverse();
    VecX r0(b);  // initial r = b - A*0 = b
    VecX z0 = M_inv * r0;
    VecX p(z0);
    VecX w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VecX r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n) {
        i++;
        VecX z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    return x;
}

// 求解的流程
bool ProblemSchur::Solve(int iterations) {
    // 没有添加顶点或者边
    if (edges_.size() == 0 || verticies_.size() == 0) {
        std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
        return false;
    }
    // 存放一些输出的数据
    outfile_.open("data_mu.txt");
    n_iter_ = -1;
    auto solver_start = std::chrono::system_clock::now();
    // 统计位姿的维度和landmark的维度
    SetOrdering();
    // 计算H矩阵和b矩阵
    MakeHessian();
    // 计算停止迭代的条件和 lamda值, lm初始化
    ComputeLambdaInitLM();
    // lm迭代流程
    bool stop = false;
    int iter = 0;

    while (!stop && (iter < iterations)) {
        std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << std::endl;

        // 迭代，不断尝试lamda，如果发现残差变小则成功
        bool oneStepSuccess = false;
        int false_cnt = 0;
        while (!oneStepSuccess) {
            // 舒尔补的方式求解线性方程
            SolveLinearSystem();
            //  delta_x_ = H.ldlt().solve(b_);
            // 如果增量很小,或者一直失败
            if (delta_x.squaredNorm() <= 1e-6 || false_cnt > 10) {
                std::cout << "delta_x norm:" << delta_x.squaredNorm() << std::endl;
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
            std::cout << "currentChi  <= stopThresholdLM_ " << std::endl;
            stop = true;
        }
        std::cout << "stop:" << stop << ", false count:" << false_cnt << std::endl;
    }
    outfile_.close();
    auto t_solve = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = t_solve - solver_start;
    std::cout << "problem solve cost: " << elapsed_seconds.count() << " ms" << std::endl;
    return true;
}

}  // namespace vslam::backend