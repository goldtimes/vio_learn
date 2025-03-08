/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-04 23:12:57
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-08 12:21:06
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
class ProblemSchur {
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

    ProblemSchur(ProblemType type);
    ~ProblemSchur();

    bool AddVertex(std::shared_ptr<Vertex> vertex);

    bool AddEdge(std::shared_ptr<Edge> edge);

    bool Solve(int iterations);

    void TestMarginalize();

   private:
    ProblemType problem_type_;
    //    所有的顶点
    // std::map 默认使用小于运算符 < 来比较键，因此它按照键的升序来存储元素。
    HashVertex verticies_;
    // 所有的边
    HashEdge edges_;
    // 顶点和边的关系
    HashVertexIdToEdge vertexsIdToEdge;

    std::ofstream outfile_;
    // 记录迭代的次数
    int n_iter_;

    // 位姿的维度
    HashVertex idx_pose_vertices_;
    ulong ordering_pose_;
    // landmark的维度
    HashVertex idx_landmark_vertices_;
    ulong ordering_landmark_;
    // 总的维度
    ulong ordering_total_;

    // Hessian矩阵
    MatXX Hessian_;
    VecX b_;
    VecX delta_x;

    MatXX Hessian_tmp_;

    // 先验
    VecX err_prior_;
    MatXX H_prior_;
    VecX b_prior_;

    /// SBA的Pose部分
    MatXX H_pp_schur_;
    VecX b_pp_schur_;

    int ni_;
    double currentLambda_;
    double stopThresholdLM_;
    double currentChi_;

   private:
    //    统计优化变量的维度
    void SetOrdering();
    // J^T * information * J
    void MakeHessian();

    void ComputeLambdaInitLM();
    void AddLambdatoHessianLM();

    void RemoveLambdaHessianLM();
    void UpdateStates();
    bool IsGoodStepInLM();
    void RollbackStates();

    bool IsPoseVertex(const std::shared_ptr<Vertex>& v);
    bool IsLandmarkVertex(const std::shared_ptr<Vertex>& v);
    void ResizePoseHessiansWhenAddingPose(const std::shared_ptr<Vertex>& v);
    void AddOrderingSLAM(const std::shared_ptr<Vertex>& vertex);
    void SolveLinearSystem();
    VecX PCGSolver(const MatXX& A, const VecX& b, int maxIter = -1);
};
}  // namespace vslam::backend