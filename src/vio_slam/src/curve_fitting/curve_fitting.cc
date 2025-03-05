#include <iostream>
#include <random>
#include "edge.hh"
#include "problem.hh"
#include "vertex.hh"

using namespace vslam::backend;
using namespace std;

// 曲线拟合，估计abc三个参数，三个参数为顶点
class CurveFittingVertex : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingVertex() : Vertex(3) {
    }
    ~CurveFittingVertex() {
    }

    virtual std::string TypeInfo() const {
        return "abc";
    }

   private:
};
// 边，误差函数
class CurveFittingEdge : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x, double y) : Edge(1, 1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }

    // 计算残差
    virtual void ComputeResidual() override {
        // 获取待优化的参数
        auto abc = verticies_[0]->Parameters();
        // 曲线方程解算的值(估计值) - 观测值
        residual_[0] = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]) - y_;
    }

    // 计算雅可比矩阵

    virtual void ComputeJacobians() override {
        auto abc = verticies_[0]->Parameters();
        double exp_y = std::exp(abc[0] * x_ * x_ + abc[1] * x_ + abc[2]);
        // 1x3的jacobian矩阵
        Eigen::Matrix<double, 1, 3> jacobian_abc;
        jacobian_abc << x_ * x_ * exp_y, x_ * exp_y, exp_y;
        jacobians_[0] = jacobian_abc;
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
        return "CurveFittingEdge";
    }

   public:
    double x_;
    double y_;
};

int main(int argc, char** argv) {
    // 设置真实的参数
    double a = 1.0, b = 2.0, c = 1.0;
    // 数据点
    int N = 100;
    // 噪声sigma值 越大，最后迭代出来的参数越准确
    double w_sigma = 0.1;

    // 创建随机数的生成器
    std::default_random_engine generator;
    // 正态分布的随机数
    std::normal_distribution<double> noise(0, w_sigma);

    // 构建优化问题
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM);

    // 顶点
    std::shared_ptr<CurveFittingVertex> vertex = std::make_shared<CurveFittingVertex>();
    // 设置带估计的参数初始值
    vertex->SetParameters(Eigen::Vector3d(0, 0, 0));

    problem.AddVertex(vertex);
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        double n = noise(generator);
        double y = std::exp(a * x * x + b * x + c) + n;
        std::shared_ptr<CurveFittingEdge> edge(new CurveFittingEdge(x, y));
        // 设置每个边关联的顶点
        edge->SetVertex(std::vector<std::shared_ptr<Vertex>>{vertex});
        problem.AddEdge(edge);
    }

    std::cout << "\nTest CurveFitting start..." << std::endl;
    /// 使用 LM 求解
    problem.Solve(30);  // 30

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;

    return 0;
}