#pragma once

#include "edge.hh"

namespace vslam::backend {

/**
 * 此边是视觉重投影误差，此边为三元边，与之相连的顶点有：
 * 路标点的逆深度InveseDepth、第一次观测到该路标点的source Camera的位姿T_World_From_Body1，
 * 和观测到该路标点的mearsurement Camera位姿T_World_From_Body2。
 * 注意：verticies_顶点顺序必须为InveseDepth、T_World_From_Body1、T_World_From_Body2。
 */
class EdgeReprojection : public Edge {
   public:
    EdgeReprojection(const Vec3& pt_i, const Vec3& pt_j)
        : Edge(2, 3, std::vector<std::string>{"VertexInverseDepth", "VertexPose", "VertexPose"}) {
        pts_i_ = pt_i;
        pts_j_ = pt_j;
    }

    virtual std::string TypeInfo() const override {
        return "EdgeReprojection";
    }
    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond& qic, Vec3& tic);

   private:
    // 路标的世界系坐标
    Vec3 pts_i_, pts_j_;

    Eigen::Quaterniond qic_;
    Eigen::Vector3d tic_;
};

/**
 * 此边是视觉重投影误差，此边为二元边，与之相连的顶点有：
 * 路标点的世界坐标系XYZ、观测到该路标点的 Camera 的位姿T_World_From_Body1
 * 注意：verticies_顶点顺序必须为 XYZ、T_World_From_Body1。
 */
class EdgeReprojectionXYZ : public Edge {
   public:
    EdgeReprojectionXYZ(const Vec3& pt_w)
        : Edge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}), pt_w_(pt_w) {
    }

    virtual std::string TypeInfo() const override {
        return "EdgeReprojectionXYZ";
    }
    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

    void SetTranslationImuFromCamera(Eigen::Quaterniond& qic, Vec3& tic);

   private:
    // 路标的世界系坐标
    Vec3 pt_w_;
    Eigen::Quaterniond qic_;
    Eigen::Vector3d tic_;
};

// 只优化pose
class EdgeReprojectionPoseOnly : public Edge {
   public:
    EdgeReprojectionPoseOnly(VecX& landmark_world, Mat33& K)
        : Edge(1, 2, std::vector<std::string>{"VertexPose"}), landmark_world_(landmark_world), K_(K) {
    }

    virtual std::string TypeInfo() const override {
        return "EdgeReprojectionPoseOnly";
    }
    /// 计算残差
    virtual void ComputeResidual() override;

    /// 计算雅可比
    virtual void ComputeJacobians() override;

   private:
    // 路标的世界系坐标
    Vec3 landmark_world_;
    // 内参矩阵
    Mat33 K_;
    // 继承了edge的像素观测变量
};

}  // namespace vslam::backend