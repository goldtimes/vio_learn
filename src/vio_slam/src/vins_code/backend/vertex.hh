/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-03-04 23:12:57
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-03-05 00:38:25
 * @FilePath: /vio_learn/src/vio_slam/src/backend/vertex.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "eigen_types.hh"

namespace vslam::vins {
/**
    优化问题中的顶点类
 */
class Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     * 构造函数
     * @param num_dimension 顶点自身维度
     * @param local_dimension 本地参数化维度，为-1时认为与本身维度一样
     */
    explicit Vertex(int num_dimension, int local_dimension = -1);
    // 需要声明为virtual
    virtual ~Vertex();

    /// 返回变量维度
    int Dimension() const;

    /// 返回变量本地维度
    int LocalDimension() const;

    /// 该顶点的id
    unsigned long Id() const {
        return id_;
    }

    /// 返回参数值
    VecX Parameters() const {
        return values_;
    }

    /// 返回参数值的引用
    VecX &Parameters() {
        return values_;
    }

    /// 设置参数值
    void SetParameters(const VecX &params) {
        values_ = params;
    }

    /// 加法，可重定义
    /// 默认是向量加
    virtual void Plus(const VecX &delta);

    /// 返回顶点的名称，在子类中实现
    virtual std::string TypeInfo() const = 0;

    int OrderingId() const {
        return ordering_id_;
    }

    void SetOrderingId(unsigned long id) {
        ordering_id_ = id;
    };

    /// 固定该点的估计值
    void SetFixed(bool fixed = true) {
        fixed_ = fixed;
    }

    /// 测试该点是否被固定
    bool IsFixed() const {
        return fixed_;
    }

   protected:
    // 存储顶点的值
    VecX values_;
    // 参数的维度 比如四元素
    int local_dimension_;
    // 顶点的id
    unsigned long id_ = 0;

    unsigned long ordering_id_;

    // 顶点是否需要被优化
    bool fixed_ = false;
};
}  // namespace vslam::vins