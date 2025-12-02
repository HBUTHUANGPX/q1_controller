// 文件: observation_base.hpp
#ifndef OBSERVATION_BASE_HPP
#define OBSERVATION_BASE_HPP

#include "data_store.hpp" // 新增
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <iostream>
/**
 * @brief 观测组件基类，用于抽象不同观测数据的初始化、更新和命名。
 *
 * 此基类定义了观测组件的通用接口，每个子类代表一种特定的观测数据来源（如关节位置、角速度等）。
 * 每个组件负责管理自己的数据维度、名称，并提供更新逻辑。
 * 更新时，组件将数据填充到整体观测向量的指定偏移位置。
 */
class ObservationComponent
{
  public:
    /**
     * @brief 构造函数。
     * @param dim 数据维度。
     * @param name 组件名称，用于日志或调试。
     * @param scale 数据缩放因子（从YAML读取）。
     */
    ObservationComponent(int dim, const std::string &name, float scale = 1.0,
                         std::shared_ptr<DataStore> data_store = nullptr);
    virtual ~ObservationComponent() = default;

    /**
     * @brief 更新组件数据。
     * @param obs 整体观测矩阵 (Eigen::MatrixXf)，组件将数据填充到 offset_ 开始的位置。
     * @param time_step 当前时间步（可选，用于时间相关计算）。
     */
    virtual void Update(Eigen::MatrixXf &obs, float time_step = 0.0) = 0;

    // 获取维度
    int GetDim() const
    {
        return dim_;
    }

    // 获取名称
    const std::string &GetName() const
    {
        return name_;
    }

    // 设置偏移（在整体 obs 中的起始位置）
    void SetOffset(int offset)
    {
        offset_ = offset;
    }

    // 获取偏移
    int GetOffset() const
    {
        return offset_;
    }

  protected: // TODO： 清除无用变量
    int dim_;                               // 数据维度（从YAML len读取）
    std::string name_;                      // 组件名称（从YAML键读取）
    float scale_;                          // 数据缩放因子（从YAML scale读取）
    int offset_ = 0;                        // 在整体 obs 中的偏移（由管理类设置，根据slice
    std::shared_ptr<DataStore> data_store_; // 共享数据存储
};

#endif // OBSERVATION_BASE_HPP