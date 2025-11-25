// 文件: observation_manager.hpp
#ifndef OBSERVATION_MANAGER_HPP
#define OBSERVATION_MANAGER_HPP

#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

////////////////////////////
#include "data_store.hpp" // 新增：数据存储
#include "obs/base_ang_vel.hpp"
#include "obs/joint_pos.hpp"
#include "obs/joint_vel.hpp"
#include "obs/last_actions.hpp"
#include "obs/motion_joint_pos_command.hpp"
#include "obs/motion_joint_vel_command.hpp"
#include "obs/motion_ref_ori_b.hpp"
#include "obs/gravity_orientation.hpp"
#include "obs/sin_cos.hpp"
#include "obs/cmd_vel.hpp"
#include "observation_base.hpp"
/**
 * @brief 观测管理类，用于从YAML加载配置，创建组件实例，并管理观测拼接。
 *
 * 根据YAML的obs顺序创建子类实例，设置偏移，并更新整体观测矩阵。
 */
class ObservationManager
{
  public:
    /**
     * @brief 构造函数，从YAML加载配置，并接收数据存储指针。
     * @param config YAML节点。
     * @param data_store 共享数据存储指针。
     */
    ObservationManager(const YAML::Node &config, std::shared_ptr<DataStore> data_store);
    
    /**
     * @brief 更新整体观测。
     * @param time_step 当前时间步。
     * @return 更新后的观测矩阵 (1 x total_dim)。
     */
    Eigen::MatrixXf UpdateObservations(float time_step);

  private:
    std::vector<std::shared_ptr<ObservationComponent>> components_; // 组件列表
    int total_dim_ = 0;                                             // 总维度
    std::shared_ptr<DataStore> data_store_;                         // 数据存储
};

#endif // OBSERVATION_MANAGER_HPP