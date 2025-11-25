// 文件: pin_mj.hpp
#ifndef PIN_MJ_HPP
#define PIN_MJ_HPP

#include <Eigen/Dense>                        // Eigen 向量/矩阵
#include <pinocchio/algorithm/frames.hpp>     // Frame 操作
#include <pinocchio/algorithm/kinematics.hpp> // 前向运动学
#include <pinocchio/multibody/data.hpp>       // 数据结构
#include <pinocchio/multibody/model.hpp>      // 模型定义
#include <pinocchio/parsers/urdf.hpp>         // URDF 解析
#include <string>

/**
 * @brief Pinocchio 插件类，用于运动学计算。
 *
 * 此类从 URDF 加载 Pinocchio 模型，支持浮动基。
 * 提供 Mujoco 数据到 Pinocchio q 的转换，以及 link 四元数获取。
 * 设计为模块化组件，需集成到 ROS 节点中使用。
 */
class PinMj
{
  public:
    /**
     * @brief 构造函数，从 URDF 加载模型。
     * @param urdf_path URDF 文件路径。
     */
    PinMj(const std::string &urdf_path);

    /**
     * @brief 析构函数。
     */
    ~PinMj() = default;

    /**
     * @brief 将 Mujoco 数据转换为 Pinocchio q，并执行前向运动学。
     * @param joint_angles 关节角度向量。
     * @param base_pos 基座位置 (x, y, z)，默认为 [0, 0, 0]。
     * @param base_quat 基座四元数 (x, y, z, w)，默认为 [0, 0, 0, 1]。
     * @return Pinocchio q 向量。
     */
    Eigen::VectorXf mujocoToPinocchio(const Eigen::VectorXf &joint_angles,
                                      const Eigen::Vector3d &base_pos = Eigen::Vector3d::Zero(),
                                      const Eigen::Quaternionf &base_quat = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0));

    /**
     * @brief 获取指定 link 的四元数。
     * @param link_name link 名称。
     * @return 四元数 (x, y, z, w)。
     */
    Eigen::Quaternionf getLinkQuaternion(const std::string &link_name) const;

  private:
    pinocchio::Model model_;                // Pinocchio 模型
    std::unique_ptr<pinocchio::Data> data_; // Pinocchio 数据（unique_ptr 管理）
    Eigen::Vector3d base_pos_world_;        // 基座位置（世界坐标）
    Eigen::Quaternionf base_quat_world_;    // 基座四元数（世界坐标）
};

#endif // PIN_MJ_HPP