// 文件: motor_manager.hpp
#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include "data_store.hpp"
#include "motor_base.hpp"
#include "actual_virtual_map.hpp"
#include "q1_controller/msg/multi_motor_command.hpp" // 自定义消息
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

#if defined(USE_TENSORRT)
#include "rt_comm_inipc/DoubleBuffer.hpp"
#include "rt_comm_inipc/MotorCommandAPI.hpp"
#include "rt_comm_inipc/MotorInfo.hpp"
#include "rt_comm_inipc/rt_ipc.hpp"
#endif
/**
 * @brief Motor管理类，用于从YAML加载并管理多个MotorBase实例。
 *
 * 此类按YAML顺序初始化电机，提供name/index索引。
 * 集成ROS订阅（/mujoco_joint_states更新当前状态）和发布（/target_pos发送命令）。
 * 设计为模块化组件，需集成到ROS节点中使用。
 */
class MotorManager
{
  public:
    /**
     * @brief 构造函数，从YAML加载电机配置。
     * @param node ROS节点指针，用于创建订阅/发布器。
     * @param config YAML节点，包含"motors"部分。
     */
    // MotorManager(rclcpp::Node::SharedPtr node, const YAML::Node& config);
    MotorManager(rclcpp::Node *node, const YAML::Node &config, std::shared_ptr<DataStore> data_store);
    /**
     * @brief 通过名称获取电机指针。
     * @param name 电机名称。
     * @return 电机共享指针，若不存在返回nullptr。
     */
    std::shared_ptr<MotorBase> getMotorByName(const std::string &name) const;
    std::shared_ptr<MotorBase> getMotorByName_in_id(const std::string &name) const;

    /**
     * @brief 通过索引获取电机指针。
     * @param index 索引（从0开始）。
     * @return 电机共享指针，若越界返回nullptr。
     */
    std::shared_ptr<MotorBase> getMotorByIndex(size_t index) const;
    std::shared_ptr<MotorBase> getMotorByID(int ID) const;

    std::string jointCommand(const Eigen::VectorXf &actions,bool zero_kp,bool zero_kd);
    void jointStateUpdate(const std::string &message, int from_port);

    /**
     * @brief 发布目标位置命令。
     * @param actions 动作向量（目标位置）。
     */
    void publishTargetPos(const Eigen::VectorXf &actions,bool zero_kp,bool zero_kd);

    Eigen::VectorXf getCurrentPos()
    {
        return current_pos_;
    };
    Eigen::VectorXf getCurrentVel()
    {
        return current_vel_;
    };
    std::vector<size_t> findMutualIndices(const std::vector<std::shared_ptr<MotorBase>> &src,
                                          const std::vector<std::shared_ptr<MotorBase>> &target) const;

  private:
    std::vector<std::shared_ptr<MotorBase>> motors_, motors_in_id_; // 电机列表，按YAML顺序
    std::vector<size_t> indices_from_motors,indices_from_motors_in_id;
    std::map<std::string, size_t> name_to_index_,name_to_index_in_id_;                   // 名称到索引的映射
    float ankle_pitch_add_angle_;
    Eigen::VectorXf current_pos_;                                   // 当前位置（从订阅更新）
    Eigen::VectorXf current_vel_;                                   // 当前速度（从订阅更新）

    rclcpp::Node::SharedPtr node_;                                                       // ROS节点指针
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;            // 关节状态订阅器
    rclcpp::Publisher<q1_controller::msg::MultiMotorCommand>::SharedPtr target_pos_pub_;                // 目标位置发布器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_recv_pub_,state_ctrl_pub_;          // 目标位置发布器
    std::shared_ptr<DataStore> data_store_;
    actual_virtual_map avm_;
    std::map<std::string, size_t> joint_indices_in_motors;
    std::vector<std::string> joint_index_in_need;
#if defined(USE_TENSORRT)
    std::vector<MotorInfo> control_data;
#endif

    /**
     * @brief JointState回调函数，更新当前位置和速度。
     * @param msg JointState消息指针。
     */
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

#endif // MOTOR_MANAGER_HPP