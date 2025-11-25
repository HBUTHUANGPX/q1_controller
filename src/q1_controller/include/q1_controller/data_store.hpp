// 文件: data_store.hpp
#ifndef DATA_STORE_HPP
#define DATA_STORE_HPP

#include "motion_loader.hpp"
#include <Eigen/Dense>
#include <map>
#include <mutex>
#include <vector>
#include <yaml-cpp/yaml.h>
/**
 * @brief 数据存储类，用于持有机器人最新观测数据，支持线程安全访问。
 *
 * 此类作为共享存储，外部回调函数更新数据，观测组件从中读取。
 * 支持关节位置、速度、IMU角速度、动捕命令等数据。
 */
class DataStore
{
  public:
    /**
     * @brief 构造函数，初始化数据向量。
     * @param joint_dim 关节维度（用于初始化向量大小）。
     */
    DataStore(const YAML::Node &config, std::shared_ptr<MotionLoader> motion_loader);

    // 更新关节位置数据（例如从receiveMotordata回调）
    void UpdateJointPositions(const Eigen::VectorXf &positions);

    // 更新关节速度数据
    void UpdateJointVelocities(const Eigen::VectorXf &velocities);

    // 更新基座角速度数据
    void UpdateBaseAngularVelocities(const Eigen::VectorXf &ang_vel);

    // 更新最后动作数据
    void UpdateLastActions(const Eigen::VectorXf &actions);

    // 更新运动参考方向四元数（用于计算方向矩阵）
    void UpdateRobotQuat(const Eigen::Quaternionf &robot_quat);

    // 更新cmd vel 指令
    void UpdateCmdVel(const Eigen::VectorXf &cmd_Vel);

    // 获取cmd vel指令
    Eigen::VectorXf GetCmdVel() const;
    // 获取关节位置
    Eigen::VectorXf GetJointPositions() const;

    // 获取关节速度
    Eigen::VectorXf GetJointVelocities() const;

    // 获取基座角速度
    Eigen::VectorXf GetBaseAngularVelocities() const;

    // 获取重力投影
    Eigen::Vector3f get_gravity_orientation() const;

    // 获取最后动作
    Eigen::VectorXf GetLastActions() const;

    // 获取运动关节位置命令
    Eigen::VectorXf GetMotionJointPosCommand(float time_step) const;

    // 获取运动关节速度命令
    Eigen::VectorXf GetMotionJointVelCommand(float time_step) const;

    // 计算运动参考方向矩阵（body frame）
    Eigen::Matrix3f ComputeMotionRefOriMatrix(float time_step) const;

  private:
    mutable std::mutex mutex_; // 互斥锁，确保线程安全

    bool isaac_sim_trans_flag_;
    int num_actions_;
    std::vector<int> mujoco2isaac_sim_index;
    Eigen::VectorXf joint_positions_, motion_joint_pos_commands_;  // 关节位置
    Eigen::VectorXf joint_velocities_, motion_joint_vel_commands_; // 关节速度
    Eigen::VectorXf base_ang_vel_;                                 // 基座角速度
    Eigen::VectorXf last_actions_;                                 // 最后动作
    Eigen::VectorXf cmd_vel_, gamepad_cmd_vel;                     // 最后动作
    float scale_vx, scale_vy, scale_wz;
    Eigen::Quaternionf robot_quat_, motion_ref_quats_;
    std::shared_ptr<MotionLoader> motion_loader_;
};

#endif // DATA_STORE_HPP