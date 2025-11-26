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
     * @brief 构造函数DataStore，初始化数据向量。
     * @param config YAML配置节点，包含参数设置
     * @param motion_loader 运动数据加载器指针
     */
    DataStore(const YAML::Node &config, std::shared_ptr<MotionLoader> motion_loader);

    /**
     * @brief 更新关节位置数据
     * @param positions 新的关节位置
     */
    void UpdateJointPositions(const Eigen::VectorXf &positions);

    /**
     * @brief 更新关节速度数据
     * @param velocities 新的关节速度
     */
    void UpdateJointVelocities(const Eigen::VectorXf &velocities);

    /**
     * @brief 更新基座角速度数据
     * @param ang_vel 新的基座角速度
     */
    void UpdateBaseAngularVelocities(const Eigen::VectorXf &ang_vel);

    /**
     * @brief 更新最后动作数据
     * @param actions 新的最后动作
     */
    void UpdateLastActions(const Eigen::VectorXf &actions);

    /**
     * @brief 更新运动参考方向四元数（用于计算方向矩阵）
     * @param robot_quat 新的运动参考方向四元数
     */
    void UpdateRobotQuat(const Eigen::Quaternionf &robot_quat);

    /**
     * @brief 更新cmd vel指令
     * @param cmd_Vel 新的cmd vel指令
     */
    void UpdateCmdVel(const Eigen::VectorXf &cmd_Vel);

    /**
     * @brief 获取cmd vel指令
     * @return cmd vel指令
     */
    Eigen::VectorXf GetCmdVel() const;
    
    /**
     * @brief 获取关节位置
     * @return 关节位置
     */
    Eigen::VectorXf GetJointPositions() const;

    /**
     * @brief 获取关节速度
     * @return 关节速度
     */
    Eigen::VectorXf GetJointVelocities() const;

    /**
     * @brief 获取基座角速度
     * @return 基座角速度
     */
    Eigen::VectorXf GetBaseAngularVelocities() const;

    /**
     * @brief 获取重力投影
     * @return 重力投影
     */
    Eigen::Vector3f get_gravity_orientation() const;

    /**
     * @brief 获取最后动作
     * @return 最后动作
     */
    Eigen::VectorXf GetLastActions() const;

    /**
     * @brief 获取运动关节位置命令
     * @param time_step 时间步长
     * @return 运动关节位置命令
     */
    Eigen::VectorXf GetMotionJointPosCommand(float time_step) const;

    /**
     * @brief 获取运动关节速度命令
     * @param time_step 时间步长
     * @return 运动关节速度命令
     */
    Eigen::VectorXf GetMotionJointVelCommand(float time_step) const;

    /**
     * @brief 计算运动参考方向矩阵
     * @param time_step 时间步长
     * @return 运动参考方向矩阵
     */
    Eigen::Matrix3f ComputeMotionRefOriMatrix(float time_step) const;

  private:
    mutable std::mutex mutex_;                                      // 互斥锁，确保线程安全

    bool isaac_sim_trans_flag_;                                     // Isaac Sim坐标转换标志  
    int num_actions_;                                               // 动作数量
    std::vector<int> mujoco2isaac_sim_index;
    Eigen::VectorXf joint_positions_, motion_joint_pos_commands_;   // 关节位置
    Eigen::VectorXf joint_velocities_, motion_joint_vel_commands_;  // 关节速度
    Eigen::VectorXf base_ang_vel_;                                  // 基座角速度
    Eigen::VectorXf last_actions_;                                  // 最后动作
    Eigen::VectorXf cmd_vel_, gamepad_cmd_vel;                      // 最后动作
    float scale_vx, scale_vy, scale_wz;                             // cmd vel缩放系数
    Eigen::Quaternionf robot_quat_, motion_ref_quats_;              // 机器人四元数
    std::shared_ptr<MotionLoader> motion_loader_;                   // 运动数据加载器指针
};

#endif // DATA_STORE_HPP