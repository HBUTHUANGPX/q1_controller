// 文件: inference_base.hpp
#ifndef INFERENCE_BASE_HPP
#define INFERENCE_BASE_HPP

#include <Eigen/Dense>
#include <map>
#include <memory> // C++17
#include <mutex>  // 用于线程安全，如果需要
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include "network_io.hpp"
#include <yaml-cpp/yaml.h> // 假设 YAML 用于配置
#include "motor_manager.hpp" // 新增：Motor管理类
/**
 * @brief 推理引擎基类，用于抽象不同后端的推理逻辑。
 *
 * 此基类管理模型加载、状态维护和推理过程。
 * 子类需实现后端特定的加载和推理方法。
 * 使用 NetworkIOBase 来抽象输入输出处理，支持扩展不同网络结构。
 */
class InferenceBase
{
  public:
    /**
     * @brief 构造函数。
     * @param config YAML 配置路径，用于加载参数。
     * @param network_io 网络 IO 处理器共享指针。
     * @param motor_manager Motor 管理器共享指针。
     */
    InferenceBase(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,std::shared_ptr<MotorManager> motor_manager);

    virtual ~InferenceBase() = default;

    /**
     * @brief 更新动作：执行推理并返回动作。
     * @param observations 当前观测 (Eigen::MatrixXf)。
     * @return 计算得到的动作 (Eigen::VectorXf)。
     */
    Eigen::VectorXf UpdateAction(const Eigen::MatrixXf &observations, float time_step);

    /**
     * @brief 使用kp和tq max重新计算action scale。
     * @param motor_manager 。
     * @return void。
     */
    void UpdateActionScale(std::shared_ptr<MotorManager> motor_manager);


    /**
     * @brief 对action进行scale和clamp。
     * @param actions 网络输出的原始action (Eigen::VectorXf)。
     * @return 计算得到的动作 (Eigen::VectorXf)。
     */
    Eigen::VectorXf Scale_and_clamp_Action(const Eigen::VectorXf &actions);

    /**
     * @brief 重置内部状态（如 LSTM 的 hidden 和 cell）。
     */
    void ResetMemory();

    // 获取配置（如果需要）
    const YAML::Node &GetConfig() const
    {
        return config_;
    }

    /**
     * @brief 执行推理（纯虚函数，由子类实现）。
     * @param inputs 准备好的输入列表。
     * @return 原始输出列表。
     */
    virtual std::map<std::string, Eigen::MatrixXf> Infer(const std::vector<Eigen::MatrixXf> &inputs) = 0;

  protected:
    /**
     * @brief 加载模型（纯虚函数，由子类实现）。
     * @param model_path 模型路径。
     */
    virtual void LoadModel(const std::string &model_path) = 0;

    // 内部状态：如 LSTM 的 hidden 和 cell
    std::vector<Eigen::MatrixXf> states_;

    // 配置和参数
    YAML::Node config_;
    std::string policy_path_;
    int num_actions_;
    int hidden_size_;
    int num_layers_;
    float action_limit_;
    bool isaac_sim_trans_flag_,recal_action_scale_by_tq_max_and_kp;
    std::vector<int> isaac_sim2mujoco_index;
    std::vector<float> action_scale_;
    // 网络 IO 处理器
    std::shared_ptr<NetworkIOBase> network_io_;

    // 互斥锁，用于线程安全推理（如果多线程）
    std::mutex infer_mutex_;
};

#endif // INFERENCE_BASE_HPP