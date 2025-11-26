// 文件: mlp_network_io.cpp
#ifndef MLP_NETWORK_IO_HPP
#define MLP_NETWORK_IO_HPP

#include "network_io.hpp"
#include <yaml-cpp/yaml.h> // 假设 YAML 用于配置
#include <iostream>
class MLPNetworkIO : public NetworkIOBase
{
  public:
    /**
     * @brief 构造函数。
     * @param config YAML 配置节点，包含参数设置。
     */
    MLPNetworkIO(const YAML::Node &config_);

    /**
     * @brief 准备网络输入数据。
     * @param observations 当前观测数据 (Eigen::MatrixXf)。
     * @param states 额外状态，如 LSTM 的 hidden 和 cell (std::vector<Eigen::MatrixXf>)。
     * @param time_step 当前时间步长（可选）。
     * @return 准备好的输入列表，供推理引擎使用。
     */
    std::vector<Eigen::MatrixXf> PrepareInputs(const Eigen::MatrixXf &observations,
                                               const std::vector<Eigen::MatrixXf> &states,
                                               float time_step = 0.0) override;

    /**
     * @brief 从网络输出中提取结果。
     * @param raw_outputs 推理引擎返回的原始输出列表。
     * @return 提取的动作 (Eigen::VectorXf) 和更新后的状态 (std::vector<Eigen::MatrixXf>)。
     */
    std::pair<Eigen::VectorXf, std::vector<Eigen::MatrixXf>> ExtractOutputs(
        const std::map<std::string, Eigen::MatrixXf> &raw_outputs) override;

  private:
    int num_obs_;     // 输入观测维度
    int num_actions_; // 输出动作维度
};

#endif // MLP_NETWORK_IO_HPP