// 文件: mlp_network_io.cpp
#ifndef MLP_NETWORK_IO_HPP
#define MLP_NETWORK_IO_HPP

#include "network_io.hpp"
#include <yaml-cpp/yaml.h> // 假设 YAML 用于配置
#include <iostream>
class MLPNetworkIO : public NetworkIOBase
{
  public:
    MLPNetworkIO(const YAML::Node &config_);
    std::vector<Eigen::MatrixXf> PrepareInputs(const Eigen::MatrixXf &observations,
                                               const std::vector<Eigen::MatrixXf> &states,
                                               float time_step = 0.0) override;
    std::pair<Eigen::VectorXf, std::vector<Eigen::MatrixXf>> ExtractOutputs(
        const std::map<std::string, Eigen::MatrixXf> &raw_outputs) override;

  private:
    int num_obs_;
    int num_actions_;
};

#endif // MLP_NETWORK_IO_HPP