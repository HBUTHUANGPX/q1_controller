// 文件: network_io.cpp
#include "../../include/q1_controller/network_io.hpp"

NetworkIOBase::NetworkIOBase(const std::vector<std::string> &input_names,
                             const std::vector<Eigen::VectorXf> &input_shapes,
                             const std::vector<std::string> &output_names,
                             const std::vector<Eigen::VectorXf> &output_shapes)
    : input_names_(input_names), input_shapes_(input_shapes),
      output_names_(output_names), output_shapes_(output_shapes)
{
    // 验证名称与形状数量匹配
    if (input_names_.size() != input_shapes_.size() || output_names_.size() != output_shapes_.size()) {
        throw std::runtime_error("NetworkIOBase: 输入/输出名称与形状数量不匹配。");
    }
    printf("NetworkIOBase: construct function init ok\r\n");
}