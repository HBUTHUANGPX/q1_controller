// 文件: network_io.hpp
#ifndef NETWORK_IO_HPP
#define NETWORK_IO_HPP

#include <Eigen/Dense>
#include <map>    // 添加map支持
#include <memory> // C++17 及以上支持
#include <string>
#include <vector>

/**
 * @brief 网络输入输出基类，用于抽象不同网络结构的输入输出处理。
 *
 * 此基类定义了网络输入和输出的通用接口，使用 Eigen 作为张量表示。
 * 子类需实现具体的输入准备、输出提取逻辑，以支持如 MLP、LSTM 等网络。
 * 输入和输出使用 std::vector<Eigen::MatrixXf> 表示多张量场景。
 */
class NetworkIOBase
{
  public:
    /**
     * @brief 构造函数。
     * @param input_shapes 输入形状列表，每个元素为 Eigen::VectorXf 表示维度 (e.g., {batch, features})。
     * @param output_shapes 输出形状列表，类似输入。
     */
    NetworkIOBase(const std::vector<std::string> &input_names,
                  const std::vector<Eigen::VectorXf> &input_shapes,
                  const std::vector<std::string> &output_names,
                  const std::vector<Eigen::VectorXf> &output_shapes);
    virtual ~NetworkIOBase() = default;

    /**
     * @brief 准备网络输入数据。
     * @param observations 当前观测数据 (Eigen::MatrixXf)。
     * @param states 额外状态，如 LSTM 的 hidden 和 cell (std::vector<Eigen::MatrixXf>)。
     * @return 准备好的输入列表，供推理引擎使用。
     */
    virtual std::vector<Eigen::MatrixXf> PrepareInputs(const Eigen::MatrixXf &observations,
                                                       const std::vector<Eigen::MatrixXf> &states,
                                                       float time_step = 0.0) = 0;

    /**
     * @brief 从网络输出中提取结果。
     * @param raw_outputs 推理引擎返回的原始输出列表。
     * @return 提取的动作 (Eigen::VectorXf) 和更新后的状态 (std::vector<Eigen::MatrixXf>)。
     */
    virtual std::pair<Eigen::VectorXf, std::vector<Eigen::MatrixXf>> ExtractOutputs(
        const std::map<std::string, Eigen::MatrixXf> &raw_outputs) = 0;

    // 获取输入/输出形状，用于验证
    const std::vector<Eigen::VectorXf> &GetInputShapes() const
    {
        return input_shapes_;
    }
    const std::vector<Eigen::VectorXf> &GetOutputShapes() const
    {
        return output_shapes_;
    }

    const std::vector<std::string> &GetInputNames() const
    {
        return input_names_;
    }
    const std::vector<std::string> &GetOutputNames() const
    {
        return output_names_;
    }

  protected:
    std::vector<Eigen::VectorXf> input_shapes_;
    std::vector<Eigen::VectorXf> output_shapes_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
};

#endif // NETWORK_IO_HPP