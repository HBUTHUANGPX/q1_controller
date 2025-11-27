// 文件: mlp_network_io.cpp
#include "../../include/q1_controller/mlp_network_io.hpp"

namespace
{
/**
 * @brief 从 YAML 配置加载输入名称列表。
 * @param cfg YAML 配置节点。
 * @return 输入名称列表。
 */
auto load_input_names = [](const YAML::Node &cfg) -> std::vector<std::string> {
    std::vector<std::string> names;
    for (const auto &input : cfg["inputs"])
    {
        names.push_back(input["name"].as<std::string>());
    }
    return names;
};

/**
 * @brief 从 YAML 配置加载输出名称列表。
 * @param cfg YAML 配置节点。
 * @return 输出名称列表。
 */
auto load_output_names = [](const YAML::Node &cfg) -> std::vector<std::string> {
    std::vector<std::string> names;
    for (const auto &output : cfg["outputs"])
    {
        names.push_back(output["name"].as<std::string>());
    }
    return names;
};

/**
 * @brief 从 YAML 配置生成输入形状列表。
 * @param cfg YAML 配置节点。
 * @return 输入形状列表。
 */
auto make_input_shapes = [](const YAML::Node &cfg) -> std::vector<Eigen::VectorXf> {
    std::vector<Eigen::VectorXf> shapes;
    for (const auto &input : cfg["inputs"])
    {
        const auto &shape_node = input["shape"];
        Eigen::VectorXf shape(shape_node.size());
        for (size_t i = 0; i < shape_node.size(); ++i)
        {
            shape(i) = shape_node[i].as<float>();
        }
        shapes.push_back(shape);
    }
    return shapes;
};

/**
 * @brief 从 YAML 配置生成输出形状列表。
 * @param cfg YAML 配置节点。
 * @return 输出形状列表。
 */
auto make_output_shapes = [](const YAML::Node &cfg) -> std::vector<Eigen::VectorXf> {
    std::vector<Eigen::VectorXf> shapes;
    for (const auto &output : cfg["outputs"])
    {
        const auto &shape_node = output["shape"];
        Eigen::VectorXf shape(shape_node.size());
        for (size_t i = 0; i < shape_node.size(); ++i)
        {
            shape(i) = shape_node[i].as<float>();
        }
        shapes.push_back(shape);
    }
    return shapes;
};
} // namespace

/**
 * @brief MLPNetworkIO 构造函数实现。
 * @param config YAML 配置节点，包含参数设置。
 */
MLPNetworkIO::MLPNetworkIO(const YAML::Node &config_)
    : NetworkIOBase(load_input_names(config_),    // 从 YAML 加载输入名称
                    make_input_shapes(config_),   // 自定义函数生成形状
                    load_output_names(config_),   // 从 YAML 加载输出名称
                    make_output_shapes(config_)), // 自定义函数生成形状
      num_obs_(config_["num_single_obs"].as<int>() * config_["frame_stack"].as<int>()),
      num_actions_(config_["num_actions"].as<int>())
{
    // // printf("MLPNetworkIO: construct function init ok\r\n");
}

/**
 * @brief 准备网络输入数据实现。
 * @param observations 当前观测数据 (Eigen::MatrixXf)。
 * @param states 额外状态，如 LSTM 的 hidden 和 cell (std::vector<Eigen::MatrixXf>)。
 * @param time_step 当前时间步长（可选）。
 * @return 准备好的输入列表，供推理引擎使用。
 */
std::vector<Eigen::MatrixXf> MLPNetworkIO::PrepareInputs(const Eigen::MatrixXf &observations,
                                                         const std::vector<Eigen::MatrixXf> &states, float time_step)
{
    if (!states.empty())
    {
        throw std::runtime_error("MLP 不需要 hidden/cell 状态");
    }
    if (observations.rows() != 1 || observations.cols() != num_obs_)
    {
        throw std::runtime_error("observations 维度错误，期望 [1, " + std::to_string(num_obs_) + "]");
    }

    std::vector<Eigen::MatrixXf> inputs;
    inputs.reserve(input_names_.size());

    for (size_t i = 0; i < input_names_.size(); ++i)
    {
        const std::string &name = input_names_[i];

        if (name == "obs")
        {
            // obs 直接来自外部观测
            inputs.push_back(observations);
        }
        else if (name == "time_step")
        {
            // time_step 是标量输入，自动构造
            Eigen::MatrixXf t(1, 1);
            t(0, 0) = time_step;
            inputs.push_back(t);
        }
        else
        {
            // 未来可能出现的其他输入（例如 privileged information、latent code 等）
            // 这里可以扩展：从 config 或其他来源读取
            throw std::runtime_error("不支持的输入名称: " + name + "，请在 MLPNetworkIO 中添加对应处理逻辑");
        }
    }

    return inputs;
}

/**
 * @brief 提取网络输出数据实现。
 * @param raw_outputs 原始输出数据映射。
 * @return 提取的动作和状态数据。
 */
std::pair<Eigen::VectorXf, std::vector<Eigen::MatrixXf>> MLPNetworkIO::ExtractOutputs(
    const std::map<std::string, Eigen::MatrixXf> &raw_outputs)
{
    // // printf("MLPNetworkIO: ExtractOutputs function start \r\n");
    if (raw_outputs.empty())
    {
        throw std::runtime_error("No outputs from MLP.");
    }
    // 使用"actions"键提取动作（假设第一个输出为actions）
    auto it = raw_outputs.find("actions");
    if (it == raw_outputs.end())
    {
        throw std::runtime_error("Output 'actions' not found in map.");
    }
    Eigen::VectorXf actions = it->second.row(0); // 取第一个输出作为actions
    // // printf("MLPNetworkIO: ExtractOutputs function process ok \r\n");
    // std::cout<<"ExtractOutputs:\r\n"<<actions.transpose()<<std::endl;
    return {actions, {}}; // 无状态
}