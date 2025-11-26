// 文件: onnx_inference.hpp
#ifndef ONNX_INFERENCE_HPP
#define ONNX_INFERENCE_HPP

#include "inference_base.hpp"
#include <onnxruntime_cxx_api.h>  // ONNX Runtime C++ API 头文件（基于提供的onnxruntime_cxx_api.h）
#include "motor_manager.hpp"      // 新增：Motor 管理类

/**
 * @brief ONNX Runtime 后端的推理类，继承自 InferenceBase。
 *
 * 此类实现 ONNX Runtime 特定的模型加载和推理逻辑。
 * 假设使用 CPU 设备；支持扩展到 GPU（需配置 SessionOptions）。
 * 输入假设为 "obs" 和 "time_step"，输出至少包括 "actions"。
 * 使用本地 ONNX Runtime 库路径，避免系统安装。
 * 基于提供的头文件，确保使用Ort::Env、Ort::Session等包装类，实现异常安全和RAII资源管理。
 */
class OnnxInference : public InferenceBase
{
  public:
    /**
     * @brief 构造函数，初始化基类并加载模型。
     * @param config YAML 配置节点，用于获取参数如 policy_path_。
     * @param network_io 网络 IO 处理器共享指针，用于输入输出处理。
     * @param motor_manager Motor 管理器共享指针，用于动作缩放等。
     * 初始化Ort::Env时设置日志级别为警告，以减少不必要输出（匹配onnxruntime_cxx_inline.h中的Env构造函数）。
     */
    OnnxInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io, std::shared_ptr<MotorManager> motor_manager);

  protected:
    /**
     * @brief 加载 ONNX 模型（实现基类纯虚函数）。
     * @param model_path 模型文件路径（.onnx）。
     * 创建Ort::SessionOptions和Ort::Session，并验证输入/输出形状与配置匹配。
     * 使用GetInputNameAllocated等API获取名称，并抛出Ort::Exception异常（匹配ThrowOnError机制）。
     */
    void LoadModel(const std::string &model_path) override;

    /**
     * @brief 执行推理（实现基类纯虚函数）。
     * @param inputs 准备好的输入列表（Eigen::MatrixXf）。
     * @return 输出映射，键为输出名称（如 "actions"），值为 Eigen::MatrixXf。
     * 转换输入为 Ort::Value (Tensor)，运行会话，提取输出。
     * 使用std::lock_guard确保线程安全，匹配基类的infer_mutex_。
     */
    std::map<std::string, Eigen::MatrixXf> Infer(const std::vector<Eigen::MatrixXf> &inputs) override;

  private:
    Ort::Env env_;                        // ONNX Runtime 环境实例（全局共享，日志级别为警告）。
    Ort::SessionOptions session_options_; // 会话选项（如线程数和优化级别）。
    Ort::Session session_;                // ONNX 会话实例，用于推理。
    Ort::AllocatorWithDefaultOptions allocator_; // 默认分配器，用于获取节点名称等。
    std::vector<std::shared_ptr<char>> inputNames;
    std::vector<std::shared_ptr<char>> outputNames;
    std::vector<char *> inputNodeNames;
    std::vector<char *> outputNodeNames;
    // 创建输入 tensor（使用辅助函数）。
    std::vector<Ort::Value> input_tensors;
    size_t inputNodeCount,outputNodeCount;
    std::vector<std::vector<int64_t>> input_shapes_;  // 存储动态形状
    std::vector<std::vector<int64_t>> output_shapes_;

    // 辅助函数：将 ONNX Ort::Value 转换为 Eigen 矩阵。
    // 使用GetTensorMutableData<float>获取数据，并复制到Eigen矩阵（确保RAII安全）。
    Eigen::MatrixXf OrtValueToEigen(const Ort::Value &value);
};

#endif // ONNX_INFERENCE_HPP