// 文件: tensorrt_inference.hpp
#ifndef TENSORRT_INFERENCE_HPP
#define TENSORRT_INFERENCE_HPP

#include "inference_base.hpp"  // 基类头文件
#include <NvInfer.h>           // TensorRT 核心头文件
#include <NvOnnxParser.h>      // ONNX 解析器头文件
#include <cuda_runtime_api.h>  // CUDA 运行时 API
#include "motor_manager.hpp"   // Motor 管理类头文件

// 使用 TensorRT 命名空间
using namespace nvinfer1;

/**
 * @brief TensorRT 后端的推理类，继承自 InferenceBase。
 *
 * 此类实现 TensorRT 特定的模型加载和推理逻辑。
 * 支持从 ONNX 文件构建或加载序列化引擎，支持 FP16 优化。
 * 输入假设为 "obs" 和 "time_step"，输出至少包括 "actions"。
 * 使用 Logger 类处理日志，确保线程安全。
 */
class TensorRTInference : public InferenceBase
{
public:
    /**
     * @brief 构造函数，初始化基类并加载模型。
     * @param config YAML 配置节点，用于获取参数如 policy_path_。
     * @param network_io 网络 IO 处理器共享指针，用于输入输出处理。
     * @param motor_manager Motor 管理器共享指针，用于动作缩放等。
     * 初始化 TensorRT Runtime 和 Logger，确保异常安全。
     */
    TensorRTInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io, std::shared_ptr<MotorManager> motor_manager);

    /**
     * @brief 析构函数，释放 TensorRT 资源。
     * 确保引擎、运行时等智能指针自动释放。
     */
    virtual ~TensorRTInference();

protected:
    /**
     * @brief 加载或构建 TensorRT 模型（实现基类纯虚函数）。
     * @param model_path 模型路径（.onnx 或 .engine）。
     * 如果引擎文件存在，则反序列化加载；否则，从 ONNX 构建并保存引擎。
     * 支持 FP16 优化，验证输入/输出形状与配置匹配。
     */
    void LoadModel(const std::string &model_path) override;

    /**
     * @brief 执行推理（实现基类纯虚函数）。
     * @param inputs 准备好的输入列表（Eigen::MatrixXf）。
     * @return 输出映射，键为输出名称（如 "actions"），值为 Eigen::MatrixXf。
     * 转换输入为 CUDA 缓冲区，设置地址，执行 enqueueV3，提取输出。
     * 使用互斥锁确保线程安全。
     */
    std::map<std::string, Eigen::MatrixXf> Infer(const std::vector<Eigen::MatrixXf> &inputs) override;

private:
    // TensorRT 日志记录器类（继承 ILogger）
    class Logger : public ILogger
    {
    public:
        void log(Severity severity, const char *msg) noexcept override
        {
            if (severity <= Severity::kWARNING)
            {
                std::cout << "[TensorRT] " << msg << std::endl;
            }
        }
    };

    // TensorRT 核心对象（使用智能指针管理资源）
    std::unique_ptr<IRuntime> runtime_;                // TensorRT 运行时
    std::unique_ptr<ICudaEngine> engine_;              // TensorRT 引擎
    std::unique_ptr<IExecutionContext> context_;       // 执行上下文
    Logger logger_;                                    // 日志记录器实例

    // 输入/输出名称和形状（从引擎查询）
    std::vector<std::string> input_names_;             // 输入名称列表
    std::vector<std::string> output_names_;            // 输出名称列表
    std::vector<int64_t> obs_shape_;                   // obs 输入形状
    std::vector<int64_t> time_shape_;                  // time_step 输入形状

    std::vector<void *> input_buffers;
    bool input_first_flag;
    std::vector<void *> output_buffers;
    std::vector<size_t> output_sizes;

    // CUDA 流（用于异步推理）
    cudaStream_t stream_;

    // 辅助函数：将 Eigen 矩阵转换为 CUDA 缓冲区
    void *EigenToCudaBuffer(const Eigen::MatrixXf &matrix, size_t size);

    // 辅助函数：从 CUDA 缓冲区转换为 Eigen 矩阵
    Eigen::MatrixXf CudaBufferToEigen(void *buffer, size_t size);

    // 辅助函数：计算张量元素数
    size_t getTensorSize(const Dims &dims);
};

#endif // TENSORRT_INFERENCE_HPP