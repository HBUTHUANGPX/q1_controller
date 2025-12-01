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
    TensorRTInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io, std::shared_ptr<MotorManager> motor_manager);
    virtual ~TensorRTInference();

protected:
    void LoadModel(const std::string &model_path) override;
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
    // std::vector<int64_t> obs_shape_;                   // obs 输入形状
    // std::vector<int64_t> time_shape_;                  // time_step 输入形状

    std::vector<void *> input_buffers;
    std::vector<void *> output_buffers;
    std::vector<size_t> output_sizes;

    // CUDA 流（用于异步推理）
    cudaStream_t stream_;

    // 辅助函数：将 Eigen 矩阵转换为 CUDA 缓冲区
    void *EigenToCudaBuffer(const Eigen::MatrixXf &matrix, size_t size);

    // 辅助函数：从 CUDA 缓冲区转换为 Eigen 矩阵
    Eigen::MatrixXf CudaBufferToEigen(void *buffer, size_t size, Eigen::Index batch = 1);

    // 辅助函数：计算张量元素数
    // size_t getTensorSize(const Dims &dims);
    size_t getTensorSize(const Dims &dims, Eigen::Index batch = 1);
};

#endif // TENSORRT_INFERENCE_HPP