// 文件: tensorrt_inference.cpp
#include "../../include/q1_controller/tensorrt_inference.hpp"

#include <fstream>   // 文件 IO
#include <iostream>  // 调试输出
#include <stdexcept> // 异常处理
#include <vector>    // 形状和数据向量
using namespace nvinfer1;
using namespace nvonnxparser;
/**
 * @brief 构造函数，调用基类构造函数并加载模型。
 * 初始化 TensorRT Runtime 和 Logger，创建 CUDA 流。
 * 如果构建失败，抛出 std::runtime_error。
 */
TensorRTInference::TensorRTInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,
                                     std::shared_ptr<MotorManager> motor_manager)
    : InferenceBase(config, network_io, motor_manager), input_first_flag(true)
{
    // 初始化 TensorRT Runtime
    runtime_ = std::unique_ptr<IRuntime>(createInferRuntime(logger_));
    if (!runtime_)
    {
        throw std::runtime_error("TensorRTInference: 创建 Runtime 失败。");
    }

    // 创建 CUDA 流
    if (cudaStreamCreate(&stream_) != cudaSuccess)
    {
        throw std::runtime_error("TensorRTInference: 创建 CUDA 流失败。");
    }

    // 加载模型
    LoadModel(policy_path_);
}

/**
 * @brief 析构函数，释放资源。
 * 销毁 CUDA 流，确保无内存泄漏。
 */
TensorRTInference::~TensorRTInference()
{   
    // 清理缓冲区
    for (auto buf : input_buffers)
        cudaFree(buf);
    for (auto buf : output_buffers)
        cudaFree(buf);
    cudaStreamDestroy(stream_);
}

/**
 * @brief 加载或构建 TensorRT 模型。
 * 如果引擎文件存在，则加载；否则，从 ONNX 构建并保存。
 * 支持 FP16 优化，验证输入/输出名称和形状。
 * @param model_path 模型路径（.onnx 或 .engine）。
 */
void TensorRTInference::LoadModel(const std::string &model_path)
{
    // 尝试加载引擎文件
    std::string engine_path = model_path.substr(0, model_path.find_last_of('.')) + ".engine";
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (engine_file.good())
    {
        engine_file.seekg(0, std::ios::end);
        size_t size = engine_file.tellg();
        engine_file.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        engine_file.read(engine_data.data(), size);
        engine_ = std::unique_ptr<ICudaEngine>(runtime_->deserializeCudaEngine(engine_data.data(), size));
        if (!engine_)
        {
            throw std::runtime_error("TensorRTInference: 引擎反序列化失败。");
        }
        std::cout << "TensorRTInference: 引擎加载成功。" << std::endl;
    }
    else
    {
        // 从 ONNX 构建引擎
        auto builder = std::unique_ptr<IBuilder>(createInferBuilder(logger_));
        const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = std::unique_ptr<INetworkDefinition>(builder->createNetworkV2(explicitBatch));
        auto parser = std::unique_ptr<IParser>(createParser(*network, logger_));
        if (!parser->parseFromFile(model_path.c_str(), static_cast<int>(ILogger::Severity::kWARNING)))
        {
            throw std::runtime_error("TensorRTInference: 解析 ONNX 失败。");
        }

        auto config = std::unique_ptr<IBuilderConfig>(builder->createBuilderConfig());
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1U << 30); // 1GB 工作空间
        config->setFlag(BuilderFlag::kFP16);                              // 启用 FP16 优化

        // 设置优化配置文件
        auto profile = builder->createOptimizationProfile();
        Dims obs_dims = network->getInput(0)->getDimensions();
        profile->setDimensions("obs", OptProfileSelector::kMIN, obs_dims);
        profile->setDimensions("obs", OptProfileSelector::kOPT, obs_dims);
        profile->setDimensions("obs", OptProfileSelector::kMAX, obs_dims);
        Dims time_dims{2, {1, 1}};
        profile->setDimensions("time_step", OptProfileSelector::kMIN, time_dims);
        profile->setDimensions("time_step", OptProfileSelector::kOPT, time_dims);
        profile->setDimensions("time_step", OptProfileSelector::kMAX, time_dims);
        config->addOptimizationProfile(profile);

        auto serialized_engine = std::unique_ptr<IHostMemory>(builder->buildSerializedNetwork(*network, *config));
        if (!serialized_engine)
        {
            throw std::runtime_error("TensorRTInference: 构建序列化引擎失败。");
        }

        // 保存引擎
        std::ofstream new_engine_file(engine_path, std::ios::binary);
        new_engine_file.write(static_cast<char *>(serialized_engine->data()), serialized_engine->size());
        std::cout << "TensorRTInference: 引擎构建并保存成功。" << std::endl;

        engine_ = std::unique_ptr<ICudaEngine>(
            runtime_->deserializeCudaEngine(serialized_engine->data(), serialized_engine->size()));
    }

    // 创建执行上下文
    context_ = std::unique_ptr<IExecutionContext>(engine_->createExecutionContext());
    if (!context_)
    {
        throw std::runtime_error("TensorRTInference: 创建执行上下文失败。");
    }

    // 查询并存储输入/输出名称
    int num_io_tensors = engine_->getNbIOTensors();
    for (int i = 0; i < num_io_tensors; ++i)
    {
        std::string name = engine_->getIOTensorName(i);
        if (engine_->getTensorIOMode(name.c_str()) == TensorIOMode::kINPUT)
        {
            input_names_.push_back(name);
        }
        else
        {
            output_names_.push_back(name);
        }
    }

    // 验证输入/输出数量
    if (input_names_.size() != 2 || output_names_.size() != 7)
    {
        throw std::runtime_error("TensorRTInference: 输入/输出数量不匹配预期（2输入 + 7输出）。");
    }

    // 存储输入形状
    obs_shape_ = std::vector<int64_t>(engine_->getTensorShape("obs").d,
                                      engine_->getTensorShape("obs").d + engine_->getTensorShape("obs").nbDims);
    time_shape_ = {1, 1};

    rclcpp::Logger logger_(rclcpp::get_logger("TensorRTInference"));

    // RCLCPP_INFO(logger_, "准备 CUDA 输出缓冲区");
    // // 准备 CUDA 输出缓冲区
    // output_buffers = std::vector<void *>(output_names_.size());
    // output_sizes = std::vector<size_t>(output_names_.size());
    // for (size_t i = 0; i < output_names_.size(); ++i)
    // {
    //     std::string name = output_names_[i];
    //     Dims dims = engine_->getTensorShape(name.c_str());
    //     output_sizes[i] = getTensorSize(dims) * sizeof(float);
    //     cudaMalloc(&output_buffers[i], output_sizes[i]);
    //     context_->setOutputTensorAddress(name.c_str(), output_buffers[i]);
    // }
}

/**
 * @brief 执行推理过程。
 * 将输入转换为 CUDA 缓冲区，设置地址，执行 enqueueV3，提取输出并转换为 Eigen 矩阵。
 * 只提取 "actions" 输出，其他输出忽略。
 * @param inputs 输入列表（0: obs, 1: time_step）。
 * @return 输出映射，键为 "actions"，值为 Eigen::MatrixXf。
 */
std::map<std::string, Eigen::MatrixXf> TensorRTInference::Infer(const std::vector<Eigen::MatrixXf> &inputs)
{
    // std::lock_guard<std::mutex> lock(infer_mutex_);

    if (inputs.size() != input_names_.size())
    {
        throw std::runtime_error("TensorRTInference: 输入数量不匹配。");
    }

    rclcpp::Logger logger_(rclcpp::get_logger("TensorRTInference"));
    // if (input_first_flag)
    // {
    //     // 准备 CUDA 输入缓冲区
    //     RCLCPP_INFO(logger_, "准备 CUDA 输入缓冲区");
    //     input_buffers = std::vector<void *>(inputs.size());
    //     for (size_t i = 0; i < inputs.size(); ++i)
    //     {
    //         std::string name = input_names_[i];
    //         size_t size = getTensorSize(engine_->getTensorShape(name.c_str())) * sizeof(float);
    //         input_buffers[i] = EigenToCudaBuffer(inputs[i], size);
    //         context_->setInputTensorAddress(name.c_str(), input_buffers[i]);
    //     }
    //     input_first_flag = false;
    // }
    // else
    // {   
    // }
    input_buffers = std::vector<void *>(inputs.size());
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        std::string name = input_names_[i];
        size_t size = getTensorSize(engine_->getTensorShape(name.c_str())) * sizeof(float);
        input_buffers[i] = EigenToCudaBuffer(inputs[i], size);
        context_->setInputTensorAddress(name.c_str(), input_buffers[i]);
    }
    // RCLCPP_INFO(logger_, "准备 CUDA 输出缓冲区");
    // 准备 CUDA 输出缓冲区
    output_buffers = std::vector<void *>(output_names_.size());
    output_sizes = std::vector<size_t>(output_names_.size());
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
        std::string name = output_names_[i];
        Dims dims = engine_->getTensorShape(name.c_str());
        output_sizes[i] = getTensorSize(dims) * sizeof(float);
        cudaMalloc(&output_buffers[i], output_sizes[i]);
        context_->setOutputTensorAddress(name.c_str(), output_buffers[i]);
    }
    // 执行推理
    // RCLCPP_INFO(logger_, "执行推理");
    if (!context_->enqueueV3(stream_))
    {
        throw std::runtime_error("TensorRTInference: enqueueV3 推理失败。");
    }
    cudaStreamSynchronize(stream_);

    // RCLCPP_INFO(logger_, "提取输出");
    // 提取输出（只取 "actions"）
    std::map<std::string, Eigen::MatrixXf> outputs;
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
        std::string name = output_names_[i];
        if (name == "actions")
        {
            outputs[name] = CudaBufferToEigen(output_buffers[i], output_sizes[i] / sizeof(float));
            // std::cout << "Infer: " <<outputs[name] << std::endl;
        }
        // 可扩展提取其他输出
    }

    // RCLCPP_INFO(logger_, "清理缓冲区");
    // 清理缓冲区
    for (auto buf : input_buffers)
        cudaFree(buf);
    for (auto buf : output_buffers)
        cudaFree(buf);

    return outputs;
}

/**
 * @brief 将 Eigen 矩阵转换为 CUDA 缓冲区。
 * 分配 CUDA 内存并复制数据。
 * @param matrix 输入 Eigen 矩阵。
 * @param size 缓冲区大小（字节）。
 * @return CUDA 缓冲区指针。
 */
void *TensorRTInference::EigenToCudaBuffer(const Eigen::MatrixXf &matrix, size_t size)
{
    void *buffer;
    cudaMalloc(&buffer, size);
    cudaMemcpy(buffer, matrix.data(), size, cudaMemcpyHostToDevice);
    return buffer;
}

/**
 * @brief 从 CUDA 缓冲区转换为 Eigen 矩阵。
 * 复制数据到主机并映射到 Eigen。
 * @param buffer CUDA 缓冲区指针。
 * @param num_elements 元素数量。
 * @return Eigen 矩阵。
 */
Eigen::MatrixXf TensorRTInference::CudaBufferToEigen(void *buffer, size_t num_elements)
{
    std::vector<float> host_data(num_elements);
    cudaMemcpy(host_data.data(), buffer, num_elements * sizeof(float), cudaMemcpyDeviceToHost);
    return Eigen::Map<Eigen::MatrixXf>(host_data.data(), 1, num_elements);
}

/**
 * @brief 计算张量元素数。
 * @param dims 张量形状。
 * @return 元素总数。
 */
size_t TensorRTInference::getTensorSize(const Dims &dims)
{
    size_t size = 1;
    for (int i = 0; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
    }
    return size;
}