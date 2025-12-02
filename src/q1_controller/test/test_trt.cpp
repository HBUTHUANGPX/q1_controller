#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp> // ROS 2集成

using namespace nvinfer1;
using namespace nvonnxparser;

// 日志记录器
class Logger : public ILogger
{
  public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
        { // 调整为显示警告及以上
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
};

// 辅助函数：计算张量元素数
size_t getTensorSize(const Dims &dims)
{
    size_t size = 1;
    for (int i = 0; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
    }
    return size;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tensorrt_inference_node"); // 可扩展为ROS节点

    // 路径配置（替换为实际路径）
    std::string onnx_path = "/home/niic/RL_control/Q1_control/src/q1_controller/policy/2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500/policy.onnx";
    std::string engine_path = "/home/niic/RL_control/Q1_control/src/q1_controller/policy/model.engine"; // 引擎文件路径

    Logger logger;

    // 尝试加载现有引擎；如果不存在，则从ONNX构建
    std::unique_ptr<IRuntime> runtime(createInferRuntime(logger));
    std::unique_ptr<ICudaEngine> engine;
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (engine_file.good())
    {
        engine_file.seekg(0, std::ios::end);
        size_t size = engine_file.tellg();
        engine_file.seekg(0, std::ios::beg);
        std::vector<char> engine_data(size);
        engine_file.read(engine_data.data(), size);
        engine = std::unique_ptr<ICudaEngine>(runtime->deserializeCudaEngine(engine_data.data(), size));
        std::cout << "引擎加载成功。" << std::endl;
    }
    else
    {
        // 动态构建引擎（结合Python导出逻辑：固定形状，OpSet 11兼容）
        auto builder = std::unique_ptr<IBuilder>(createInferBuilder(logger));
        const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = std::unique_ptr<INetworkDefinition>(builder->createNetworkV2(explicitBatch));
        auto parser = std::unique_ptr<IParser>(createParser(*network, logger));
        if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(ILogger::Severity::kWARNING)))
        {
            std::cerr << "解析ONNX失败。" << std::endl;
            return 1;
        }

        auto config = std::unique_ptr<IBuilderConfig>(builder->createBuilderConfig());
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 1U << 30); // 1GB
        config->setFlag(BuilderFlag::kFP16);                              // FP16优化，适用于Jetson

        // 设置优化配置文件（固定批次1，结合Python中的示例输入形状）
        auto profile = builder->createOptimizationProfile();
        // obs: [1, N]，N从网络查询
        Dims obs_dims = network->getInput(0)->getDimensions(); // obs
        profile->setDimensions("obs", OptProfileSelector::kMIN, obs_dims);
        profile->setDimensions("obs", OptProfileSelector::kOPT, obs_dims);
        profile->setDimensions("obs", OptProfileSelector::kMAX, obs_dims);
        // time_step: [1,1]
        Dims time_dims{2, {1, 1}};
        profile->setDimensions("time_step", OptProfileSelector::kMIN, time_dims);
        profile->setDimensions("time_step", OptProfileSelector::kOPT, time_dims);
        profile->setDimensions("time_step", OptProfileSelector::kMAX, time_dims);
        config->addOptimizationProfile(profile);

        auto serialized_engine = std::unique_ptr<IHostMemory>(builder->buildSerializedNetwork(*network, *config));
        if (!serialized_engine)
        {
            std::cerr << "构建引擎失败。" << std::endl;
            return 1;
        }

        // 保存引擎
        std::ofstream new_engine_file(engine_path, std::ios::binary);
        new_engine_file.write(static_cast<char *>(serialized_engine->data()), serialized_engine->size());
        std::cout << "引擎构建并保存成功。" << std::endl;

        engine = std::unique_ptr<ICudaEngine>(
            runtime->deserializeCudaEngine(serialized_engine->data(), serialized_engine->size()));
    }

    if (!engine)
    {
        std::cerr << "引擎创建失败。" << std::endl;
        return 1;
    }

    // 检验模型：获取并验证IO信息（基于名称）
    int num_io_tensors = engine->getNbIOTensors();
    std::cout << "IO张量数量: " << num_io_tensors << " (预期9: 2输入 + 7输出)" << std::endl;
    std::vector<std::string> expected_inputs = {"obs", "time_step"};
    std::vector<std::string> expected_outputs = {"actions",     "joint_pos",      "joint_vel",     "body_pos_w",
                                                 "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"};

    std::vector<std::string> input_names, output_names;
    for (int i = 0; i < num_io_tensors; ++i)
    {
        std::string name = engine->getIOTensorName(i);
        Dims dims = engine->getTensorShape(name.c_str());
        DataType dtype = engine->getTensorDataType(name.c_str());
        bool is_input = engine->getTensorIOMode(name.c_str()) == TensorIOMode::kINPUT;

        std::cout << (is_input ? "输入" : "输出") << "张量 " << i << " 名称: " << name << ", 形状: [";
        for (int j = 0; j < dims.nbDims; ++j)
            std::cout << dims.d[j] << (j < dims.nbDims - 1 ? ", " : "");
        std::cout << "], 类型: " << static_cast<int>(dtype) << std::endl;

        if (is_input)
            input_names.push_back(name);
        else
            output_names.push_back(name);

        // 验证名称
        bool matched = false;
        if (is_input)
        {
            for (const auto &exp : expected_inputs)
                if (name == exp)
                    matched = true;
        }
        else
        {
            for (const auto &exp : expected_outputs)
                if (name == exp)
                    matched = true;
        }
        if (!matched)
        {
            std::cerr << "张量名称不匹配预期: " << name << std::endl;
            return 1;
        }
    }
    if (input_names.size() != 2 || output_names.size() != 7)
    {
        std::cerr << "输入/输出数量不匹配预期。" << std::endl;
        return 1;
    }
    std::cout << "模型检验通过。" << std::endl;

    // 创建执行上下文
    auto context = std::unique_ptr<IExecutionContext>(engine->createExecutionContext());
    if (!context)
    {
        std::cerr << "上下文创建失败。" << std::endl;
        return 1;
    }

    // 准备输入数据（示例：全零obs，time_step=0；结合Python clamp逻辑）
    std::cout << "准备输入数据。" << std::endl;
    Dims obs_dims = engine->getTensorShape("obs");
    size_t obs_size = getTensorSize(obs_dims);
    std::vector<float> obs_data(obs_size, 0.0f); // 示例数据

    float time_step_value = 0.0f; // 示例值
    // 主机端clamp（假设time_step_total从模型元数据获取；这里示例为67500，基于您的ONNX文件名）
    const int time_step_total = 67500; // 从Python time_step_total获取，或预设
    time_step_value = std::clamp(static_cast<int>(time_step_value), 0, time_step_total - 1);
    std::vector<float> time_data(1, time_step_value);

    // CUDA输入缓冲区
    std::cout << "建立CUDA输入缓冲区" << std::endl;
    float *d_obs;
    // cudaMalloc(&d_obs, obs_size * sizeof(float));
    cudaMalloc((void**)&d_obs, obs_size * sizeof(float));
    cudaMemcpy(d_obs, obs_data.data(), obs_size * sizeof(float), cudaMemcpyHostToDevice);
    float *d_time;
    // cudaMalloc(&d_time, sizeof(float));
    cudaMalloc((void**)&d_time, sizeof(float));
    cudaMemcpy(d_time, time_data.data(), sizeof(float), cudaMemcpyHostToDevice);

    // 设置输入地址
    context->setInputTensorAddress("obs", d_obs);
    context->setInputTensorAddress("time_step", d_time);

    std::cout << "建立CUDA输出缓冲区" << std::endl;
    // CUDA输出缓冲区（为所有7个输出分配）
    std::vector<float *> d_outputs(7);
    std::vector<std::vector<float>> host_outputs(7);
    for (size_t i = 0; i < expected_outputs.size(); ++i)
    {
        std::string out_name = expected_outputs[i];
        Dims out_dims = engine->getTensorShape(out_name.c_str());
        size_t out_size = getTensorSize(out_dims);
        cudaMalloc((void**)&d_outputs[i], out_size * sizeof(float));
        context->setOutputTensorAddress(out_name.c_str(), d_outputs[i]);
        host_outputs[i].resize(out_size);
    }

    std::cout << "执行推理" << std::endl;
    // 执行推理 (使用enqueueV3，异步)
    cudaStream_t stream;
    cudaStreamCreate(&stream);
    if (!context->enqueueV3(stream))
    {
        std::cerr << "推理执行失败。" << std::endl;
        cudaStreamDestroy(stream);
        return 1;
    }
    cudaStreamSynchronize(stream); // 同步以获取输出

    // 获取并打印输出（示例：打印actions的前5个值；类似处理其他输出）
    for (size_t i = 0; i < expected_outputs.size(); ++i)
    {
        size_t out_size = host_outputs[i].size();
        cudaMemcpy(host_outputs[i].data(), d_outputs[i], out_size * sizeof(float), cudaMemcpyDeviceToHost);
        std::cout << expected_outputs[i] << " 的前5个值: ";
        for (size_t j = 0; j < 5 && j < out_size; ++j)
        {
            std::cout << host_outputs[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "推理完成。" << std::endl;

    // 清理资源
    cudaFree(d_obs);
    cudaFree(d_time);
    for (auto &d_out : d_outputs)
        cudaFree(d_out);
    cudaStreamDestroy(stream);

    rclcpp::shutdown();
    return 0;
}