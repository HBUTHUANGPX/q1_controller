// 文件: onnx_inference.cpp
#include "../../include/q1_controller/onnx_inference.hpp"

#include <iostream>  // 用于调试输出（可选，可移除以减少日志）。
#include <stdexcept> // 用于 std::runtime_error 异常处理。
#include <vector>    // 用于形状向量和数据复制。

/**
 * @brief 构造函数，调用基类构造函数并加载模型。
 * 初始化 ONNX Runtime 环境（日志级别为警告，以减少输出）。
 * 基于onnxruntime_cxx_inline.h中的Env构造函数，确保异常安全。
 */
OnnxInference::OnnxInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,
                             std::shared_ptr<MotorManager> motor_manager)
    : InferenceBase(config, network_io, motor_manager),
      env_(ORT_LOGGING_LEVEL_WARNING, "OnnxInference"), // 初始化环境，设置日志级别为警告（匹配头文件Env定义）。
      session_(env_, policy_path_.c_str(), session_options_)
{
    // 加载模型，使用基类中的 policy_path_。
    session_options_.SetIntraOpNumThreads(1); // 单线程推理，适合简单场景。
    LoadModel(policy_path_);
}

/**
 * @brief 加载 ONNX 模型文件。
 * 创建会话选项和会话，验证模型的输入/输出形状与配置匹配。
 * 如果形状不匹配，抛出Ort::Exception异常以确保模型兼容性。
 * 基于onnxruntime_cxx_inline.h中的ThrowOnError，确保所有API调用异常安全。
 * @param model_path 模型路径（如 "policy.onnx"）。
 */
void OnnxInference::LoadModel(const std::string &model_path)
{
    // 获取动态名称和形状
    const auto &input_names = network_io_->GetInputNames();
    const auto &output_names = network_io_->GetOutputNames();
    const auto &config_input_shapes = network_io_->GetInputShapes();
    const auto &config_output_shapes = network_io_->GetOutputShapes();

    // 检验模型：获取输入节点信息
    inputNodeCount = session_.GetInputCount();
    Ort::AllocatorWithDefaultOptions allocator;

    // 输入节点名称列表
    for (size_t i = 0; i < inputNodeCount; ++i)
    {
        inputNames.push_back(std::move(session_.GetInputNameAllocated(i, allocator)));
        inputNodeNames.push_back(inputNames.back().get());
        std::cout << "输入节点 " << i << " 名称: " << inputNodeNames[i] << std::endl;

        // 获取输入类型和形状
        Ort::TypeInfo inputTypeInfo = session_.GetInputTypeInfo(i);
        auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        ONNXTensorElementDataType inputDataType = inputTensorInfo.GetElementType();
        std::vector<int64_t> inputShape = inputTensorInfo.GetShape();

        std::cout << "输入节点 " << i << " 数据类型: " << inputDataType << " (1: FLOAT)" << std::endl;
        std::cout << "输入节点 " << i << " 形状: ";
        for (auto dim : inputShape)
        {
            std::cout << dim << " ";
        }
        std::cout << std::endl;
    }

    // 检验模型：获取输出节点信息
    outputNodeCount = session_.GetOutputCount();
    std::cout << "输出节点数量: " << outputNodeCount << std::endl;

    // 输出节点名称列表
    for (size_t i = 0; i < outputNodeCount; ++i)
    {
        outputNames.push_back(std::move(session_.GetOutputNameAllocated(i, allocator)));
        outputNodeNames.push_back(outputNames.back().get());
        std::cout << "输出节点 " << i << " 名称: " << outputNodeNames[i] << std::endl;

        // 获取输出类型和形状
        Ort::TypeInfo outputTypeInfo = session_.GetOutputTypeInfo(i);
        auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
        ONNXTensorElementDataType outputDataType = outputTensorInfo.GetElementType();
        std::vector<int64_t> outputShape = outputTensorInfo.GetShape();

        std::cout << "输出节点 " << i << " 数据类型: " << outputDataType << " (1: FLOAT)" << std::endl;
        std::cout << "输出节点 " << i << " 形状: ";
        for (auto dim : outputShape)
        {
            std::cout << dim << " ";
        }
        std::cout << std::endl;
    }

    // 验证名称和形状
    if (inputNodeCount != input_names.size())
    {
        throw std::runtime_error("输入数量不匹配。");
    }
    for (size_t i = 0; i < inputNodeCount; ++i)
    {
        if (strcmp(inputNodeNames[i], input_names[i].c_str()) != 0)
        {
            throw std::runtime_error("输入名称不匹配: " + std::string(inputNodeNames[i]));
        }
        // 验证形状 (类似处理输出)
        auto model_shape = session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        if (model_shape.size() != config_input_shapes[i].size())
        {
            throw std::runtime_error("输入形状不匹配。");
        }
        for (size_t j = 0; j < model_shape.size(); j++)
        {
            std::cout << "model_shape: "<<model_shape[j] << std::endl;
        }
        
        input_shapes_.push_back(model_shape);
    }
    if (outputNodeCount != output_names.size())
    {
        throw std::runtime_error("输出数量不匹配。");
    }

    for (size_t i = 0; i < outputNodeCount; ++i)
    {
        if (strcmp(outputNodeNames[i], output_names[i].c_str()) != 0)
        {
            throw std::runtime_error("输出名称不匹配: " + std::string(outputNodeNames[i]));
        }
        // 验证形状 (类似处理输出)
        auto model_shape = session_.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        if (model_shape.size() != config_output_shapes[i].size())
        {
            throw std::runtime_error("输出形状不匹配。");
        }
        output_shapes_.push_back(model_shape);
    }
    std::cout << "模型输入输出检验通过。" << std::endl;
    // 准备输入形状（ONNX 需要 int64_t 向量，匹配GetShape）。
    obs_shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    time_shape = {1, 1};
}

/**
 * @brief 执行推理过程。
 * 将输入转换为 ONNX Tensor，运行会话，提取输出并转换为 Eigen 矩阵。
 * 只提取 "actions" 输出，其他输出忽略。
 * 基于onnxruntime_cxx_inline.h中的Run调用，确保输入/输出名称为const char*数组。
 * @param inputs 输入列表（0: obs, 1: time_step）。
 * @return 输出映射，键为 "actions"，值为 Eigen::MatrixXf。
 */
std::map<std::string, Eigen::MatrixXf> OnnxInference::Infer(const std::vector<Eigen::MatrixXf> &inputs)
{
    if (inputs.size() != inputNodeCount)
    {
        std::cout << inputs.size() <<" "<< inputNodeCount << std::endl;
        ORT_CXX_API_THROW("Input count mismatch for ONNX inference.", ORT_INVALID_ARGUMENT);
    }
    // 在循环外部初始化数据缓冲区向量
    std::vector<std::vector<float>> input_datas(inputs.size());
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> input_tensors;
    // 循环计算大小、分配缓冲区并复制数据
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        size_t size = 1;
        for (auto dim : input_shapes_[i])
        {
            size *= dim;
            std::cout << size <<" "<< dim << std::endl;

        }

        // 验证 Eigen 输入大小匹配
        if (static_cast<size_t>(inputs[i].size()) != size)
        {
            std::cout << static_cast<size_t>(inputs[i].size()) <<" "<< size << std::endl;
            ORT_CXX_API_THROW("Eigen input size does not match ONNX shape.", ORT_INVALID_ARGUMENT);
        }
        

        // 分配并初始化缓冲区
        input_datas[i] = std::vector<float>(size, 0.0f);

        // 高效复制 Eigen 数据（假设行优先存储）
        const float *input_ptr = inputs[i].data();
        std::copy(input_ptr, input_ptr + size, input_datas[i].begin());
        input_tensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, input_datas[i].data(),
                                                                input_datas[i].size(), input_shapes_[i].data(),
                                                                input_shapes_[i].size()));
    }
    std::vector<Ort::Value> output_tensors =
        session_.Run(Ort::RunOptions{nullptr}, inputNodeNames.data(), input_tensors.data(), inputNodeCount,
                     outputNodeNames.data(), outputNodeCount);

    std::map<std::string, Eigen::MatrixXf> outputs;
    const auto &output_names = network_io_->GetOutputNames();
    if (!output_tensors.empty())
    {
        for (size_t i = 0; i < output_names.size(); i++)
        {
            outputs[output_names[i]] = OrtValueToEigen(output_tensors[i]);
        }
    }
    else
    {
        ORT_CXX_API_THROW("No outputs from ONNX inference.", ORT_FAIL);
    }
    return outputs;
}

/**
 * @brief 将 ONNX Ort::Value 转换为 Eigen 矩阵。
 * 假设 Tensor 为 float32，形状为 [1, features]。
 * 使用GetTensorData<float>获取数据，并复制到Eigen矩阵（确保数据拷贝安全）。
 * @param value 输入 Ort::Value。
 * @return 转换后的 Eigen::MatrixXf (行向量)。
 */
Eigen::MatrixXf OnnxInference::OrtValueToEigen(const Ort::Value &value)
{
    if (!value.IsTensor())
    {
        ORT_CXX_API_THROW("ONNX output is not a tensor.", ORT_INVALID_ARGUMENT);
    }

    auto type_shape = value.GetTensorTypeAndShapeInfo();
    auto shape = type_shape.GetShape();
    if (shape.empty() || shape[0] != 1)
    {
        ORT_CXX_API_THROW("Invalid ONNX output shape: expected batch size 1 as first dimension.", ORT_INVALID_ARGUMENT);
    }

    // const float *data = value.GetTensorData<float>(); // 获取const数据指针（匹配GetTensorData）。
    // Eigen::MatrixXf result(1, static_cast<Eigen::Index>(shape[1]));
    // std::copy(data, data + shape[1], result.data()); // 拷贝数据到Eigen。
    // 计算展平后的列数（剩余维度的乘积）
    Eigen::Index rows = static_cast<Eigen::Index>(shape[0]); // 批次大小
    Eigen::Index cols = 1;
    for (size_t i = 1; i < shape.size(); ++i)
    {
        cols *= static_cast<Eigen::Index>(shape[i]);
    }
    const float *data = value.GetTensorData<float>();
    Eigen::MatrixXf result(rows, cols);
    std::copy(data, data + rows * cols, result.data());
    return result;
}