#include <array>
#include <cstring> // std::strcmp
#include <iostream>
#include <memory> // std::shared_ptr
#include <onnxruntime_cxx_api.h>
#include <vector>

using namespace std;

int main()
{
    // 模型路径
    std::string model_path = "/home/hpx/HPX_LOCO_2/whole_body_tracking/deploy_mujoco/deploy_policy/Q1/"
                             "2025-10-31_21-45-24_Q1_251021_03_walk_120Hz_67500.onnx";

    // 创建 ONNX Runtime 环境
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXModel");

    // 创建会话选项
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1); // 设置线程数，可根据需要调整
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // 加载模型并创建会话（读取和初始化模型）
    Ort::Session session(env, model_path.c_str(), sessionOptions);
    cout << "模型加载成功。" << endl;

    // 检验模型：获取输入节点信息
    size_t inputNodeCount = session.GetInputCount();
    cout << "输入节点数量: " << inputNodeCount << endl;

    Ort::AllocatorWithDefaultOptions allocator;

    // 输入节点名称列表
    vector<std::shared_ptr<char>> inputNames;
    vector<char *> inputNodeNames;
    for (size_t i = 0; i < inputNodeCount; ++i)
    {
        inputNames.push_back(std::move(session.GetInputNameAllocated(i, allocator)));
        inputNodeNames.push_back(inputNames.back().get());
        cout << "输入节点 " << i << " 名称: " << inputNodeNames[i] << endl;

        // 获取输入类型和形状
        Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(i);
        auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        ONNXTensorElementDataType inputDataType = inputTensorInfo.GetElementType();
        vector<int64_t> inputShape = inputTensorInfo.GetShape();

        cout << "输入节点 " << i << " 数据类型: " << inputDataType << " (1: FLOAT)" << endl;
        cout << "输入节点 " << i << " 形状: ";
        for (auto dim : inputShape)
        {
            cout << dim << " ";
        }
        cout << endl;
    }

    // 检验模型：获取输出节点信息
    size_t outputNodeCount = session.GetOutputCount();
    cout << "输出节点数量: " << outputNodeCount << endl;

    // 输出节点名称列表
    vector<std::shared_ptr<char>> outputNames;
    vector<char *> outputNodeNames;
    for (size_t i = 0; i < outputNodeCount; ++i)
    {
        outputNames.push_back(std::move(session.GetOutputNameAllocated(i, allocator)));
        outputNodeNames.push_back(outputNames.back().get());
        cout << "输出节点 " << i << " 名称: " << outputNodeNames[i] << endl;

        // 获取输出类型和形状
        Ort::TypeInfo outputTypeInfo = session.GetOutputTypeInfo(i);
        auto outputTensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
        ONNXTensorElementDataType outputDataType = outputTensorInfo.GetElementType();
        vector<int64_t> outputShape = outputTensorInfo.GetShape();

        cout << "输出节点 " << i << " 数据类型: " << outputDataType << " (1: FLOAT)" << endl;
        cout << "输出节点 " << i << " 形状: ";
        for (auto dim : outputShape)
        {
            cout << dim << " ";
        }
        cout << endl;
    }

    // 模型检验：验证输入输出名称是否匹配预期（基于 Python 导出代码）
    if (inputNodeCount != 2 || strcmp(inputNodeNames[0], "obs") != 0 || strcmp(inputNodeNames[1], "time_step") != 0)
    {
        cout << "输入节点不匹配预期。" << endl;
        return 1;
    }
    if (outputNodeCount != 7 || strcmp(outputNodeNames[0], "actions") != 0 ||
        strcmp(outputNodeNames[1], "joint_pos") != 0 || strcmp(outputNodeNames[2], "joint_vel") != 0 ||
        strcmp(outputNodeNames[3], "body_pos_w") != 0 || strcmp(outputNodeNames[4], "body_quat_w") != 0 ||
        strcmp(outputNodeNames[5], "body_lin_vel_w") != 0 || strcmp(outputNodeNames[6], "body_ang_vel_w") != 0)
    {
        cout << "输出节点不匹配预期。" << endl;
        return 1;
    }
    cout << "模型输入输出检验通过。" << endl;

    // 准备输入数据（推理阶段）
    // 假设 obs 的形状为 [1, N]，其中 N 来自模型形状（需替换为实际维度，例如从 inputShape[1] 获取）
    // 这里使用示例数据：全零初始化，实际使用时需替换为真实观测数据
    vector<int64_t> obsShape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    size_t obsSize = 1;
    for (auto dim : obsShape)
    {
        obsSize *= dim;
    }
    vector<float> obsData(obsSize, 0.0f); // 示例：全零观测

    // time_step 的形状为 [1, 1]，示例值为 0
    vector<int64_t> timeStepShape = {1, 1};
    vector<float> timeStepData = {0.0f}; // 示例时间步

    // 创建输入张量
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    vector<Ort::Value> inputTensors;
    inputTensors.push_back(
        Ort::Value::CreateTensor<float>(memoryInfo, obsData.data(), obsData.size(), obsShape.data(), obsShape.size()));
    inputTensors.push_back(Ort::Value::CreateTensor<float>(memoryInfo, timeStepData.data(), timeStepData.size(),
                                                           timeStepShape.data(), timeStepShape.size()));

    // 执行推理
    vector<Ort::Value> outputTensors = session.Run(Ort::RunOptions{nullptr}, inputNodeNames.data(), inputTensors.data(),
                                                   inputNodeCount, outputNodeNames.data(), outputNodeCount);

    // 处理输出（示例：打印第一个输出 actions 的前几个值）
    cout << "推理完成。示例输出（actions 的前 5 个值）: ";
    float *actionsData = outputTensors[0].GetTensorMutableData<float>();
    for (int i = 0; i < 5 && i < outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount(); ++i)
    {
        cout << actionsData[i] << " ";
    }
    cout << endl;

    // 类似地，可处理其他输出：joint_pos 等
    // 注意：实际应用中需根据输出形状提取数据

    getchar(); // 暂停以查看输出
    return 0;
}