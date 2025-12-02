// 文件: openvino_inference.cpp
#include "../../include/q1_controller/openvino_inference.hpp"

#include <iostream>  // 用于调试输出（可选）
#include <stdexcept> // 用于 std::runtime_error

OpenVINOInference::OpenVINOInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,
                                     std::shared_ptr<MotorManager> motor_manager)
    : InferenceBase(config, network_io, motor_manager)
{
    // printf("OpenVINOInference: construct function init\r\n");
    LoadModel(policy_path_);
    // printf("OpenVINOInference: construct function init ok\r\n");
}

void OpenVINOInference::LoadModel(const std::string &model_path)
{
    model_ = core_.read_model(model_path);

    // 获取动态名称和形状
    const auto &input_names = network_io_->GetInputNames();
    const auto &output_names = network_io_->GetOutputNames();
    const auto &config_input_shapes = network_io_->GetInputShapes();
    const auto &config_output_shapes = network_io_->GetOutputShapes();

    // 验证输入数量
    if (model_->inputs().size() != input_names.size())
    {
        throw std::runtime_error("输入数量不匹配: 模型有 " + std::to_string(model_->inputs().size()) +
                                 " 个输入，但 YAML 配置有 " + std::to_string(input_names.size()) + " 个。");
    }

    // 循环验证每个输入
    for (size_t i = 0; i < input_names.size(); ++i)
    {
        std::string name = input_names[i];
        auto input = model_->input(name);
        auto ps = input.get_partial_shape();
        std::cout << "模型输入 " << name << " 形状: " << ps.to_string() << std::endl;

        // 验证 rank
        if (static_cast<size_t>(ps.rank().get_length()) != config_input_shapes[i].size())
        {
            throw std::runtime_error("输入 " + name + " rank 不匹配: 模型为 " + std::to_string(ps.rank().get_length()) +
                                     "，配置为 " + std::to_string(config_input_shapes[i].size()) + "。");
        }

        // 验证维度（忽略动态 -1）
        for (size_t j = 0; j < config_input_shapes[i].size(); ++j)
        {
            int64_t config_dim = static_cast<int64_t>(config_input_shapes[i](j));
            auto model_dim = ps[j];
            if (model_dim.is_static() && model_dim.get_length() != config_dim && config_dim >= 0)
            {
                throw std::runtime_error("输入 " + name + " 维度 " + std::to_string(j) + " 不匹配: 配置为 " +
                                         std::to_string(config_dim) + "，模型为 " +
                                         std::to_string(model_dim.get_length()) + "。");
            }
        }
    }

    // 验证输出数量
    if (model_->outputs().size() != output_names.size())
    {
        std::cout << "WARNING: 输出数量不匹配: 模型有 " << model_->outputs().size() << " 个输出，但 YAML 配置有 "
                  << output_names.size() << " 个。请检查模型。" << std::endl;
    }

    // 循环验证每个输出
    for (size_t i = 0; i < output_names.size(); ++i)
    {
        std::string name = output_names[i];
        auto output = model_->output(name);
        auto ps = output.get_partial_shape();
        std::cout << "模型输出 " << name << " 形状: " << ps.to_string() << std::endl;

        // 验证 rank
        if (static_cast<size_t>(ps.rank().get_length()) != config_output_shapes[i].size())
        {
            throw std::runtime_error("输出 " + name + " rank 不匹配: 模型为 " + std::to_string(ps.rank().get_length()) +
                                     "，配置为 " + std::to_string(config_output_shapes[i].size()) + "。");
        }

        // 验证维度（忽略动态 -1）
        for (size_t j = 0; j < config_output_shapes[i].size(); ++j)
        {
            int64_t config_dim = static_cast<int64_t>(config_output_shapes[i](j));
            auto model_dim = ps[j];
            if (model_dim.is_static() && model_dim.get_length() != config_dim && config_dim >= 0)
            {
                throw std::runtime_error("输出 " + name + " 维度 " + std::to_string(j) + " 不匹配: 配置为 " +
                                         std::to_string(config_dim) + "，模型为 " +
                                         std::to_string(model_dim.get_length()) + "。");
            }
        }
    }

    // 动态 reshape（支持动态维度，使用配置形状替换 -1）
    std::map<std::string, ov::PartialShape> reshape_map;
    for (size_t i = 0; i < input_names.size(); ++i)
    {
        std::string name = input_names[i];
        std::vector<ov::Dimension> dims;
        for (Eigen::Index j = 0; j < config_input_shapes[i].size(); ++j)
        {
            dims.emplace_back(static_cast<int64_t>(config_input_shapes[i](j)));
        }
        reshape_map[name] = ov::PartialShape(dims);
    }
    model_->reshape(reshape_map);

    model_->validate_nodes_and_infer_types();

    compiled_model_ = core_.compile_model(model_, "CPU");
    infer_request_ = compiled_model_.create_infer_request();

    // printf("OpenVINOInference: LoadModel function start \r\n");
    model_ = core_.read_model(model_path);
#if 0
    // 获取配置的输入/输出形状
    const auto &_input_shapes = network_io_->GetInputShapes();
    const auto &_output_shapes = network_io_->GetOutputShapes();

    // 校验输入和输出形状
    // printf("OpenVINOInference: Validating model shapes...\r\n");

    // 校验输入: obs
    auto obs_input = model_->input("obs");
    auto obs_ps = obs_input.get_partial_shape();
    // printf("OpenVINOInference: Model obs shape: %s\r\n", obs_ps.to_string().c_str());
    if (obs_ps.rank().get_length() != 2 || obs_ps[0] != 1)
    {
        throw std::runtime_error("Invalid model obs input shape: expected rank 2 with batch=1.");
    }
    if (obs_ps[1].is_static())
    {
        int64_t model_obs_dim = obs_ps[1].get_length();
        int64_t config_obs_dim = static_cast<int64_t>(_input_shapes[0][1]);
        if (model_obs_dim != config_obs_dim)
        {
            throw std::runtime_error("Observation dimension mismatch: configured " + std::to_string(config_obs_dim) +
                                     ", but model expects " + std::to_string(model_obs_dim) + ". Check YAML config.");
        }
    }

    // 校验输入: time_step
    auto time_input = model_->input("time_step");
    auto time_ps = time_input.get_partial_shape();
    // printf("OpenVINOInference: Model time_step shape: %s\r\n", time_ps.to_string().c_str());
    if (time_ps.rank().get_length() != 2 || time_ps[0] != 1)
    {
        throw std::runtime_error("Invalid model time_step input shape: expected rank 2 with batch=1.");
    }
    if (time_ps[1].is_static())
    {
        int64_t model_time_dim = time_ps[1].get_length();
        int64_t config_time_dim = static_cast<int64_t>(_input_shapes[1][1]);
        if (model_time_dim != config_time_dim)
        {
            throw std::runtime_error("Time_step dimension mismatch: configured " + std::to_string(config_time_dim) +
                                     ", but model expects " + std::to_string(model_time_dim) + ".");
        }
    }

    // 校验输出: action (假设单一输出)
    if (model_->get_output_size() != 1)
    {
        printf("WARNING: Unexpected number of outputs: expected 1 for actions.Please "
               "check your model,If it meets your "
               "expectations, please ignore this warning message\r\n");
    }
    auto action_output = model_->output(0);
    auto action_ps = action_output.get_partial_shape();
    // printf("OpenVINOInference: Model action shape: %s\r\n", action_ps.to_string().c_str());
    if (action_ps.rank().get_length() != 2 || action_ps[0] != 1)
    {
        throw std::runtime_error("Invalid model action output shape: expected rank 2 with batch=1.");
    }
    if (action_ps[1].is_static())
    {
        int64_t model_act_dim = action_ps[1].get_length();
        int64_t config_act_dim = static_cast<int64_t>(_output_shapes[0][1]);
        if (model_act_dim != config_act_dim)
        {
            throw std::runtime_error("Action dimension mismatch: configured " + std::to_string(config_act_dim) +
                                     ", but model expects " + std::to_string(model_act_dim) + ". Check model outputs.");
        }
    }

    std::map<std::string, ov::PartialShape> reshape_map;
    // printf("OpenVINOInference: input_shapes \r\n");
    const auto &input_shapes = network_io_->GetInputShapes();
    std::vector<ov::Dimension> obs_dims;
    for (Eigen::Index i = 0; i < input_shapes[0].size(); ++i)
    {
        obs_dims.emplace_back(static_cast<int64_t>(input_shapes[0](i)));
    }
    reshape_map["obs"] = ov::PartialShape(obs_dims);

    // printf("OpenVINOInference: time_dims \r\n");
    std::vector<ov::Dimension> time_dims;
    for (Eigen::Index i = 0; i < input_shapes[1].size(); ++i)
    {
        time_dims.emplace_back(static_cast<int64_t>(input_shapes[1](i)));
    }
    reshape_map["time_step"] = ov::PartialShape(time_dims);
    // printf("OpenVINOInference: reshape_map \r\n");
    model_->reshape(reshape_map);
    // printf("OpenVINOInference: Validating nodes and inferring types...\r\n");
    model_->validate_nodes_and_infer_types();
    // printf("OpenVINOInference: Node validation passed.\r\n");
    // printf("OpenVINOInference: compile_model \r\n");
    compiled_model_ = core_.compile_model(model_, "CPU");
    infer_request_ = compiled_model_.create_infer_request();
    // printf("OpenVINOInference: LoadModel function process ok \r\n");
#endif
}

std::map<std::string, Eigen::MatrixXf> OpenVINOInference::Infer(const std::vector<Eigen::MatrixXf> &inputs)
{
    const auto &input_names = network_io_->GetInputNames();
    if (inputs.size() != input_names.size())
    {
        throw std::runtime_error("输入数量不匹配。");
    }

    // 设置输入（使用名称查找 port）
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        std::string name = input_names[i];
        auto port = compiled_model_.input(name);
        ov::Tensor tensor = EigenToOVTensor(inputs[i], port.get_shape());
        infer_request_.set_tensor(port, tensor);
    }

    // 执行推理
    infer_request_.infer();

    // 获取输出（动态提取所有输出）
    std::map<std::string, Eigen::MatrixXf> outputs;
    const auto &output_names = network_io_->GetOutputNames();
    for (const auto &name : output_names)
    {
        auto port = compiled_model_.output(name);
        ov::Tensor tensor = infer_request_.get_tensor(port);
        outputs[name] = OVTensorToEigen(tensor);
    }
    return outputs;
}

ov::Tensor OpenVINOInference::EigenToOVTensor(const Eigen::MatrixXf &matrix, const ov::Shape &shape)
{
    ov::Tensor tensor(ov::element::f32, shape);
    float *data = tensor.data<float>();
    Eigen::Map<Eigen::MatrixXf>(data, matrix.rows(), matrix.cols()) = matrix.cast<float>();
    return tensor;
}

Eigen::MatrixXf OpenVINOInference::OVTensorToEigen(const ov::Tensor &tensor)
{
    const float *data = tensor.data<const float>();
    ov::Shape shape = tensor.get_shape();
    if (shape.empty())
    {
        throw std::runtime_error("无效的输出形状: 空形状。");
    }

    // batch 维度（第一个维度）
    Eigen::Index batch = static_cast<Eigen::Index>(shape[0]);

    // 计算剩余维度的乘积作为 flattened columns
    Eigen::Index flattened = 1;
    for (size_t i = 1; i < shape.size(); ++i)
    {
        flattened *= static_cast<Eigen::Index>(shape[i]);
    }

    // 映射为 MatrixXf
    Eigen::MatrixXf result(batch, flattened);
    std::copy(data, data + batch * flattened, result.data());
    return result;
}