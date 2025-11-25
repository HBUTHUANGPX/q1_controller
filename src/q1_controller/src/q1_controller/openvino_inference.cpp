// 文件: openvino_inference.cpp
#include "../../include/q1_controller/openvino_inference.hpp"

#include <iostream>  // 用于调试输出（可选）
#include <stdexcept> // 用于 std::runtime_error

OpenVINOInference::OpenVINOInference(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,std::shared_ptr<MotorManager> motor_manager)
    : InferenceBase(config, network_io,motor_manager)
{
    // printf("OpenVINOInference: construct function init\r\n");
    LoadModel(policy_path_);
    // printf("OpenVINOInference: construct function init ok\r\n");
}

void OpenVINOInference::LoadModel(const std::string &model_path)
{
    // printf("OpenVINOInference: LoadModel function start \r\n");
    model_ = core_.read_model(model_path);

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
}

std::map<std::string, Eigen::MatrixXf> OpenVINOInference::Infer(const std::vector<Eigen::MatrixXf> &inputs)
{
    // printf("OpenVINOInference: get input_ports \r\n");
    auto input_ports = compiled_model_.inputs();
    if (input_ports.size() != inputs.size())
    {
        throw std::runtime_error("Input count mismatch.");
    }

    // 设置输入
    // printf("OpenVINOInference: set input tensor \r\n");
    for (size_t i = 0; i < inputs.size(); ++i)
    {
        ov::Tensor tensor = EigenToOVTensor(inputs[i], input_ports[i].get_shape());
        infer_request_.set_tensor(input_ports[i], tensor);
    }
    // printf("OpenVINOInference: infer \r\n");
    // 执行推理
    infer_request_.infer();

    // 获取输出
    // printf("OpenVINOInference: get outputs \r\n");
    auto output_ports = compiled_model_.outputs();
    std::map<std::string, Eigen::MatrixXf> outputs;
    // printf("OpenVINOInference: get outputs tensor \r\n");
    for (const auto &port : output_ports)
    {
        std::string name = port.get_any_name();
        ov::Tensor tensor = infer_request_.get_tensor(port);
        outputs[name] = OVTensorToEigen(tensor);
    }
    // printf("OpenVINOInference: Infer has done \r\n");

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
    if (shape.size() < 2)
    {
        throw std::runtime_error("Tensor rank must be at least 2 (batch dimension first).");
    }

    // 计算行数（batch，通常为1）和列数（剩余维度的乘积）
    Eigen::Index rows = static_cast<Eigen::Index>(shape[0]);
    Eigen::Index cols = 1;
    for (size_t i = 1; i < shape.size(); ++i)
    {
        cols *= static_cast<Eigen::Index>(shape[i]);
    }

    // 映射为MatrixXf并转换为MatrixXf
    Eigen::MatrixXf result = Eigen::Map<const Eigen::MatrixXf>(data, rows, cols).cast<float>();
    return result;
}