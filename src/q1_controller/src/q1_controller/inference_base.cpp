// 文件: inference_base.cpp
#include "../../include/q1_controller/inference_base.hpp"

InferenceBase::InferenceBase(const YAML::Node &config, std::shared_ptr<NetworkIOBase> network_io,
                             std::shared_ptr<MotorManager> motor_manager)
    : network_io_(network_io)
{
    config_ = config;

    // 加载通用参数（忽略 unitree 特定部分）
    policy_path_ = config_["policy_path"].as<std::string>();
    num_actions_ = config_["num_actions"].as<int>();
    action_scale_.reserve(num_actions_);
    float as = config_["action_scale"].as<float>();
    for (size_t i = 0; i < num_actions_; i++)
    {
        action_scale_.push_back(as);
    }
    action_limit_ = config_["action_limit"].as<float>();

    isaac_sim_trans_flag_ = config_["isaac_sim_trans_flag"].as<bool>();
    if (isaac_sim_trans_flag_)
    {
        const YAML::Node &seq = config_["isaac_sim2mujoco_index"];
        if (!seq.IsSequence())
        {
            throw std::runtime_error("Node for key isaac_sim2mujoco_index is not a sequence.");
        }
        isaac_sim2mujoco_index.reserve(seq.size());
        std::cout << "isaac_sim2mujoco_index: [";
        for (const auto &item : seq)
        {
            isaac_sim2mujoco_index.push_back(item.as<int>());
            std::cout << item.as<int>() << " ";
        }
        std::cout << "]" << std::endl;
        printf("InferenceBase: isaac_sim_trans_flag is true\r\n");
    }
    else
    {
        isaac_sim2mujoco_index.reserve(num_actions_);
        std::cout << "isaac_sim2mujoco_index: [";
        for (size_t i = 0; i < num_actions_; i++)
        {
            isaac_sim2mujoco_index.push_back(i);
            std::cout << i << " ";
        }
        std::cout << "]" << std::endl;
        printf("InferenceBase: isaac_sim_trans_flag is false\r\n");
    }

    recal_action_scale_by_tq_max_and_kp = config_["recal_action_scale_by_tq_max_and_kp"].as<bool>();

    if (recal_action_scale_by_tq_max_and_kp)
    {
        UpdateActionScale(motor_manager);
    }
    std::cout << "action_scale: [";
    for (size_t i = 0; i < num_actions_; i++)
    {
        std::cout << action_scale_[i] << " ";
    }
    std::cout << "]" << std::endl;

    // 初始化状态（对于 LSTM）
    states_ = {};
}

Eigen::VectorXf InferenceBase::UpdateAction(const Eigen::MatrixXf &observations, float time_step)
{
    rclcpp::Logger logger_(rclcpp::get_logger("InferenceBase"));
    std::lock_guard<std::mutex> lock(infer_mutex_);
    // RCLCPP_INFO(logger_, "PrepareInputs");
    auto inputs = network_io_->PrepareInputs(observations, states_, time_step);
    // RCLCPP_INFO(logger_, "Infer");
    auto raw_outputs = Infer(inputs);
    // RCLCPP_INFO(logger_, "ExtractOutputs");
    auto [actions, new_states] = network_io_->ExtractOutputs(raw_outputs);
    // RCLCPP_INFO(logger_, "clamp");
    states_ = new_states;
    return actions;
}

Eigen::VectorXf InferenceBase::Scale_and_clamp_Action(const Eigen::VectorXf &actions)
{
    // printf("InferenceBase: clamped action\r\n");
    Eigen::VectorXf new_actions = actions;
    for (int i = 0; i < num_actions_; ++i)
    {
        new_actions(i) = std::clamp(std::clamp(actions(isaac_sim2mujoco_index[i]), -static_cast<float>(action_limit_),
                                               static_cast<float>(action_limit_)) *
                                        action_scale_[i],
                                    -static_cast<float>(action_limit_), static_cast<float>(action_limit_));
        // printf("%4.2f ", new_actions(i));
    }
    // printf("\r\n");
    return new_actions;
}

void InferenceBase::UpdateActionScale(std::shared_ptr<MotorManager> motor_manager)
{
    for (size_t i = 0; i < action_scale_.size(); i++)
    {
        std::shared_ptr<MotorBase> motor = motor_manager->getMotorByIndex(i);
        action_scale_[i] *= (motor->getMaxTorque() / motor->getKp());
    }
}
void InferenceBase::ResetMemory()
{
    for (auto &state : states_)
    {
        state.setZero();
    }
}