// 文件: observation_manager.cpp
#include "../../include/q1_controller/observation_manager.hpp"
#include <iostream>
#include <stdexcept>
ObservationManager::ObservationManager(const YAML::Node &config,std::shared_ptr<DataStore> data_store)
{
    if (!config["obs"].IsSequence())
    {
        throw std::runtime_error("YAML 'obs' must be a sequence.");
    }
    else
    {
        std::cout << "obs read ok" << std::endl;
    }

    int offset = 0;
    
    for (const auto &item : config["obs"])
    {
        // 检查item是否为Map且仅有一个键值对
        if (!item.IsMap() || item.size() != 1)
        {
            throw std::runtime_error("Each 'obs' item must be a map with exactly one key-value pair.");
        }

        // 获取Map的第一个（唯一）键值对迭代器
        auto map_iter = item.begin();
        std::string name = map_iter->first.as<std::string>(); // 键作为名称
        YAML::Node params = map_iter->second;                 // 值作为参数节点

        int dim = params["len"].as<int>();
        float scale = params["scale"].as<float>();
        std::cout << name << ": " << dim << ", " << scale << std::endl;

        std::shared_ptr<ObservationComponent> comp;
        if (name == "motion_joint_pos_command")
        {
            comp = std::make_shared<MotionJointPosCommand>(dim, scale, data_store);
        }
        else if (name == "motion_joint_vel_command")
        {
            comp = std::make_shared<MotionJointVelCommand>(dim, scale, data_store);
        }
        else if (name == "motion_ref_ori_b")
        {
            comp = std::make_shared<MotionRefOriB>(dim, scale, data_store);
        }
        else if (name == "base_ang_vel")
        {
            comp = std::make_shared<BaseAngVel>(dim, scale, data_store);
        }
        else if (name == "joint_pos")
        {
            comp = std::make_shared<JointPos>(dim, scale, data_store);
        }
        else if (name == "joint_vel")
        {
            comp = std::make_shared<JointVel>(dim, scale, data_store);
        }
        else if (name == "last_actions")
        {
            comp = std::make_shared<LastActions>(dim, scale, data_store);
        }
        else if (name == "gravity_orientation")
        {
            comp = std::make_shared<GravityOrientation>(dim, scale, data_store);
        }
        else if (name == "sin_cos")
        {
            comp = std::make_shared<SinCos>(dim, scale, data_store);
        }
        else if (name == "cmd_vel")
        {
            comp = std::make_shared<CmdVel>(dim, scale, data_store);
        }
        else
        {
            throw std::runtime_error("Unknown obs component: " + name);
        }

        comp->SetOffset(offset);
        offset += dim;
        components_.push_back(comp);
    }
    total_dim_ = offset;
}
Eigen::MatrixXf ObservationManager::UpdateObservations(float time_step)
{
    Eigen::MatrixXf obs = Eigen::MatrixXf::Zero(1, total_dim_);
    for (auto &comp : components_)
    {
        // std::cout << "components: "<<comp->GetName() << std::endl;
        comp->Update(obs, time_step);
    }
    obs = obs.cwiseMin(10.0f).cwiseMax(-10.0f);
    return obs;
}