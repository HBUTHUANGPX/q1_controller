// 文件: data_store.cpp
#include "../../include/q1_controller/data_store.hpp"

DataStore::DataStore(const YAML::Node &config_, std::shared_ptr<MotionLoader> motion_loader)
    : motion_loader_(motion_loader)
{
    printf("DataStore: DataStore start init\r\n");
    std::cout << "motion time step total: " << motion_loader_->getTimeStepTotal() << std::endl;
    base_ang_vel_ = Eigen::VectorXf::Zero(3);
    cmd_vel_ = Eigen::VectorXf::Zero(3);
    if (!config_["joint_index_in_need"].IsSequence())
    {
        throw std::runtime_error("YAML 'joint_index_in_need' must be a sequence.");
    }
    size_t joint_index_in_need =config_["joint_index_in_need"].size();
    

    isaac_sim_trans_flag_ = config_["isaac_sim_trans_flag"].as<bool>();
    num_actions_ = config_["num_actions"].as<int>();
    last_actions_ = Eigen::VectorXf::Zero(num_actions_);
    joint_positions_ = Eigen::VectorXf::Zero(num_actions_);
    joint_velocities_ = Eigen::VectorXf::Zero(num_actions_);
    if (num_actions_ != joint_index_in_need)
    {
        throw std::runtime_error("action num in yaml mismatch the joint_index_in_need joint dim .");
    }

    mujoco2isaac_sim_index.reserve(num_actions_);
    if (isaac_sim_trans_flag_)
    {
        const YAML::Node &seq = config_["mujoco2isaac_sim_index"];
        if (!seq.IsSequence())
        {
            throw std::runtime_error("Node for key mujoco2isaac_sim_index is not a sequence.");
        }
        std::cout << "mujoco2isaac_sim_index: [";
        for (const auto &item : seq)
        {
            mujoco2isaac_sim_index.push_back(item.as<int>());
            std::cout << item.as<int>() << " ";
        }
        std::cout << "]" << std::endl;
        printf("InferenceBase: isaac_sim_trans_flag is true\r\n");
    }
    else
    {
        std::cout << "mujoco2isaac_sim_index: [";
        for (size_t i = 0; i < num_actions_; i++)
        {
            mujoco2isaac_sim_index.push_back(i);
            std::cout << i << " ";
        }
        std::cout << "]" << std::endl;
        printf("InferenceBase: isaac_sim_trans_flag is false\r\n");
    }

    scale_vx = config_["gamepad_cmd_vel_scale_vx"].as<float>();
    scale_vy = config_["gamepad_cmd_vel_scale_vy"].as<float>();
    scale_wz = config_["gamepad_cmd_vel_scale_wz"].as<float>();
    gamepad_cmd_vel = Eigen::VectorXf::Zero(3);
    gamepad_cmd_vel << scale_vx, scale_vy, scale_wz;
}

void DataStore::UpdateJointPositions(const Eigen::VectorXf &positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    joint_positions_ = positions;
}

void DataStore::UpdateJointVelocities(const Eigen::VectorXf &velocities)
{
    std::lock_guard<std::mutex> lock(mutex_);
    joint_velocities_ = velocities;
}

void DataStore::UpdateBaseAngularVelocities(const Eigen::VectorXf &ang_vel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    base_ang_vel_ = ang_vel;
}

void DataStore::UpdateLastActions(const Eigen::VectorXf &actions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_actions_ = actions;
}

void DataStore::UpdateRobotQuat(const Eigen::Quaternionf &robot_quat)
{
    std::lock_guard<std::mutex> lock(mutex_);
    robot_quat_ = robot_quat;
}

void DataStore::UpdateCmdVel(const Eigen::VectorXf &cmd_Vel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_vel_ = cmd_Vel.cwiseProduct(gamepad_cmd_vel);
}

Eigen::VectorXf DataStore::GetCmdVel() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return cmd_vel_;
}

Eigen::Vector3f DataStore::get_gravity_orientation() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    // float qw = 1+0*robot_quat_.w();
    // float qx = 0*robot_quat_.x();
    // float qy = 0*robot_quat_.y();
    // float qz = 0*robot_quat_.z();
    float qw = robot_quat_.w();
    float qx = robot_quat_.x();
    float qy = robot_quat_.y();
    float qz = robot_quat_.z();

    Eigen::Vector3f gravity_orientation;
    gravity_orientation(0) = 2.f * (-qz * qx + qw * qy);
    gravity_orientation(1) = -2.f * (qz * qy + qw * qx);
    gravity_orientation(2) = 1.f - 2.f * (qw * qw + qz * qz);

    return gravity_orientation;
}

Eigen::VectorXf DataStore::GetJointPositions() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::VectorXf new_joint_positions_ = joint_positions_;
    // std::cout << "new_joint_positions_: " << current_joint_positions_.transpose() << std::endl;
    for (size_t i = 0; i < num_actions_; i++)
    {
        new_joint_positions_(i) = joint_positions_(mujoco2isaac_sim_index[i]);
    }

    // std::cout << "GetJointPositions: " << new_joint_positions_.transpose() << std::endl;
    return new_joint_positions_;
}

Eigen::VectorXf DataStore::GetJointVelocities() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::VectorXf new_joint_velocities_ = joint_velocities_;
    for (size_t i = 0; i < num_actions_; i++)
    {
        new_joint_velocities_(i) = joint_velocities_(mujoco2isaac_sim_index[i]);
    }

    return new_joint_velocities_;
}

Eigen::VectorXf DataStore::GetBaseAngularVelocities() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return base_ang_vel_;
}

Eigen::VectorXf DataStore::GetLastActions() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_actions_;
}

Eigen::VectorXf DataStore::GetMotionJointPosCommand(float time_step) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_loader_)
    {
        throw std::runtime_error("GetMotionJointPosCommand !motion_loader_.");
        return Eigen::VectorXf::Zero(joint_positions_.size()); // 未加载，返回零向量
    }
    size_t index = static_cast<int>(time_step); // 将 time_step 转为 int 作为索引
    if (index < 0 || index >= motion_loader_->getTimeStepTotal())
    {
        throw std::runtime_error("GetMotionJointPosCommand time_step is out of mocap dataset.");
        return Eigen::VectorXf::Zero(joint_positions_.size()); // 索引无效，返回零向量
    }

    return motion_loader_->getJointPos(index); // 返回指定行的 VectorXf
}

Eigen::VectorXf DataStore::GetMotionJointVelCommand(float time_step) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!motion_loader_)
    {
        throw std::runtime_error("GetMotionJointVelCommand !motion_loader_.");
        return Eigen::VectorXf::Zero(joint_velocities_.size()); // 未加载，返回零向量
    }
    size_t index = static_cast<int>(time_step); // 将 time_step 转为 int 作为索引
    if (index < 0 || index >= motion_loader_->getTimeStepTotal())
    {
        throw std::runtime_error("GetMotionJointVelCommand time_step is out of mocap dataset.");
        return Eigen::VectorXf::Zero(joint_velocities_.size()); // 索引无效，返回零向量
    }

    return motion_loader_->getJointVel(index); // 返回指定行的 VectorXf
}

Eigen::Matrix3f DataStore::ComputeMotionRefOriMatrix(float time_step) const
{ // Q1的imu是放在torsorlink的
    // std::lock_guard<std::mutex> lock(mutex_);

    if (!motion_loader_)
    {
        printf("DataStore: !motion_loader_\r\n");
        return Eigen::Matrix3f::Zero(); // 未加载，返回零矩阵
    }

    int index = static_cast<int>(time_step); // 将 time_step 转为 int 作为索引
    auto ref_body_quats = motion_loader_->getBodyQuatW(index);
    Eigen::VectorXf joint_angle_ = joint_positions_;
    // std::cout << "new_joint_positions_: " << current_joint_positions_.transpose() << std::endl;
    for (size_t i = 0; i < num_actions_; i++)
    {
        joint_angle_(i) = joint_positions_(mujoco2isaac_sim_index[i]);
    }
    Eigen::AngleAxisf joint_rotation(joint_angle_(3), Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf joint_quat(joint_rotation);
    Eigen::Quaternionf robot_torsor_quat_ = robot_quat_ * joint_quat;  // torsor姿态 = 父link姿态 * 关节旋转

    Eigen::Vector4f ref_quat_vec = ref_body_quats.row(7);
    Eigen::Quaternionf ref_quat(ref_quat_vec(0), ref_quat_vec(1), ref_quat_vec(2), ref_quat_vec(3));
    Eigen::Matrix3f robot_rot = robot_torsor_quat_.inverse().toRotationMatrix();
    Eigen::Matrix3f ref_rot = ref_quat.toRotationMatrix();
    return robot_rot * ref_rot; 
}