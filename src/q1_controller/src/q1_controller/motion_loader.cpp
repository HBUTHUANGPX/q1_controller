// 文件: motion_loader.cpp
#include "../../include/q1_controller/motion_loader.hpp"
#include <stdexcept>

/**
 * @brief MotionLoader 构造函数实现。
 * @param config_ YAML 配置节点，包含 motion_file 和 body_indexes。
 */
MotionLoader::MotionLoader(const YAML::Node &config_)
{
    std::string mocap_path_;
    mocap_path_ = config_["mocap_path"].as<std::string>();
    loadData(mocap_path_);
    const YAML::Node &seq = config_["motion_body_index"];
    if (!seq.IsSequence())
    {
        throw std::runtime_error("Node for key motion_body_index is not a sequence.");
    }
    body_indexes_.reserve(seq.size());
    std::cout << "motion_body_index: [";
    for (const auto &item : seq)
    {
        body_indexes_.push_back(item.as<int>());
        std::cout << item.as<int>() << " ";
    }
    std::cout << "]" << std::endl;
}

/**
 * @brief 加载 NPZ 数据实现。
 * @param motion_file NPZ 文件路径。
 */
void MotionLoader::loadData(const std::string &motion_file)
{
    cnpy::npz_t data = cnpy::npz_load(motion_file);
    if (data.empty())
    {
        throw std::runtime_error("Failed to load NPZ file: " + motion_file);
    }

    // 加载 FPS (scalar)
    auto &fps_np = data["fps"];
    if (fps_np.num_vals != 1)
    {
        throw std::runtime_error("FPS does not have exactly one value.");
    }
    fps_ = fps_np.data<float>()[0]; // 直接访问第一个元素

    // 加载 joint_pos [时间, 29]
    const auto &joint_pos_np = data["joint_pos"];
    if (joint_pos_np.shape.size() != 2 || joint_pos_np.shape[1] != 29)
    {
        throw std::runtime_error("joint_pos is not 2D or has unexpected columns (expected 29).");
    }
    const size_t time_steps = joint_pos_np.shape[0];
    const size_t joint_dims = joint_pos_np.shape[1]; // 29
    joint_pos_.resize(joint_dims, time_steps);
    const float *joint_pos_data = joint_pos_np.data<float>();
    for (size_t t = 0; t < time_steps; ++t)
    {
        for (size_t j = 0; j < joint_dims; ++j)
        {
            joint_pos_(j, t) = joint_pos_data[t * joint_dims + j];
        }
    }

    const auto &joint_vel_np = data["joint_vel"];
    if (joint_vel_np.shape.size() != 2 || joint_vel_np.shape[1] != 29)
    {
        throw std::runtime_error("joint_vel is not 2D or has unexpected columns (expected 29).");
    }
    joint_vel_.resize(joint_dims, time_steps);
    const float *joint_vel_data = joint_vel_np.data<float>();
    for (size_t t = 0; t < time_steps; ++t)
    {
        for (size_t j = 0; j < joint_dims; ++j)
        {
            joint_vel_(j, t) = joint_vel_data[t * joint_dims + j];
        }
    }

    // 加载 body_pos_w [时间步, 30, 3]
    const auto &body_pos_w_np = data["body_pos_w"];
    if (body_pos_w_np.shape.size() != 3 || body_pos_w_np.shape[1] != 30 || body_pos_w_np.shape[2] != 3)
    {
        throw std::runtime_error("body_pos_w is not 3D or has unexpected shape (expected [time, 30, 3]).");
    }
    time_step_total_ = body_pos_w_np.shape[0];
    const size_t body_num = body_pos_w_np.shape[1]; // 30
    size_t dim = body_pos_w_np.shape[2];            // 3
    body_pos_w_.resize(time_step_total_);
    const float *body_pos_w_data = body_pos_w_np.data<float>();
    for (size_t t = 0; t < time_step_total_; ++t)
    {
        body_pos_w_[t].resize(dim, body_num); // 3 x 30
        for (size_t b = 0; b < body_num; ++b)
        {
            for (size_t d = 0; d < dim; ++d)
            {
                body_pos_w_[t](d, b) = body_pos_w_data[t * (body_num * dim) + b * dim + d];
            }
        }
    }
    std::cout << body_pos_w_[0].col(0).transpose() << std::endl;
    std::cout << body_pos_w_[1].col(0).transpose() << std::endl;
    std::cout << body_pos_w_[2].col(0).transpose() << std::endl;
    // 加载 body_quat_w [时间步, 30, 4]
    const auto &body_quat_w_np = data["body_quat_w"];
    if (body_quat_w_np.shape.size() != 3 || body_quat_w_np.shape[1] != 30 || body_quat_w_np.shape[2] != 4)
    {
        throw std::runtime_error("body_quat_w is not 3D or has unexpected shape (expected [time, 30, 4]).");
    }
    body_quat_w_.resize(time_step_total_);
    dim = body_quat_w_np.shape[2]; // 4
    const float *body_quat_w_data = body_quat_w_np.data<float>();
    for (size_t t = 0; t < time_step_total_; ++t)
    {
        body_quat_w_[t].resize(dim, body_num); // 4 x 30
        for (size_t b = 0; b < body_num; ++b)
        {
            for (size_t d = 0; d < dim; ++d)
            {
                body_quat_w_[t](d, b) = body_quat_w_data[t * (body_num * dim) + b * dim + d];
            }
        }
    }
    // 加载 body_lin_vel_w [时间步, 30, 3]
    const auto &body_lin_vel_w_np = data["body_lin_vel_w"];
    if (body_lin_vel_w_np.shape.size() != 3 || body_lin_vel_w_np.shape[1] != 30 || body_lin_vel_w_np.shape[2] != 3)
    {
        throw std::runtime_error("body_lin_vel_w is not 3D or has unexpected shape (expected [time, 30, 3]).");
    }
    body_lin_vel_w_.resize(time_step_total_);
    dim = body_lin_vel_w_np.shape[2]; // 3
    const float *body_lin_vel_w_data = body_lin_vel_w_np.data<float>();
    for (size_t t = 0; t < time_step_total_; ++t)
    {
        body_lin_vel_w_[t].resize(dim, body_num); // 3 x 30
        for (size_t b = 0; b < body_num; ++b)
        {
            for (size_t d = 0; d < dim; ++d)
            {
                body_lin_vel_w_[t](d, b) = body_lin_vel_w_data[t * (body_num * dim) + b * dim + d];
            }
        }
    }

    // 加载 body_ang_vel_w [时间步, 30, 3]
    const auto &body_ang_vel_w_np = data["body_ang_vel_w"];
    if (body_ang_vel_w_np.shape.size() != 3 || body_ang_vel_w_np.shape[1] != 30 || body_ang_vel_w_np.shape[2] != 3)
    {
        throw std::runtime_error("body_ang_vel_w is not 3D or has unexpected shape (expected [time, 30, 3]).");
    }
    body_ang_vel_w_.resize(time_step_total_);
    dim = body_ang_vel_w_np.shape[2]; // 3
    const float *body_ang_vel_w_data = body_ang_vel_w_np.data<float>();
    for (size_t t = 0; t < time_step_total_; ++t)
    {
        body_ang_vel_w_[t].resize(dim, body_num); // 3 x 30
        for (size_t b = 0; b < body_num; ++b)
        {
            for (size_t d = 0; d < dim; ++d)
            {
                body_ang_vel_w_[t](d, b) = body_ang_vel_w_data[t * (body_num * dim) + b * dim + d];
            }
        }
    }
}

/**
 * @brief 获取 body_pos_w 子集。
 * @param index 时间步索引。
 * @return body_pos_w 子集数据。
 */
Eigen::MatrixXf MotionLoader::getBodyPosW(size_t index) const
{
    if (index < 0 || index >= time_step_total_)
    {
        throw std::runtime_error("time_step is out of mocap dataset.");
        return Eigen::MatrixXf(body_indexes_.size(), 3); // 索引无效，返回零矩阵
    }
    Eigen::MatrixXf subset = Eigen::MatrixXf(body_indexes_.size(), 3);
    for (size_t i = 0; i < body_indexes_.size(); ++i)
    {
        subset.row(i) = body_pos_w_[index].col(body_indexes_[i]).transpose();
    }
    // std::cout << "body_pos_w_[" << index
    //           << "].col(body_indexes_[0]).transpose(): " << body_pos_w_[index].col(body_indexes_[0]).transpose()
    //           << std::endl;

    // std::cout << "subset.row(0)" << subset.row(0) << std::endl;
    return subset;
}

/**
 * @brief 获取 body_quat_w 子集。
 * @param index 时间步索引。
 * @return body_quat_w 子集数据。
 */
Eigen::MatrixXf MotionLoader::getBodyQuatW(size_t index) const
{
    if (index < 0 || index >= time_step_total_)
    {
        throw std::runtime_error("time_step is out of mocap dataset.");
        return Eigen::MatrixXf(body_indexes_.size(), 4); // 索引无效，返回零矩阵
    }
    Eigen::MatrixXf subset = Eigen::MatrixXf(body_indexes_.size(), 4);
    for (size_t i = 0; i < body_indexes_.size(); ++i)
    {
        subset.row(i) = body_quat_w_[index].col(body_indexes_[i]).transpose();
    }
    return subset;
}

/**
 * @brief 获取 body_lin_vel_w 子集。
 * @param index 时间步索引。
 * @return body_lin_vel_w 子集数据。
 */
Eigen::MatrixXf MotionLoader::getBodyLinVelW(size_t index) const
{
    if (index < 0 || index >= time_step_total_)
    {
        throw std::runtime_error("time_step is out of mocap dataset.");
        return Eigen::MatrixXf(body_indexes_.size(), 3); // 索引无效，返回零矩阵
    }
    Eigen::MatrixXf subset = Eigen::MatrixXf(body_indexes_.size(), 3);
    for (size_t i = 0; i < body_indexes_.size(); ++i)
    {
        subset.row(i) = body_lin_vel_w_[index].col(body_indexes_[i]).transpose();
    }
    return subset;
}

/**
 * @brief 获取 body_ang_vel_w 子集。
 * @param index 时间步索引。
 * @return body_ang_vel_w 子集数据。
 */
Eigen::MatrixXf MotionLoader::getBodyAngVelW(size_t index) const
{
    if (index < 0 || index >= time_step_total_)
    {
        throw std::runtime_error("time_step is out of mocap dataset.");
        return Eigen::MatrixXf(body_indexes_.size(), 3); // 索引无效，返回零矩阵
    }
    Eigen::MatrixXf subset = Eigen::MatrixXf(body_indexes_.size(), 3);
    for (size_t i = 0; i < body_indexes_.size(); ++i)
    {
        subset.row(i) = body_ang_vel_w_[index].col(body_indexes_[i]).transpose();
    }
    return subset;
}