// 文件: pin_mj.cpp
#include "../../include/q1_controller/pin_mj.hpp"
// #include <pinocchio/algorithm/joints.hpp> // 关节操作
#include <stdexcept>

/**
 * @brief PinMj 构造函数实现。
 */
PinMj::PinMj(const std::string &urdf_path)
    : base_pos_world_(Eigen::Vector3d::Zero()), base_quat_world_(Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0))
{
    // 加载 URDF 模型，支持浮动基
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
}

/**
 * @brief mujocoToPinocchio 函数实现。
 */
Eigen::VectorXf PinMj::mujocoToPinocchio(const Eigen::VectorXf &joint_angles, const Eigen::Vector3d &base_pos,
                                         const Eigen::Quaternionf &base_quat)
{
    Eigen::VectorXf q = Eigen::VectorXf::Zero(model_.nq); // 广义坐标 q [7 + nJoints] 如果浮动基

    if (model_.joints[1].shortname() == "JointModelFreeFlyer")
    {
        // 浮动基：前 3 维位置，后 4 维四元数 (Pinocchio 顺序: xyz, qwxyz，但输入是 xyzw)
        q.head<3>() = base_pos;
        q.segment<4>(3) =
            Eigen::Vector4f(base_quat.x(), base_quat.y(), base_quat.z(), base_quat.w()); // 转换为 Pinocchio 顺序
        q.tail(model_.nq - 7) = joint_angles;
    }
    else
    {
        // 固定基：q 全部为关节角度
        q = joint_angles;
    }

    // 执行前向运动学
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);

    return q;
}

/**
 * @brief getLinkQuaternion 函数实现。
 */
Eigen::Quaternionf PinMj::getLinkQuaternion(const std::string &link_name) const
{
    pinocchio::FrameIndex frame_id = model_.getFrameId(link_name);
    if (frame_id == static_cast<pinocchio::FrameIndex>(-1))
    {
        throw std::runtime_error("Link not found: " + link_name);
    }

    // 获取旋转矩阵，并转换为四元数 (scalar_first: xyzw 顺序)
    const pinocchio::SE3 &oMf = data_->oMf[frame_id];
    Eigen::Matrix3f rotation = oMf.rotation();
    Eigen::Quaternionf quat(rotation);
    quat.normalize(); // 确保单位四元数

    return quat;
}