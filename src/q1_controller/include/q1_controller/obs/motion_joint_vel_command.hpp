// 文件: motion_joint_vel_command.hpp
#ifndef MOTION_JOINT_VEL_COMMAND_HPP
#define MOTION_JOINT_VEL_COMMAND_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 运动关节速度命令观测组件子类。
 *
 * 从动捕数据集或ROS话题中获取期望关节速度命令，并应用缩放。
 */
class MotionJointVelCommand : public ObservationComponent
{
  public:
    MotionJointVelCommand(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "motion_joint_vel_command", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetMotionJointVelCommand(time_step);
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for motion_joint_vel_command.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
};

#endif // MOTION_JOINT_VEL_COMMAND_HPP