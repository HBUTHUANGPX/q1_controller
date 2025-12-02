// 文件: motion_joint_pos_command.hpp
#ifndef MOTION_JOINT_POS_COMMAND_HPP
#define MOTION_JOINT_POS_COMMAND_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>
/**
 * @brief 运动关节位置命令观测组件子类。
 *
 * 从动捕数据集或ROS话题中获取期望关节位置命令，并应用缩放。
 * 初始化使用YAML配置，避免硬编码。
 */
class MotionJointPosCommand : public ObservationComponent
{
  public:
    /**
     * @brief 构造函数，从YAML读取配置。
     * @param dim 维度（len）。
     * @param scale 缩放因子。
     */
    MotionJointPosCommand(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "motion_joint_pos_command", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        // 占位符：从动捕数据集或ROS话题获取数据（例如从motion.joint_pos[time_step]）
        Eigen::VectorXf data = data_store_->GetMotionJointPosCommand(time_step); // 替换为实际获取逻辑
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            std::cout << "MotionJointPosCommand data size: "<<data.size() << std::endl;
            throw std::runtime_error("Data size mismatch for motion_joint_pos_command.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
};

#endif // MOTION_JOINT_POS_COMMAND_HPP