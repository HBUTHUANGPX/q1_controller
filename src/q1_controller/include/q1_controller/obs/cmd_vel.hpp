// 文件: cmd_vel.hpp
#ifndef CMD_VEL_HPP
#define CMD_VEL_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 基座角速度观测组件子类。
 *
 * 从IMU或ROS话题获取基座角速度。
 */
class CmdVel : public ObservationComponent
{
  public:
    CmdVel(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "cmd_vel", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetCmdVel(); // 从IMU话题获取
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for base_ang_vel.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
    
};

#endif // BASE_ANG_VEL_HPP