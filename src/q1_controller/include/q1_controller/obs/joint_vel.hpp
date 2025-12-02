// 文件: joint_vel.hpp
#ifndef JOINT_VEL_HPP
#define JOINT_VEL_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 关节速度观测组件子类。
 *
 * 从JointStates话题获取当前关节速度。
 */
class JointVel : public ObservationComponent
{
  public:
    JointVel(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "joint_vel", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetJointVelocities();
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for joint_vel.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
  
};

#endif // JOINT_VEL_HPP