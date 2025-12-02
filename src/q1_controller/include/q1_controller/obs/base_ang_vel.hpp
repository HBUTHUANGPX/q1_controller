// 文件: base_ang_vel.hpp
#ifndef BASE_ANG_VEL_HPP
#define BASE_ANG_VEL_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 基座角速度观测组件子类。
 *
 * 从IMU或ROS话题获取基座角速度。
 */
class BaseAngVel : public ObservationComponent
{
  public:
    BaseAngVel(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "base_ang_vel", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetBaseAngularVelocities(); // 从IMU话题获取
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for base_ang_vel.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
    
};

#endif // BASE_ANG_VEL_HPP