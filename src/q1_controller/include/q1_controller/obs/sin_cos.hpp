// 文件: sin_cos.hpp
#ifndef SIN_COS_HPP
#define SIN_COS_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>
#include <cmath>
/**
 * @brief 上一次动作观测组件子类。
 *
 * 从内部状态或上一帧动作获取最后动作。
 */
class SinCos : public ObservationComponent
{
  public:
    SinCos(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "sin_cos", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        float period = 0.8f;
        float count = time_step / 50.f;
        float phase = std::fmod(count, period) / period;
        Eigen::Vector2f data;
        Eigen::Vector3f cmd_vel = data_store_->GetCmdVel();
        float cmd_norm = cmd_vel.norm();
        float stand_command = (cmd_norm < 0.1f) ? 1.0f : 0.0f;

        if (stand_command > 0.5)
        {
            data << 0.f, 1.f;
        }
        else
        {
            data << std::sin(2.f * M_PI * phase), std::cos(2.f * M_PI * phase);
        }

        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
};

#endif // SIN_COS_HPP