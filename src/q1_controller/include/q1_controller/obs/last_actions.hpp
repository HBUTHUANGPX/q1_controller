// 文件: last_actions.hpp
#ifndef LAST_ACTIONS_HPP
#define LAST_ACTIONS_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 上一次动作观测组件子类。
 *
 * 从内部状态或上一帧动作获取最后动作。
 */
class LastActions : public ObservationComponent
{
  public:
    LastActions(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "last_actions", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetLastActions();
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for last_actions.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
    }

  private:
  
};

#endif // LAST_ACTIONS_HPP