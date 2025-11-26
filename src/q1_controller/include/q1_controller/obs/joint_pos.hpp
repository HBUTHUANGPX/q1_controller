// 文件: joint_pos.hpp
#ifndef JOINT_POS_HPP
#define JOINT_POS_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>

/**
 * @brief 关节位置观测组件子类。
 *
 * 从JointStates话题获取当前关节位置，并减去默认位置。
 */
class JointPos : public ObservationComponent
{
  public:
    JointPos(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "joint_pos", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        Eigen::VectorXf data = data_store_->GetJointPositions(); // 减默认位置
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data[i]);
        // }
        // printf("]\n");
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for joint_pos.");
        }
        obs.block(0, offset_, 1, dim_) = (data * scale_).transpose();
        // std::cout<<"Update "<<name_<<": "<< obs.block(0, offset_, 1, dim_)<<std::endl;
    }
};

#endif // JOINT_POS_HPP