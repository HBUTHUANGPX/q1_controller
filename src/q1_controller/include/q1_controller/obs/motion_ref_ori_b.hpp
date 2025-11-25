// 文件: motion_ref_ori_b.hpp
#ifndef MOTION_REF_ORI_B_HPP
#define MOTION_REF_ORI_B_HPP

#include "../observation_base.hpp"
#include <Eigen/Dense>
#include <iostream>
/**
 * @brief 运动参考方向（body frame）观测组件子类。
 *
 * 计算参考方向的矩阵表示（从四元数转换），并展平为向量。
 */
class MotionRefOriB : public ObservationComponent
{
  public:
    MotionRefOriB(int dim, float scale, std::shared_ptr<DataStore> data_store)
        : ObservationComponent(dim, "motion_ref_ori_b", scale, data_store)
    {
    }

    void Update(Eigen::MatrixXf &obs, float time_step) override
    {
        // std::cout << "Update\r\n";
        Eigen::MatrixXf data = ComputeRefOriFromQuat(time_step); // 形状为(3,3)，展平为6（前两列）
        // std::cout << "data\r\n";
        // std::cout << "components: "<<name_ <<"\n[";
        // for (int i = 0; i < data.size(); ++i) {
        //     printf("%7.4f ",data(0, i));
        // }
        // printf("]\n");
        if (data.rows() != 1 || data.cols() != dim_)
        {
            throw std::runtime_error("Invalid data shape for motion_ref_ori_b: expected 1x" + std::to_string(dim_));
        }
        if (data.size() != dim_)
        {
            throw std::runtime_error("Data size mismatch for motion_ref_ori_b.");
        }
        // std::cout << "block\r\n";
        obs.block(0, offset_, 1, dim_) = data * scale_;
        // std::cout << "ok\r\n";
    }

  private:
    Eigen::MatrixXf ComputeRefOriFromQuat(float time_step)
    {
        // std::cout << "ComputeRefOriFromQuat\r\n";
        Eigen::Matrix3f mat = data_store_->ComputeMotionRefOriMatrix(time_step);
        // std::cout << "mat\r\n";
        // 展平前两列为1x6
        Eigen::MatrixXf block_mat = mat.block(0, 0, 3, 2).eval(); // 使用 .eval() 物化视图
        // std::cout << "ComputeRefOriFromQuat block_mat ： \r\n";
        // std::cout << block_mat.reshaped<Eigen::RowMajor>(1, 6) << " \r\n";
        return block_mat.reshaped<Eigen::RowMajor>(1, 6); // 现在安全重塑
        // return mat.block(0, 0, 3, 2).reshaped<Eigen::RowMajor>(1, 6);
    }
};

#endif // MOTION_REF_ORI_B_HPP