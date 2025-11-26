// 文件: motion_loader.hpp
#ifndef MOTION_LOADER_HPP
#define MOTION_LOADER_HPP

#include <Eigen/Dense>
#include <cnpy.h> // 第三方库，用于加载 NPZ 文件
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h> // 假设 YAML 用于配置
/**
 * @brief MotionLoader 类，用于从 NPZ 文件加载运动数据。
 *
 * 此类加载 NPZ 文件中的数据，包括 joint_pos、joint_vel 和 3D body 数据。
 * 数据使用 Eigen 存储：2D 数据为 MatrixXf (行: 时间步, 列: 维度)；
 * 3D 数据为 vector<MatrixXf> (vector 维度: 时间步, MatrixXf: body_num x 维度)。
 * 支持根据 body_indexes 返回子集视图。
 */
class MotionLoader
{
  public:
    /**
     * @brief 构造函数，从 NPZ 文件加载数据。
     * @param config_ YAML 配置节点，包含 motion_file 和 body_indexes。
     */
    MotionLoader(const YAML::Node &config_);

    /**
     * @brief 析构函数。
     */
    ~MotionLoader() = default;

    /**
     * @brief 获取 FPS 值
     * @return FPS 值
     */
    float getFps() const
    {
        return fps_;
    }

    /**
     * @brief 获取总时间步数
     * @return 总时间步数
     */
    size_t getTimeStepTotal() const
    {
        return time_step_total_;
    }

    /**
     * @brief 获取 joint_pos (完整数据)
     * @param index 时间步索引
     * @return joint_pos 数据
     */
    Eigen::MatrixXf getJointPos(size_t index) const
    {

        Eigen::MatrixXf col = joint_pos_.col(index);
        return col;
    }

    /**
     * @brief 获取 joint_vel (完整数据)
     * @param index 时间步索引
     * @return joint_vel 数据
     */
    Eigen::MatrixXf getJointVel(size_t index) const
    {
        Eigen::MatrixXf col = joint_vel_.col(index);
        return col;
    }

    /**
     * @brief 获取 body_pos_w 子集 (时间 x body_indexes.size() x 3)
     * @param index 时间步索引
     * @return body_pos_w 子集数据
     */
    Eigen::MatrixXf getBodyPosW(size_t index) const;

    /**
     * @brief 获取 body_quat_w 子集 (时间 x body_indexes.size() x 4)
     * @param index 时间步索引
     * @return body_quat_w 子集数据
     */
    Eigen::MatrixXf getBodyQuatW(size_t index) const;

    /**
     * @brief 获取 body_lin_vel_w 子集 (时间 x body_indexes.size() x 3)
     * @param index 时间步索引
     * @return body_lin_vel_w 子集数据
     */
    Eigen::MatrixXf getBodyLinVelW(size_t index) const;

    /**
     * @brief 获取 body_ang_vel_w 子集 (时间 x body_indexes.size() x 3)
     * @param index 时间步索引
     * @return body_ang_vel_w 子集数据
     */
    Eigen::MatrixXf getBodyAngVelW(size_t index) const;

  private:
    float fps_;              // FPS 值
    size_t time_step_total_; // 总时间步数

    Eigen::MatrixXf joint_pos_; // joint_pos 数据 [时间, 29]
    Eigen::MatrixXf joint_vel_; // joint_vel 数据 [时间, 29]

    std::vector<Eigen::MatrixXf> body_pos_w_;     // body_pos_w 数据 [时间] x [30, 3]
    std::vector<Eigen::MatrixXf> body_quat_w_;    // body_quat_w 数据 [时间] x [30, 4]
    std::vector<Eigen::MatrixXf> body_lin_vel_w_; // body_lin_vel_w 数据 [时间] x [30, 3]
    std::vector<Eigen::MatrixXf> body_ang_vel_w_; // body_ang_vel_w 数据 [时间] x [30, 3]

    std::vector<int> body_indexes_; // body 索引序列

    /**
     * @brief 从 NPZ 文件加载数据。
     * @param motion_file NPZ 文件路径。
     */
    void loadData(const std::string &motion_file);
};

#endif // MOTION_LOADER_HPP