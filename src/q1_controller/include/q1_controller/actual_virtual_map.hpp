#ifndef __ACTUAL_VIRTUAL_MAP__
#define __ACTUAL_VIRTUAL_MAP__
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tuple>

class actual_virtual_map
{
  private:
    float a, b, c, d;

  public:
    actual_virtual_map(/* args */);
    ~actual_virtual_map();

    /**
     * @brief ABC函数：计算逆运动学中的系数A, B, C
     * @param a 系统几何参数
     * @param b 系统几何参数
     * @param c_or_d 根据上下文为c（用于phi）或d（用于theta）
     * @param alpha 虚拟关节角度（弧度）
     * @param beta 虚拟关节角度（弧度）
     * @return A,B,C std::tuple<float, float, float>
     */
    std::tuple<float, float, float> ABC(float a, float b, float c_or_d, float alpha, float beta);

    /**
     * @brief ankle_ik函数：逆运动学计算，给定alpha和beta，求phi和theta
     * @param alpha 虚拟关节角度（弧度）
     * @param beta 虚拟关节角度（弧度）
     * @return phi theta 
     */
    std::pair<float, float> ankle_ik(float alpha, float beta);

    /**
     * @brief ankle_fk函数：前向运动学，使用Newton-Raphson迭代求alpha和beta
     * @param phi 真实关节角度（弧度）
     * @param theta 真实关节角度（弧度）
     * @param alpha_init 初始猜测（默认0.0）
     * @param beta_init 初始猜测（默认0.0）
     * @param max_iterations 最大迭代次数（默认100）
     * @param tolerance 收敛容差（默认1e-5）
     * @return phi theta 
     * @note 若成功返回Eigen::Vector2d {alpha, beta}；否则返回{0.0, 0.0}
     */
    Eigen::Vector2f ankle_fk(float phi, float theta, float alpha_init = 0.0, float beta_init = 0.0,
                             int max_iterations = 100, float tolerance = 1e-5);

    /**
     * @brief velocity_mapping函数：计算虚拟关节角速度dot_alpha和dot_beta
     * @param phi 真实关节角度（弧度）
     * @param theta 真实关节角度（弧度）
     * @param alpha 当前虚拟关节角度（弧度）
     * @param beta 当前虚拟关节角度（弧度）
     * @param phi_dot 真实关节角速度
     * @param theta_dot 真实关节角速度
     * @return dot_alpha dot_beta
     * @note 若成功返回Eigen::Vector2d {dot_alpha, dot_beta}；否则返回{0.0, 0.0}
     */
    Eigen::Vector2f velocity_mapping(float phi, float theta, float alpha, float beta, float phi_dot, float theta_dot);
};

#endif