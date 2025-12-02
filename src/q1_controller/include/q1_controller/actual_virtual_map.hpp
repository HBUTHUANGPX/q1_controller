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
    // ABC函数：计算逆运动学中的系数A, B, C。
    // 参数：
    //   - a, b: 系统几何参数。
    //   - c_or_d: 根据上下文为c（用于phi）或d（用于theta）。
    //   - alpha, beta: 虚拟关节角度（弧度）。
    // 返回：std::tuple<float, float, float> {A, B, C}。
    std::tuple<float, float, float> ABC(float a, float b, float c_or_d, float alpha, float beta);
    // ankle_ik函数：逆运动学计算，给定alpha和beta，求phi和theta。
    // 参数：
    //   - alpha, beta: 虚拟关节角度（弧度）。
    //   - a, b, c, d: 系统参数。
    // 返回：std::pair<float, float> {phi, theta}。
    std::pair<float, float> ankle_ik(float alpha, float beta);
    // ankle_fk函数：前向运动学，使用Newton-Raphson迭代求alpha和beta。
    // 参数：
    //   - phi, theta: 真实关节角度（弧度）。
    //   - a, b, c, d: 系统参数。
    //   - alpha_init, beta_init: 初始猜测（默认0.0）。
    //   - max_iterations: 最大迭代次数（默认100）。
    //   - tolerance: 收敛容差（默认1e-5）。
    // 返回：Eigen::Vector2d {alpha, beta}，若成功；否则{0.0, 0.0}。
    Eigen::Vector2f ankle_fk(float phi, float theta, float alpha_init = 0.0, float beta_init = 0.0,
                             int max_iterations = 100, float tolerance = 1e-5);
    // velocity_mapping函数：计算虚拟关节角速度dot_alpha和dot_beta。
    // 参数：
    //   - phi, theta: 真实关节角度（弧度）。
    //   - alpha, beta: 当前虚拟关节角度（弧度）。
    //   - phi_dot, theta_dot: 真实关节角速度。
    //   - a, b, c, d: 系统参数。
    // 返回：Eigen::Vector2d {dot_alpha, dot_beta}，若成功；否则{0.0, 0.0}。
    Eigen::Vector2f velocity_mapping(float phi, float theta, float alpha, float beta, float phi_dot, float theta_dot);
};

#endif