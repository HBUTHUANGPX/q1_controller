#ifndef __ACTUAL_VIRTUAL_MAP__
#define __ACTUAL_VIRTUAL_MAP__
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>

class actual_virtual_map
{
  private:
    float a; // 连杆参数 a
    float b; // 连杆参数 b
    float c; // 连杆参数 c（下支链长度约束）
    float d; // 连杆参数 d（上支链长度约束）

  public:
    actual_virtual_map(/* args */);
    ~actual_virtual_map();
    /**
     * @brief 计算单个支链的逆运动学解析解（两个可能解：正分支和负分支）
     *
     * 该函数对应Python中的ABC函数，实现最新的解析逆运动学。
     * 参数b可正可负（用于区分上/下支链的Y方向偏移符号）。
     * length为约束长度（c或d）。
     * 返回std::pair<float, float>：第一个为psi + acos，第二个为psi - acos。
     * 若无实解，返回{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()}。
     */
    std::pair<float, float> ABC(float b, float length, float alpha, float beta);
    /**
     * @brief 完整的踝关节逆运动学（IK）
     *
     * 给定虚拟关节alpha、beta，计算两个驱动关节的可能解（每个驱动角有两个装配模式）。
     * 返回顺序：theta_pos, theta_neg, phi_pos, phi_neg。
     */
    std::tuple<float, float, float, float> ankle_ik(float alpha, float beta);
    /**
     * @brief 踝关节正运动学（FK），使用Newton-Raphson迭代求解alpha、beta
     *
     * 给定驱动角phi、theta，通过牛顿迭代求解虚拟关节alpha、beta。
     * 与最新Python实现完全一致，包括残差函数、雅可比计算和收敛判断。
     * 若奇异或不收敛，返回{0.0f, 0.0f}。
     */
    Eigen::Vector2f ankle_fk(float phi, float theta, float init_alpha = 0.0f, float init_beta = 0.0f,
                             int max_iterations = 100, float tolerance = 1e-5f);
    /**
     * @brief 计算速度雅可比矩阵（虚拟关节速度到电机关节速度的映射）
     *
     * 返回2x2矩阵J，使得 dot_x = J * dot_q，其中dot_x = [dot_alpha, dot_beta]^T，dot_q = [dot_phi, dot_theta]^T。
     * 与最新Python ankle_velocity_map完全一致（返回 -df_dx_inv * df_dq）。
     * 若雅可比奇异，返回零矩阵。
     */
    Eigen::Vector2f ankle_velocity_map(float phi, float theta, float alpha, float beta,float phi_dot, float theta_dot);
};

#endif