#include "../../include/q1_controller/actual_virtual_map.hpp"

actual_virtual_map::actual_virtual_map(/* args */)
{
    a = 0.052;
    b = 0.023;
    c = 0.225;
    d = 0.284;
}

actual_virtual_map::~actual_virtual_map()
{
}

std::pair<float, float> actual_virtual_map::ABC(float b, float length, float alpha, float beta)
{
    // 计算三角函数（提高效率，避免重复调用）
    float sin_beta = std::sin(beta);
    float cos_beta = std::cos(beta);
    float sin_alpha = std::sin(alpha);
    float cos_alpha = std::cos(alpha);

    // P = a * cos_alpha - b * sin_alpha * sin_beta   （对应E_x或D_x的核心项）
    float P = a * cos_alpha - b * sin_alpha * sin_beta;

    // Q = length + a * sin_alpha + b * cos_alpha * sin_beta   （对应d - E_z或c - D_z的核心项）
    float Q = length + a * sin_alpha + b * cos_alpha * sin_beta;

    // M = b² (1 - cos_beta)²   （Y方向约束项）
    float M = b * b * (1.0f - cos_beta) * (1.0f - cos_beta);

    // Delta = a² + P² + Q² + M - length²
    float Delta = a * a + P * P + Q * Q + M - length * length;

    // S = Delta / (2a)   （避免除零）
    float S = Delta / (2.0f * a + 1e-10f);

    // R = sqrt(P² + Q²)   （避免下溢）
    float R = std::sqrt(P * P + Q * Q + 1e-10f);

    // 判断是否存在实解：|S| ≤ R
    if (std::abs(S) > R + 1e-6f)
    {
        std::cout << "No real solution for this branch (ABC function)" << std::endl;
        float nan_val = std::numeric_limits<float>::quiet_NaN();
        return {nan_val, nan_val};
    }

    // psi = atan2(Q, P)
    float psi = std::atan2(Q, P);

    // 计算acos参数并clamp到[-1, 1]以避免数值误差导致的NaN
    float acos_arg = S / R;
    if (acos_arg > 1.0f)
        acos_arg = 1.0f;
    if (acos_arg < -1.0f)
        acos_arg = -1.0f;
    float ac = std::acos(acos_arg);

    // 两个解：psi + ac 和 psi - ac
    float pos = psi + ac;
    float neg = psi - ac;

    return {pos, neg};
}

std::tuple<float, float, float, float> actual_virtual_map::ankle_ik(float alpha, float beta)
{
    // 上支链（theta）：使用+b和长度d
    auto [theta_pos, theta_neg] = ABC(b, d, alpha, beta);

    // 下支链（phi）：使用-b和长度c（对应Python中-b的符号翻转）
    auto [phi_pos, phi_neg] = ABC(-b, c, alpha, beta);

    // 返回顺序与Python一致：theta_pos, theta_neg, phi_pos, phi_neg
    return {theta_pos, theta_neg, phi_pos, phi_neg};
}

Eigen::Vector2f actual_virtual_map::ankle_fk(float phi, float theta, float init_alpha, float init_beta,
                                             int max_iterations, float tolerance)
{
    float sin_phi = std::sin(phi);
    float cos_phi = std::cos(phi);
    float sin_theta = std::sin(theta);
    float cos_theta = std::cos(theta);
    Eigen::Vector2f x(init_alpha, init_beta); // 当前估计 [alpha, beta]^T

    for (int i = 0; i < max_iterations; ++i)
    {
        float alpha = x(0);
        float beta = x(1);

        // 预计算三角函数
        float sin_beta = std::sin(beta);
        float cos_beta = std::cos(beta);
        float sin_alpha = std::sin(alpha);
        float cos_alpha = std::cos(alpha);

        // 计算残差向量V1 = F - E 和 V2 = C - D
        float V1x = a * cos_theta - (a * cos_alpha - b * sin_alpha * sin_beta);
        float V1y = -b - (-b * cos_beta); // = b * (cos_beta - 1)
        float V1z = d - a * sin_theta - (-a * sin_alpha - b * cos_alpha * sin_beta);

        float V2x = a * cos_phi - (a * cos_alpha + b * sin_alpha * sin_beta);
        float V2y = b - b * cos_beta; // = b * (1 - cos_beta)
        float V2z = c - a * sin_phi - (-a * sin_alpha + b * cos_alpha * sin_beta);

        // 残差函数 f1 = ||F - E||² - d², f2 = ||C - D||² - c²
        float f1 = V1x * V1x + V1y * V1y + V1z * V1z - d * d;
        float f2 = V2x * V2x + V2y * V2y + V2z * V2z - c * c;
        Eigen::Vector2f F(f1, f2);

        // 收敛判断：残差范数
        float norm_F = std::sqrt(f1 * f1 + f2 * f2);
        if (norm_F < tolerance)
        {
            return x;
        }

        // 计算∂E/∂alpha, ∂E/∂beta, ∂D/∂alpha, ∂D/∂beta（与Python完全一致）
        float dE_alpha_x = -a * sin_alpha - b * cos_alpha * sin_beta;
        float dE_alpha_y = 0.0f;
        float dE_alpha_z = -a * cos_alpha + b * sin_alpha * sin_beta;

        float dE_beta_x = -b * sin_alpha * cos_beta;
        float dE_beta_y = b * sin_beta;
        float dE_beta_z = -b * cos_alpha * cos_beta;

        float dD_alpha_x = -a * sin_alpha + b * cos_alpha * sin_beta;
        float dD_alpha_y = 0.0f;
        float dD_alpha_z = -a * cos_alpha - b * sin_alpha * sin_beta;

        float dD_beta_x = b * sin_alpha * cos_beta;
        float dD_beta_y = -b * sin_beta;
        float dD_beta_z = b * cos_alpha * cos_beta;

        // 雅可比矩阵 ∂f/∂x（包含-2因子）
        float J_11 = -2.0f * (V1x * dE_alpha_x + V1y * dE_alpha_y + V1z * dE_alpha_z);
        float J_12 = -2.0f * (V1x * dE_beta_x + V1y * dE_beta_y + V1z * dE_beta_z);
        float J_21 = -2.0f * (V2x * dD_alpha_x + V2y * dD_alpha_y + V2z * dD_alpha_z);
        float J_22 = -2.0f * (V2x * dD_beta_x + V2y * dD_beta_y + V2z * dD_beta_z);

        Eigen::Matrix2f J;
        J << J_11, J_12, J_21, J_22;

        // 奇异性检查
        float det_J = J.determinant();
        if (std::abs(det_J) < 1e-10f)
        {
            std::cout << "Jacobian singular in FK iteration" << std::endl;
            return Eigen::Vector2f(0.0f, 0.0f);
        }

        // 牛顿更新：delta_x = -J⁻¹ F
        Eigen::Vector2f delta_x = -J.inverse() * F; // 2x2直接inverse安全
        x += delta_x;

        // 二次收敛判断（更新量）
        if (delta_x.norm() < tolerance)
        {
            return x;
        }
    }

    std::cout << "FK did not converge within max iterations" << std::endl;
    return Eigen::Vector2f(0.0f, 0.0f);
}

Eigen::Vector2f actual_virtual_map::ankle_velocity_map(float phi, float theta, float alpha, float beta,float phi_dot, float theta_dot)
{
    // 预计算三角函数（与FK中相同）
    float sin_beta = std::sin(beta);
    float cos_beta = std::cos(beta);
    float sin_alpha = std::sin(alpha);
    float cos_alpha = std::cos(alpha);
    float sin_phi = std::sin(phi);
    float cos_phi = std::cos(phi);
    float sin_theta = std::sin(theta);
    float cos_theta = std::cos(theta);

    // 计算残差向量V1、V2（与FK相同）
    float V1x = a * cos_theta - (a * cos_alpha - b * sin_alpha * sin_beta);
    float V1y = -b - (-b * cos_beta);
    float V1z = d - a * sin_theta - (-a * sin_alpha - b * cos_alpha * sin_beta);

    float V2x = a * cos_phi - (a * cos_alpha + b * sin_alpha * sin_beta);
    float V2y = b - b * cos_beta;
    float V2z = c - a * sin_phi - (-a * sin_alpha + b * cos_alpha * sin_beta);

    // 计算偏导数（与FK相同）
    float dE_alpha_x = -a * sin_alpha - b * cos_alpha * sin_beta;
    float dE_alpha_y = 0.0f;
    float dE_alpha_z = -a * cos_alpha + b * sin_alpha * sin_beta;

    float dE_beta_x = -b * sin_alpha * cos_beta;
    float dE_beta_y = b * sin_beta;
    float dE_beta_z = -b * cos_alpha * cos_beta;

    float dD_alpha_x = -a * sin_alpha + b * cos_alpha * sin_beta;
    float dD_alpha_y = 0.0f;
    float dD_alpha_z = -a * cos_alpha - b * sin_alpha * sin_beta;

    float dD_beta_x = b * sin_alpha * cos_beta;
    float dD_beta_y = -b * sin_beta;
    float dD_beta_z = b * cos_alpha * cos_beta;

    // ∂f/∂x 雅可比（包含-2因子）
    float df_dx_11 = -2.0f * (V1x * dE_alpha_x + V1y * dE_alpha_y + V1z * dE_alpha_z);
    float df_dx_12 = -2.0f * (V1x * dE_beta_x + V1y * dE_beta_y + V1z * dE_beta_z);
    float df_dx_21 = -2.0f * (V2x * dD_alpha_x + V2y * dD_alpha_y + V2z * dD_alpha_z);
    float df_dx_22 = -2.0f * (V2x * dD_beta_x + V2y * dD_beta_y + V2z * dD_beta_z);

    Eigen::Matrix2f df_dx;
    df_dx << df_dx_11, df_dx_12, df_dx_21, df_dx_22;

    // 奇异性检查
    float det_df_dx = df_dx.determinant();
    if (std::abs(det_df_dx) < 1e-10f)
    {
        std::cout << "Jacobian singular in velocity map" << std::endl;
        return Eigen::Vector2f::Zero();
    }

    Eigen::Matrix2f df_dx_inv = df_dx.inverse();

    // ∂f/∂q（对角矩阵，包含2因子）
    float df_dq_11 = 2.0f * (-a * (V1x * sin_theta + V1z * cos_theta));
    float df_dq_22 = 2.0f * (-a * (V2x * sin_phi + V2z * cos_phi));

    Eigen::Matrix2f df_dq = Eigen::Matrix2f::Zero();
    df_dq(0, 0) = df_dq_11;
    df_dq(1, 1) = df_dq_22;

    // 速度雅可比 J = - (∂f/∂x)⁻¹ * (∂f/∂q)
    Eigen::Matrix2f velocity_jacobian = -df_dx_inv * df_dq;
    
    Eigen::Vector2f q_dot(phi_dot, theta_dot); // 当前估计 [alpha, beta]^T
    Eigen::Vector2f x_dot = velocity_jacobian * q_dot;
    return x_dot;
}