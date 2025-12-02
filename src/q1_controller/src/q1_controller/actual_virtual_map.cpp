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

std::tuple<float, float, float> actual_virtual_map::ABC(float a, float b, float c_or_d, float alpha, float beta)
{
    float s_alpha = std::sin(alpha); // sin(alpha)
    float s_beta = std::sin(beta);   // sin(beta)
    float c_alpha = std::cos(alpha); // cos(alpha)
    float c_beta = std::cos(beta);   // cos(beta)

    // 计算A: 2 * a * (b * sin(alpha) * sin(beta) - a * cos(alpha))
    float A = 2 * a * (b * s_alpha * s_beta - a * c_alpha);

    // 计算B: 2 * a * (c_or_d - a * sin(alpha) - b * cos(alpha) * sin(beta))
    float B = 2 * a * (c_or_d - a * s_alpha - b * c_alpha * s_beta);

    // 计算C: 2 * a^2 + 2 * b^2 * (1 - cos(beta)) - 2 * a * c_or_d * sin(alpha) - 2 * b * c_or_d * cos(alpha) *
    // sin(beta)
    float C = 2 * a * a + 2 * b * b * (1 - c_beta) - 2 * a * c_or_d * s_alpha - 2 * b * c_or_d * c_alpha * s_beta;

    return {A, B, C};
}

std::pair<float, float> actual_virtual_map::ankle_ik(float alpha, float beta)
{
    // 计算phi的A, B, C（b正）
    auto [A_phi, B_phi, C_phi] = ABC(a, b, c, alpha, beta);

    // gamma = atan2(B_phi, A_phi)
    float gamma_phi = std::atan2(B_phi, A_phi);

    // ac = acos(-C_phi / sqrt(A_phi^2 + B_phi^2))
    float sqrt_term_phi = std::sqrt(A_phi * A_phi + B_phi * B_phi);
    float ac_phi = std::acos(-C_phi / sqrt_term_phi);

    // phi = gamma - ac
    float phi = gamma_phi - ac_phi;

    // 计算theta的A, B, C（b负）
    auto [A_theta, B_theta, C_theta] = ABC(a, -b, d, alpha, beta);

    // gamma = atan2(B_theta, A_theta)
    float gamma_theta = std::atan2(B_theta, A_theta);

    // ac = acos(-C_theta / sqrt(A_theta^2 + B_theta^2))
    float sqrt_term_theta = std::sqrt(A_theta * A_theta + B_theta * B_theta);
    float ac_theta = std::acos(-C_theta / sqrt_term_theta);

    // theta = gamma - ac
    float theta = gamma_theta - ac_theta;

    return {phi, theta};
}

Eigen::Vector2f actual_virtual_map::ankle_fk(float phi, float theta, float alpha_init, float beta_init,
                                             int max_iterations, float tolerance)
{
    Eigen::Vector2f x(alpha_init, beta_init); // 初始化x = [alpha, beta]

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        float alpha = x(0);
        float beta = x(1);
        float cos_alpha = std::cos(alpha);
        float sin_alpha = std::sin(alpha);
        float cos_beta = std::cos(beta);
        float sin_beta = std::sin(beta);

        // 计算D'
        float Dx = -a * cos_alpha + b * sin_beta * sin_alpha;
        float Dy = -b * cos_beta;
        float Dz = a * sin_alpha + b * sin_beta * cos_alpha;

        // 计算E'
        float Ex = -a * cos_alpha - b * sin_beta * sin_alpha;
        float Ey = b * cos_beta;
        float Ez = a * sin_alpha - b * sin_beta * cos_alpha;

        // 计算C
        float Cx = -a * std::cos(phi);
        float Cy = -b;
        float Cz = c + a * std::sin(phi);

        // 计算F
        float Fx = -a * std::cos(theta);
        float Fy = b;
        float Fz = d + a * std::sin(theta);

        // 计算目标函数f
        float f1 = (Cx - Dx) * (Cx - Dx) + (Cy - Dy) * (Cy - Dy) + (Cz - Dz) * (Cz - Dz) - c * c;
        float f2 = (Fx - Ex) * (Fx - Ex) + (Fy - Ey) * (Fy - Ey) + (Fz - Ez) * (Fz - Ez) - d * d;
        Eigen::Vector2f f(f1, f2);

        // 检查f的收敛
        if (f.cwiseAbs().maxCoeff() < tolerance)
        {
            return x;
        }

        // 计算偏导数（f1关于alpha）
        float dDx_da = a * sin_alpha + b * sin_beta * cos_alpha;
        float dDy_da = 0.0;
        float dDz_da = a * cos_alpha - b * sin_beta * sin_alpha;
        float df1_da = 2 * (Cx - Dx) * (-dDx_da) + 2 * (Cy - Dy) * (-dDy_da) + 2 * (Cz - Dz) * (-dDz_da);

        // f1关于beta
        float dDx_db = b * cos_beta * sin_alpha;
        float dDy_db = b * sin_beta;
        float dDz_db = b * cos_beta * cos_alpha;
        float df1_db = 2 * (Cx - Dx) * (-dDx_db) + 2 * (Cy - Dy) * (-dDy_db) + 2 * (Cz - Dz) * (-dDz_db);

        // f2关于alpha
        float dEx_da = a * sin_alpha - b * sin_beta * cos_alpha;
        float dEy_da = 0.0;
        float dEz_da = a * cos_alpha + b * sin_beta * sin_alpha;
        float df2_da = 2 * (Fx - Ex) * (-dEx_da) + 2 * (Fy - Ey) * (-dEy_da) + 2 * (Fz - Ez) * (-dEz_da);

        // f2关于beta
        float dEx_db = -b * cos_beta * sin_alpha;
        float dEy_db = -b * sin_beta;
        float dEz_db = -b * cos_beta * cos_alpha;
        float df2_db = 2 * (Fx - Ex) * (-dEx_db) + 2 * (Fy - Ey) * (-dEy_db) + 2 * (Fz - Ez) * (-dEz_db);

        // 组装雅可比矩阵J
        Eigen::Matrix2f J;
        J << df1_da, df1_db, df2_da, df2_db;

        // 求解delta = -J^{-1} f
        Eigen::Vector2f neg_f = -f;
        Eigen::ColPivHouseholderQR<Eigen::Matrix2f> qr(J);
        if (qr.info() != Eigen::Success)
        {
            std::cerr << "矩阵奇异，无法求解" << std::endl;
            return Eigen::Vector2f::Zero();
        }
        Eigen::Vector2f delta = qr.solve(neg_f);

        // 更新x
        x += delta;

        // 检查delta的收敛
        if (delta.cwiseAbs().maxCoeff() < tolerance)
        {
            return x;
        }
    }

    std::cerr << "未在最大迭代次数内收敛" << std::endl;
    return Eigen::Vector2f::Zero();
}

Eigen::Vector2f actual_virtual_map::velocity_mapping(float phi, float theta, float alpha, float beta, float phi_dot,
                                                     float theta_dot)
{
    float cos_alpha = std::cos(alpha);
    float sin_alpha = std::sin(alpha);
    float cos_beta = std::cos(beta);
    float sin_beta = std::sin(beta);

    // 计算坐标D', E', C, F
    float Dx = -a * cos_alpha + b * sin_beta * sin_alpha;
    float Dy = -b * cos_beta;
    float Dz = a * sin_alpha + b * sin_beta * cos_alpha;

    float Ex = -a * cos_alpha - b * sin_beta * sin_alpha;
    float Ey = b * cos_beta;
    float Ez = a * sin_alpha - b * sin_beta * cos_alpha;

    float Cx = -a * std::cos(phi);
    float Cy = -b;
    float Cz = c + a * std::sin(phi);

    float Fx = -a * std::cos(theta);
    float Fy = b;
    float Fz = d + a * std::sin(theta);

    // D'和E'的偏导数
    float dDx_da = a * sin_alpha + b * sin_beta * cos_alpha;
    float dDy_da = 0.0;
    float dDz_da = a * cos_alpha - b * sin_beta * sin_alpha;

    float dDx_db = b * cos_beta * sin_alpha;
    float dDy_db = b * sin_beta;
    float dDz_db = b * cos_beta * cos_alpha;

    float dEx_da = a * sin_alpha - b * sin_beta * cos_alpha;
    float dEy_da = 0.0;
    float dEz_da = a * cos_alpha + b * sin_beta * sin_alpha;

    float dEx_db = -b * cos_beta * sin_alpha;
    float dEy_db = -b * sin_beta;
    float dEz_db = -b * cos_beta * cos_alpha;

    // C和F的偏导数
    float dCx_dphi = a * std::sin(phi);
    float dCz_dphi = a * std::cos(phi);

    float dFx_dtheta = a * std::sin(theta);
    float dFz_dtheta = a * std::cos(theta);

    // f1和f2关于alpha, beta的偏导数
    float df1_da = 2 * (Cx - Dx) * (-dDx_da) + 2 * (Cy - Dy) * 0.0 + 2 * (Cz - Dz) * (-dDz_da);
    float df1_db = 2 * (Cx - Dx) * (-dDx_db) + 2 * (Cy - Dy) * (-dDy_db) + 2 * (Cz - Dz) * (-dDz_db);

    float df2_da = 2 * (Fx - Ex) * (-dEx_da) + 2 * (Fy - Ey) * 0.0 + 2 * (Fz - Ez) * (-dEz_da);
    float df2_db = 2 * (Fx - Ex) * (-dEx_db) + 2 * (Fy - Ey) * (-dEy_db) + 2 * (Fz - Ez) * (-dEz_db);

    // 雅可比矩阵J
    Eigen::Matrix2f J;
    J << df1_da, df1_db, df2_da, df2_db;

    // f1和f2关于phi, theta的偏导数
    float df1_dphi = 2 * ((Cx - Dx) * dCx_dphi + (Cz - Dz) * dCz_dphi);
    float df1_dtheta = 0.0;
    float df2_dphi = 0.0;
    float df2_dtheta = 2 * ((Fx - Ex) * dFx_dtheta + (Fz - Ez) * dFz_dtheta);

    // 右侧向量b = - [df1_dphi * phi_dot + df1_dtheta * theta_dot, df2_dphi * phi_dot + df2_dtheta * theta_dot]
    Eigen::Vector2f _b;
    _b << -(df1_dphi * phi_dot + df1_dtheta * theta_dot), -(df2_dphi * phi_dot + df2_dtheta * theta_dot);

    // 求解J * dot_v = b
    Eigen::ColPivHouseholderQR<Eigen::Matrix2f> qr(J);
    if (qr.info() != Eigen::Success)
    {
        std::cerr << "J矩阵奇异" << std::endl;
        return Eigen::Vector2f::Zero();
    }
    Eigen::Vector2f dot_v = qr.solve(_b);

    return dot_v;
}