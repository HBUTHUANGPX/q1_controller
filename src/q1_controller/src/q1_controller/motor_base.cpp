// 文件: motor_base.cpp
#include "../../include/q1_controller/motor_base.hpp"

/**
 * @brief MotorBase构造函数实现。
 * @param name 电机名称。
 * @param kp P增益。
 * @param kd D增益。
 * @param max_torque 最大扭矩。
 * @param default_pos 默认位置。
 * @param id 电机ID。
 * @param ec_id 电机在以太网中的ID。
 * @param direction 电机方向（1或-1）。
 */
MotorBase::MotorBase(const std::string &name, float kp, float kd, float max_torque, float default_pos, int id,
                     int ec_id,int direction)
    : name_(name), kp_(kp), kd_(kd), max_torque_(max_torque), default_pos_(default_pos), id_(id), pos_(0), vel_(0),
      fft_(0), ec_id_(ec_id),direction_(direction)
{
#if defined(USE_TENSORRT)
    motor_data.id = ec_id_;
    motor_data.kp = kp_;
    motor_data.kd = kd_;
    motor_data.pos = 0;
    motor_data.vel = 0;
    motor_data.tor = 0;
#endif
}