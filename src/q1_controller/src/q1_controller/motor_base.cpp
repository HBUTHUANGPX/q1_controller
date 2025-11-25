// 文件: motor_base.cpp
#include "../../include/q1_controller/motor_base.hpp"

/**
 * @brief MotorBase构造函数实现。
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