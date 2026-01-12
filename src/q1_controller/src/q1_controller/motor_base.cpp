// 文件: motor_base.cpp
#include "../../include/q1_controller/motor_base.hpp"

/**
 * @brief MotorBase构造函数实现。
 */
MotorBase::MotorBase(const std::string &name, float kp, float kd, float max_torque,float trans_eff, float nominal_pos,
                     float urdf_offset, int id, int ec_id, int direction,const std::string &_motor_type)
    : name_(name), kp_(kp/trans_eff), kd_(kd/trans_eff), max_torque_(max_torque), nominal_pos_(nominal_pos), urdf_offset_(urdf_offset),
      id_(id), pos_(0), vel_(0), fft_(0), ec_id_(ec_id), direction_(direction),trans_eff_(trans_eff)
{
    kp_ = kp/trans_eff;
    kd_ = kd/trans_eff;
    if (_motor_type == "none")
    {
        motor_type_ = motor_type::NONE;
    }
    else if (_motor_type == "A4310_P2_36")
    {
        motor_type_ = motor_type::A4310_P2_36;
    }
    else if (_motor_type == "A4315_P2_36")
    {
        motor_type_ = motor_type::A4315_P2_36;
    }
    else if (_motor_type == "A6408_P2_25")
    {
        motor_type_ = motor_type::A6408_P2_25;
    }
    else if (_motor_type == "A6416_P2_25")
    {
        motor_type_ = motor_type::A6416_P2_25;
    }
    else if (_motor_type == "A8112_P1_18")
    {
        motor_type_ = motor_type::A8112_P1_18;
    }
    else if (_motor_type == "A8116_P1_18")
    {
        motor_type_ = motor_type::A8116_P1_18;
    }
    else if (_motor_type == "A10020_P1_12")
    {
        motor_type_ = motor_type::A10020_P1_12;
    }
    else if (_motor_type == "A10020_P2_24")
    {
        motor_type_ = motor_type::A10020_P2_24;
    }
    else if (_motor_type == "Ti5_30_40")
    {
        motor_type_ = motor_type::Ti5_30_40;
    }
    else if (_motor_type == "Ti5_40_52")
    {
        motor_type_ = motor_type::Ti5_40_52;
    }
    else if (_motor_type == "Ti5_50_70")
    {
        motor_type_ = motor_type::Ti5_50_70;
    }
    else if (_motor_type == "Ti5_60_80")
    {
        motor_type_ = motor_type::Ti5_60_80;
    }
#if defined(USE_TENSORRT)
    motor_data.id = ec_id_;
    motor_data.kp = kp_;
    motor_data.kd = kd_;
    motor_data.pos = 0;
    motor_data.vel = 0;
    motor_data.tor = 0;
    motor_data.type = motor_type_;
#endif
    offset_pos_ = nominal_pos_ - urdf_offset_;
    
    
}