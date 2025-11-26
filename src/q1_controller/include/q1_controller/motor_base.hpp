// 文件: motor_base.hpp
#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#include <string>
#if defined(USE_TENSORRT)
#include "rt_comm_inipc/DoubleBuffer.hpp"
#include "rt_comm_inipc/MotorCommandAPI.hpp"
#include "rt_comm_inipc/MotorInfo.hpp"
#include "rt_comm_inipc/rt_ipc.hpp"
#endif
/**
 * @brief Motor基类，定义单个电机的基本属性。
 *
 * 此类包含电机的名称、PD增益（kp、kd）、最大扭矩和默认位置。
 * 设计为抽象基类，可扩展以添加更多功能。
 */
class MotorBase
{
  public:
    /**
     * @brief 构造函数，初始化电机属性。
     * @param name 电机名称。
     * @param kp P增益。
     * @param kd D增益。
     * @param max_torque 最大扭矩。
     * @param default_pos 默认位置。
     */
    MotorBase(const std::string &name, float kp, float kd, float max_torque, float nominal_pos, float urdf_offset,
              int id, int ec_id, int direction);

    /**
     * @brief 虚析构函数，支持子类扩展。
     */
    virtual ~MotorBase() = default;

    // 获取电机名称
    const std::string &getName() const
    {
        return name_;
    }

    // 获取P增益
    float getKp() const
    {
        return kp_;
    }

    // 获取D增益
    float getKd() const
    {
        return kd_;
    }

    // 获取最大扭矩
    float getMaxTorque() const
    {
        return max_torque_;
    }

    // 获取默认位置
    float getOffsetPos() const
    {
        return offset_pos_;
    }
    float getNominalPos() const
    {
        return nominal_pos_;
    }
    float getURDFOffset() const
    {
        return urdf_offset_;
    }

    float getCurrentPos() const
    {
        return pos_;
    }
    float getCurrentVel() const
    {
        return vel_;
    }
    float getCurrentFFT() const
    {
        return fft_;
    }
    int getID() const
    {
        return id_;
    }
    int getEC_ID() const
    {
        return ec_id_;
    }

    void writeCurrentPos(float value)
    {
        pos_ = value * direction_ - offset_pos_;
    }

    void rewriteCurrentPos(float value)
    {
        pos_ = value * direction_;
    }

    void writeCurrentVel(float value)
    {
        vel_ = value * direction_;
    }
    void rewriteCurrentVel(float value)
    {
        vel_ = value * direction_;
    }
    void writeCurrentFFT(float value)
    {
        fft_ = value * direction_;
    }
    /*
        getTargetPos用来获得PD传入的目标角度， setTargetPos 用来更新， resetTargetPos 也是用来更新
        setTargetPos 在sim2real中，将对所有关节电机乘一个转向，
        而 resetTargetPos 只在处理踝关节ik计算时使用
        对于sim2real，将使用motor_data.pos用来存放数据
        对于sim2sim，将使用独立变量target_pos_用来存放数据。
        TODO：后面可以对这个进行优化，统一使用motor_data。
    */
    float getTargetPos()
    {
#if defined(USE_TENSORRT)
        return motor_data.pos;
#else
        return target_pos_;
#endif
    }
    void setTargetPos(float value) // 将action数据给入
    {
#if defined(USE_TENSORRT)
        motor_data.pos = value * direction_ + offset_pos_; // 对于实机，此时需要进行方向转换
#else
        target_pos_ = value + offset_pos_;
#endif
    }
    void resetTargetPos(float value)
    {
#if defined(USE_TENSORRT)
        motor_data.pos = value;
#else
        target_pos_ = value;
#endif
    }
#if defined(USE_TENSORRT)
    MotorInfo getMotorInfo()
    {
        return motor_data;
    }
#endif
  protected:
    std::string name_;                // 电机名称
    float kp_;                        // P增益
    float kd_;                        // D增益
    float max_torque_;                // 最大扭矩
    float nominal_pos_, urdf_offset_,offset_pos_; // 默认位置
    float pos_, vel_, fft_;           // 当前 位置、速度、力矩
    float target_pos_;
    int id_, ec_id_, direction_;
#if defined(USE_TENSORRT)
    MotorInfo motor_data;
#endif
};

#endif // MOTOR_BASE_HPP