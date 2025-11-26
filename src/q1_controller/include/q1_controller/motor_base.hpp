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
     * @param id 电机ID。
     * @param ec_id 电机在以太网中的ID。
     * @param direction 电机方向（1或-1）。
     */
    MotorBase(const std::string &name, float kp, float kd, float max_torque, float default_pos, int id, int ec_id,
              int direction);

    /**
     * @brief 虚析构函数，支持子类扩展。
     */
    virtual ~MotorBase() = default;

    /**
     * @brief 获取电机名称
     * @return 电机名称
     */
    const std::string &getName() const
    {
        return name_;
    }

    /**
     * @brief 获取P增益
     * @return P增益
     */
    float getKp() const
    {
        return kp_;
    }

    /**
     * @brief 获取D增益
     * @return D增益
     */
    float getKd() const
    {
        return kd_;
    }

    /**
     * @brief 获取最大扭矩
     * @return 最大扭矩
     */
    float getMaxTorque() const
    {
        return max_torque_;
    }

    /**
     * @brief 获取默认位置
     * @return 默认位置
     */
    float getDefaultPos() const
    {
        return default_pos_;
    }

    /**
     * @brief 获取当前电机状态
     * @return 当前电机位置相对于default_pos的位置
     */
    float getCurrentPos() const
    {
        return pos_-default_pos_;
    }

    /**
     * @brief 获取当前电机速度
     * @return 当前电机速度
     */
    float getCurrentVel() const
    {
        return vel_;
    }

    /**
     * @brief 获取当前电机力矩
     * @return 当前电机力矩
     */
    float getCurrentFFT() const
    {
        return fft_;
    }

    /**
     * @brief 获取电机ID
     * @return 电机ID
     */
    int getID() const
    {
        return id_;
    }

    /**
     * @brief 获取电机在以太网中的ID
     * @return 电机在以太网中的ID
     */
    int getEC_ID() const
    {
        return ec_id_;
    }

    /**
     * @brief 设置当前电机位置
     * @param value 位置值
     */
    void setCurrentPos(float value)
    {
        pos_ = value * direction_;
    }

    /**
     * @brief 设置当前电机速度
     * @param value 速度值
     */
    void setCurrentVel(float value)
    {
        vel_ = value * direction_;
    }

    /**
     * @brief 设置当前电机力矩
     * @param value 力矩值
     */
    void setCurrentFFT(float value)
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
    /**
     * @brief 获取目标位置
     * @return 目标位置
     */
    float getTargetPos()
    {
#if defined(USE_TENSORRT)
        return motor_data.pos;
#else
        return target_pos_;
#endif
    }

    /**
     * @brief 设置目标位置
     * @param value 目标位置值 x相对于default_pos的位置
     */
    void setTargetPos(float value) // 将action数据给入
    {
#if defined(USE_TENSORRT)
        motor_data.pos = value * direction_ + default_pos_; // 对于实机，此时需要进行方向转换
#else
        target_pos_ = value + default_pos_;
#endif
    }

    /**
     * @brief 重置目标位置
     */
    void resetTargetPos(float value)
    {
#if defined(USE_TENSORRT)
        motor_data.pos = value;
#else
        target_pos_ = value;
#endif
    }
#if defined(USE_TENSORRT)
    /**
     * @brief 获取MotorInfo结构体
     * @return MotorInfo结构体
     */
    MotorInfo getMotorInfo()
    {
        return motor_data;
    }
#endif
  protected:
    std::string name_;              // 电机名称
    float kp_;                      // P增益
    float kd_;                      // D增益
    float max_torque_;              // 最大扭矩
    float default_pos_;             // 默认位置
    float pos_, vel_, fft_;         // 当前 位置、速度、力矩
    float target_pos_;              // 目标位置
    int id_, ec_id_, direction_;    // 电机ID，以太网ID，方向
#if defined(USE_TENSORRT)
    MotorInfo motor_data;            // MotorInfo结构体，用于TensorRT
#endif
};

#endif // MOTOR_BASE_HPP