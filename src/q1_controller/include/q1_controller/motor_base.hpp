// 文件: motor_base.hpp
#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP
#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <cstdint>

#if defined(USE_TENSORRT)
#include "rt_comm_inipc/DoubleBuffer.hpp"
#include "rt_comm_inipc/MotorCommandAPI.hpp"
#include "rt_comm_inipc/MotorInfo.hpp"
#include "rt_comm_inipc/rt_ipc.hpp"
#else
// 电机信息结构体
enum class motor_type : uint8_t
{
    NONE = 99,
    A4310_P2_36 = 0,
    A4315_P2_36 = 1,
    A6408_P2_25 = 2,
    A6416_P2_25 = 3,
    A8112_P1_18 = 4,
    A8116_P1_18 = 5,
    A10020_P1_12 = 6,
    A10020_P2_24 = 7,
    Ti5_30_40 = 10,
    Ti5_40_52 = 11,
    Ti5_50_70 = 12,
    Ti5_60_80 = 13,
};
typedef char MotorTimestamp[32];
#pragma pack(1)
// 电机信息结构体
struct MotorInfo
{
    uint16_t id = 0;
    float kp = 0.0f;
    float kd = 0.0f;
    float pos = 0.0f;
    float vel = 0.0f;
    float tor = 0.0f;
    uint8_t status = 0;
    uint8_t errcode = 0;
    bool motor_cmd_flag = false;
    bool enable_cmd = false;
    uint64_t timestamp = 0; // 从epoch开始的微秒数
    motor_type type = motor_type::A4310_P2_36;
};
#pragma pack()
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
    MotorBase(const std::string &name, float kp, float kd, float max_torque,float trans_eff, float nominal_pos, float urdf_offset,
              int id, int ec_id, int direction,const std::string &_motor_type );

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
        return pos_ + urdf_offset_;
        // return pos_ - offset_pos_;
        // return pos_ + offset_pos_;
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
        pos_ = value * direction_;
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
    void writeCurrentTimeStamp(uint64_t micros)
    {
        micros_time = micros;
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
        return motor_data.pos;
    }
    void setTargetPos(float value) // 将action数据给入
    {
        motor_data.pos = (value + offset_pos_) * direction_; //  对于实机，此时需要进行方向转换
        // std::cout << name_ <<" offset_pos_: " << offset_pos_<<"\n";
    }
    void resetTargetPos(float value)
    {
        motor_data.pos = std::clamp(value, -3.f,3.f);
    }
    MotorInfo getMotorInfo()
    {
        return motor_data;
    }
    int getDirection() const
    {
        return direction_;
    }
    uint64_t getCurrentTimestampStrMicro()
    {
        auto now = std::chrono::system_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch());
        return us.count();
    }
    std::string microsToStringStream(uint64_t micros) 
    {
        uint64_t seconds = micros / 1000000;
        uint64_t micros_part = micros % 1000000;
        
        std::time_t time_sec = static_cast<std::time_t>(seconds);
        std::tm* local_time = std::localtime(&time_sec);
        
        if (!local_time) {
            return "时间转换失败";
        }
        
        std::ostringstream oss;
        oss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S")
            << "." << std::setfill('0') << std::setw(6) << micros_part;
        
        return oss.str();
    }
  protected:
    std::string name_;                // 电机名称
    float kp_;                        // P增益
    float kd_;                        // D增益
    float max_torque_;                // 最大扭矩
    float trans_eff_;
    float nominal_pos_, urdf_offset_,offset_pos_; // 默认位置
    float pos_, vel_, fft_;           // 当前 位置、速度、力矩
    float target_pos_;
    uint64_t micros_time;
    int id_, ec_id_, direction_;
    motor_type motor_type_;
    MotorInfo motor_data;
};

#endif // MOTOR_BASE_HPP