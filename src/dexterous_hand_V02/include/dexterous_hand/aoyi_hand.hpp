#ifndef AOYI_HAND_HPP_
#define AOYI_HAND_HPP_

// #include "dexterous_hand/OHandSerialAPI.h"
// #include "dexterous_hand/aoyi_can.hpp"
#include "dexterous_hand/aoyi_protocal.hpp"
#include "dexterous_hand/socket_can.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <libgen.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

// #define DEBUG_INFO
// #define PRINT_PARSE_ACK_INFO

#define LOGGER_NAME "AoyiHand"

// 定义自定义日志宏
#define HAND_LOG_INFO(logger, format, ...)                                                                                                 \
    RCLCPP_INFO(logger, "[%s](%s:%d) " format, __func__, basename((char *)__FILE__), __LINE__, ##__VA_ARGS__)

#define HAND_LOG_WARN(logger, format, ...)                                                                                                 \
    RCLCPP_WARN(logger, "[%s](%s:%d) " format, __func__, basename((char *)__FILE__), __LINE__, ##__VA_ARGS__)

#define HAND_LOG_ERROR(logger, format, ...)                                                                                                \
    RCLCPP_ERROR(logger, "[%s](%s:%d) " format, __func__, basename((char *)__FILE__), __LINE__, ##__VA_ARGS__)

// ros2 run your_package your_node --ros-args --log-level DEBUG
#define HAND_LOG_DEBUG(logger, format, ...)                                                                                                \
    RCLCPP_DEBUG(logger, "[%s](%s:%d) " format, __func__, basename((char *)__FILE__), __LINE__, ##__VA_ARGS__)

#ifndef MAX
#define MAX(a, b) (((a) < (b)) ? (b) : (a))
#endif

#ifndef CLAMP
#define CLAMP(x, out_min, out_max) (((x) < (out_min)) ? (out_min) : (((x) > (out_max)) ? (out_max) : (x)))
#endif

// 获取状态命令
#define GET_STATE_DATA_FLAG (SUB_CMD_GET_ANGLE | SUB_CMD_GET_CURRENT | SUB_CMD_GET_FORCE | SUB_CMD_GET_STATUS)
// #define GET_STATE_DATA_FLAG    (SUB_CMD_GET_ANGLE)

// #define SET_PID   // Uncomment this if you want to set PID

#define HAS_THUMB_ROOT_MOTOR

#define NUM_FINGERS (5)

#ifdef HAS_THUMB_ROOT_MOTOR
#define NUM_MOTORS (6)
#define THUMB_ROOT_ID (5)
#else
#define NUM_MOTORS (NUM_FINGERS)
#endif

#define THUMB_ROOT_POS_CNT 3

#define ANGLE_SCALE 100

#define NON_FORCE_CONTROL_VALUE (-1) // 力度控制 时的无效值
#define HAND_CMD_INTERVAL_TIME (2) //10ms

#ifdef SET_PID
static const float _pidGains[][4] = {{250.00, 2.00, 250.00, 1.00}, {250.00, 2.00, 250.00, 1.00}, {250.00, 2.00, 250.00, 1.00},
                                     {250.00, 2.00, 250.00, 1.00}, {250.00, 2.00, 250.00, 1.00},
#ifdef HAS_THUMB_ROOT_MOTOR
                                     {250.00, 2.00, 250.00, 1.00}
#endif
};
#endif

// ====================== 完全匹配文档 - 电机状态码 0~5 ======================
enum class MotorState : uint8_t
{
    STATUS_OPENING = 0,       // 正在展开
    STATUS_CLOSING = 1,       // 正在抓取
    STATUS_POS_REACHED = 2,   // 位置到位停止
    STATUS_OVER_CURRENT = 3,  // 电流保护停止
    STATUS_FORCE_REACHED = 4, // 力控到位停止
    STATUS_STUCK = 5          // 电机堵转停止
};

// ====================== 完全匹配文档 - 灵巧手控制模式 1/2 ======================
enum class ControlMode : uint8_t
{
    NONE_CONTROL = 0, // none
    ANGLE_CONTROL = 1, // 角度控制 0.0~100.0%
    FORCE_CONTROL = 2  // 力度控制 0~65535 mN
};

// 1. 定义枚举：手指类型（核心！调用时直接用这个枚举，语义清晰）
typedef enum
{
    THUMB_BEND_INDEX,   // 大拇指 弯曲
    INDEX_BEND_INDEX,   // 食指 弯曲
    MIDDLE_BEND_INDEX,  // 中指 弯曲
    RING_BEND_INDEX,    // 无名指 弯曲
    PINKY_BEND_INDEX,   // 小拇指 弯曲
    THUMB_ROTATE_INDEX, // 大拇指 旋转
    FINGER_TYPE_MAX     // 枚举边界，不用管
} Finger_Type;

typedef struct
{
    Finger_Type fingerType; //
    float zeroAngle;        // 手指展开时的角度 0
    float hundredAngle;     // 手指紧握时的角度 100
} HAND_FINGER_DEVS;

inline constexpr HAND_FINGER_DEVS handFingerDevs[] = {
    {THUMB_BEND_INDEX, 36.76f, 2.26f},    // 0:大拇指弯曲
    {INDEX_BEND_INDEX, 178.37f, 100.22f}, // 1:食指弯曲
    {MIDDLE_BEND_INDEX, 176.06f, 97.81f}, // 2:中指弯曲
    {RING_BEND_INDEX, 176.54f, 101.38f},  // 3:无名指弯曲
    {PINKY_BEND_INDEX, 174.86f, 98.84f},  // 4:小拇指弯曲
    {THUMB_ROTATE_INDEX, 0.00f, 90.00f}   // 5:大拇指旋转
};

#if 0
// 边界钳位宏
#define CLAMP_0_100(val) ((val) < 0 ? 0 : ((val) > 100 ? 100 : (val)))

// 限幅数值在  -1  ~  65535  区间内，超出自动修正
#define CLAMP_N1_TO_65535(val) ((val) < -1 ? -1 : ((val) > 65535 ? 65535 : (val)))

// 🌟 核心转换宏：一行搞定，零开销
// 用法：PERCENT2ANGLE(手指类型, 0~100归一值) → 返回实际角度
#define PERCENT2ANGLE(finger_idx, percent_val)                                                                                             \
    (handFingerDevs[finger_idx].zeroAngle -                                                                                                \
     ((handFingerDevs[finger_idx].zeroAngle - handFingerDevs[finger_idx].hundredAngle) * CLAMP_0_100(percent_val) / 100.0f))


// 🌟 核心转换宏：与PERCENT2ANGLE严格对称互逆
// 用法：ANGLE2PERCENT(手指类型, 实际浮点角度值) → 返回0~100的百分比归一值
#define ANGLE2PERCENT(finger_idx, angle_val)                                                                                               \
    CLAMP_0_100(((handFingerDevs[finger_idx].zeroAngle - (angle_val)) /                                                                    \
                 (handFingerDevs[finger_idx].zeroAngle - handFingerDevs[finger_idx].hundredAngle)) *                                       \
                100.0f)
#endif

template <typename T> inline T clamp_val(const T &val, const T &min_val, const T &max_val)
{
    if (val > max_val)
        return max_val;
    if (val < min_val)
        return min_val;
    return val;
}

struct HandState
{
    std::string frame_id; // left_hand / right_hand 区分左右手
    uint64_t timestamp;   // 时间戳
    std::vector<std::string> joint_names;
    std::array<float, NUM_MOTORS> position;   // 位置百分比 [0.0,100.0] %
    std::array<float, NUM_MOTORS> current;    // 电机电流 mA
    std::array<MotorState, NUM_MOTORS> state; // 电机状态码

    HandState()
        : frame_id(""), // 显式初始化frame_id为""
          timestamp(0)
    {
        position.fill(0.0);
        current.fill(0.0);
        state.fill(MotorState::STATUS_POS_REACHED);
        joint_names = {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};
    }
};

// ====================== 灵巧手控制指令数据结构 (控制指令) ======================
struct HandCmd
{
    std::string frame_id;                                  // left_hand / right_hand 区分左右手
    uint64_t timestamp;                                    // 时间戳
    std::vector<std::string> joint_names;                  // 自定义关节名称列表
    ControlMode control_mode = ControlMode::ANGLE_CONTROL; // 默认角度控制
    std::array<double, NUM_MOTORS> values;                 // 控制值，与joint_names一一对应

    HandCmd()
        : frame_id(""), // 显式初始化frame_id为""
          timestamp(0)
    {
        joint_names = {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};
        values.fill(0.0);
    }
};

// 单个关节电机的全量数据
struct HandMotors
{
    std::string motor_name; // 电机名字
    float curr_pos_percent; // 当前位置(百分比) 0.0~100.0 %
    float tar_pos_percent;  // 目标位置(百分比) 0.0~100.0 %

    float curr_pos_angle; // 电机当前位置角度 (物理角度)
    float tar_pos_angle;  // 电机目标位置角度 (物理角度)

    float curr_pos_adc; // 电机当前位置adc (物理角度)
    float tar_pos_adc;  // 电机目标位置adc (物理角度)

    float cur_current; // 电机当前电流 mA
    float cur_force;   // 电机力度控制值 0~65535 mN

    MotorState motor_state; // 电机当前状态
};

class AoyiHand {
  public:
    using Ptr = std::shared_ptr<AoyiHand>;

    explicit AoyiHand(std::shared_ptr<SocketCAN> can_ptr, const std::string &hand_name, uint8_t hand_id = 0x01, uint8_t master_id = 0x01)
        : can_ptr_(std::move(can_ptr)), hand_name_(hand_name), hand_id_(hand_id), master_id_(master_id)
    {
    }

    // 析构函数 无override，独立实现
    ~AoyiHand() = default;

    /**
     * @brief 设备错误码 强类型枚举
     */
    enum class DeviceErrorCode : unsigned char
    {
        // 协议分组
        REMOTE_ERR_PROTOCOL_WRONG_CRC = 0x01,         // 校验码错误
                                                      // 命令分组
        REMOTE_ERR_COMMAND_INVALID = 0x11,            // 无效的命令
        REMOTE_ERR_COMMAND_INVALID_BYTE_COUNT = 0x12, // 字节数不正确
        REMOTE_ERR_COMMAND_INVALID_DATA = 0x13,       // 无效的值
                                                      // 状态分组
        REMOTE_ERR_STATUS_INIT = 0x21,                // 正在等待初始化命令或者正在初始化
        REMOTE_ERR_STATUS_CALI = 0x22,                // 等待校正
        REMOTE_ERR_STATUS_STUCK = 0x23,               // 电机堵转
                                                      // 操作分组
        REMOTE_ERR_OP_FAILED = 0x31,                  // 操作失败
        REMOTE_ERR_SAVE_FAILED = 0x32                 // 保存失败
    };
    std::string GetDeviceErrorDesc(DeviceErrorCode err_code);

    bool init();
    // bool send_cmd(const HandCmd &cmd);
    bool get_hand_state(HandState &state);
    bool handle_can_frame(struct can_frame frame);
    void set_hand_angle(const std::vector<uint16_t> &joint_angle);
    void set_hand_force(const std::vector<float> &joint_force);
    void send_hand_state_cmd();
    void set_hand_ctl_mode(ControlMode mode);

    std::string get_hand_name() const
    {
        return hand_name_;
    }
    uint8_t get_hand_id() const
    {
        return hand_id_;
    }
    uint8_t get_master_id() const
    {
        return master_id_;
    }

  private:
    // 私有成员变量 全部内置，无protected，全部私有封装
    std::shared_ptr<SocketCAN> can_ptr_; // CAN通信指针
    std::string hand_name_;              // 手标识 left_hand/right_hand
    uint8_t hand_id_;                    // 【修改】原std::string → 改为 uint8_t 类型
    uint8_t master_id_;                  // 【新增】uint8_t 类型 主控ID 成员变量

    ControlMode control_mode;
    std::array<HandMotors, NUM_MOTORS> hand_motors;

    uint8_t data_flag_state = 0;
    uint8_t address_master;
    HAND_PROTOCOL_DECODER_STATE initial_state;
    uint16_t timeout; /* ms */
    uint8_t is_whole_packet = 0;
    HAND_PROTOCOL_DECODER_STATE decode_state;
    uint8_t packet_data[MAX_PROTOCOL_DATA_SIZE + 5]; /* node_id, own_id, command_id, byte_cnt, data[], lrc */
    uint8_t send_buf[MAX_PROTOCOL_DATA_SIZE + 7];    /* extra 7 bytes for header0, header1, address, master address, cmd, nb_data, lrc */
    uint8_t byte_count;                              /* data bytes left in packet */
    uint8_t remote_err;

    void force_control(uint8_t finger_id, uint16_t force_value);
    uint8_t HAND_SetFingerForceTarget(uint8_t hand_id, uint8_t finger_id, uint16_t force_target, uint8_t *remote_err);
    bool parse_hand_ack_msg();

#include "aoyi_hand.inl.hpp"
};

#endif // AOYI_HAND_HPP_