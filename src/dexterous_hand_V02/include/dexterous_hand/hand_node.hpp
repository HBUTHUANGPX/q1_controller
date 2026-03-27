#ifndef DEXTEROUS_HAND_NODE_HPP_
#define DEXTEROUS_HAND_NODE_HPP_

#include "dexterous_hand/aoyi_hand.hpp"
#include "dexterous_hand/msg/hand_control.hpp"
#include "dexterous_hand/msg/hand_feedback.hpp"
#include "dexterous_hand/socket_can.hpp"
#include "std_msgs/msg/int32.hpp"

#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// 定义上层topic定义
#define PUB_FEEDBACK_HAND_LEFT "/feedback/hand/left"
#define PUB_FEEDBACK_HAND_RIGHT "/feedback/hand/right"

#define SUB_CONTROL_HAND_LEFT "/control/hand/left"
#define SUB_CONTROL_HAND_RIGHT "/control/hand/right"

// 自定义pub功能
#define CONTROL_HAND_CUSTOM_FUN
#ifdef CONTROL_HAND_CUSTOM_FUN
#define SUB_CONTROL_HAND_CUSTOM "/control/hand/custom"
#endif

// #define STOP_SEND_STATE_CMD     // 定义此宏，停止发送状态请求指令，正常情况下此宏不定义
// #define USE_ONLY_ONE_CAN_BUS    // 如果两只灵巧手，都挂载到同一条CAN总线上，则定义此宏，与USE_TWO_CAN_BUS互斥
#define USE_TWO_CAN_BUS         // 如果两只灵巧手，分别挂载到CAN0和CAN1总线上，则定义此宏，与USE_ONLY_ONE_CAN_BUS互斥

// 左灵巧手接CAN_DEV_INDEX_0, 右灵巧手接CAN_DEV_INDEX_1.
#define CAN_DEV_INDEX_0 "can0"
#define CAN_DEV_INDEX_1 "can1"
#define CAN_BAUD_RATE 1000000

#define LEFT_HAND_ID (102)   // 0x66
#define LEFT_MASTER_ID (103) // 0x67

#define RIGHT_HAND_ID (202)   // 0xca
#define RIGHT_MASTER_ID (203) // 0xcb

class HandDriverNode : public rclcpp::Node {
  public:
    HandDriverNode();
    ~HandDriverNode();

  private:
    std::shared_ptr<SocketCAN> can0_ptr_ = nullptr;
    std::shared_ptr<SocketCAN> can1_ptr_ = nullptr;
    std::shared_ptr<AoyiHand> left_hand_ptr_ = nullptr;
    std::shared_ptr<AoyiHand> right_hand_ptr_  = nullptr;

    using HandFeedbackMsg = dexterous_hand::msg::HandFeedback;
    rclcpp::Publisher<HandFeedbackMsg>::SharedPtr pub_left_feedback_, pub_right_feedback_;

    using HandControlMsg = dexterous_hand::msg::HandControl;
    rclcpp::Subscription<HandControlMsg>::SharedPtr sub_left_control_, sub_right_control_;

#ifdef CONTROL_HAND_CUSTOM_FUN
    using CustomControlMsg = std_msgs::msg::Int32;
    rclcpp::Subscription<CustomControlMsg>::SharedPtr sub_custom_control_;
    void custom_control_cb(const CustomControlMsg::SharedPtr msg);
#endif

    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<bool> is_running_ = {true};
    // CAN接收线程（一个总线一个线程）
    std::thread can0_recv_thread_;
    std::thread can1_recv_thread_;
    // CAN0阻塞接收线程函数：读can0帧 → 分发到所有挂载在can0的手
    void can0_recv_thread_func();
    // CAN1阻塞接收线程函数：读can1帧 → 分发到所有挂载在can1的手
    void can1_recv_thread_func();

    // void hand_state_loop();

    void left_control_cb(const HandControlMsg::SharedPtr msg);
    void right_control_cb(const HandControlMsg::SharedPtr msg);
    void control_hand_cmd(const HandControlMsg::SharedPtr& msg, HandCmd& cmd, const std::shared_ptr<AoyiHand>& hand_ptr);
    void timer_callback();

    void publish_hand_state(const HandState &state, rclcpp::Publisher<HandFeedbackMsg>::SharedPtr pub);
    float clamp_val(float val, float min_val, float max_val);

};

#endif // DEXTEROUS_HAND_NODE_HPP_