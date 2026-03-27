#ifndef FSM_MANAGER_HPP
#define FSM_MANAGER_HPP
#include "../include/q1_controller/data_store.hpp"
#include "q1_controller/msg/xbox_joy.hpp" // 自定义消息
#include "q1_controller/srv/state_transition.hpp"  // 替换为自定义
#include "std_msgs/msg/int32.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
enum class FSM_state : int
{
    err_state = -1,
    init_state = 0,    // 全身阻尼
    default_state = 1, // 按照yaml文件中的每个电机的default pos做pd控制
    default_state_wave = 2, // 按照yaml文件中的每个电机的default pos做pd控制
    default_state_greeting = 3, // 按照yaml文件中的每个电机的default pos做pd控制
    default_xsens_gmr = 4, // 按照yaml文件中的每个电机的default pos做pd控制
    default_vr_rp = 5, // 按照yaml文件中的每个电机的default pos做pd控制
    rl_run_state = 10, // RL control
};
struct button_pressed{
    bool lt_pressed = false;
    bool rt_pressed = false; // LT轴 > 0.4
    bool b_pressed = false;  // B按钮 (buttons[1])
    bool a_pressed = false;  // A按钮 (buttons[0])
    bool x_pressed = false;  // X按钮 (buttons[3])
    bool y_pressed = false;  // Y按钮 (buttons[4])
    bool start_pressed = false;  // start按钮 (buttons[11])
    bool back_pressed = false;  // back按钮 (buttons[10])
};
class FSM_manager
{
  private:
    /* data */
    rclcpp::Subscription<q1_controller::msg::XboxJoy>::SharedPtr joy_sub_; // 关节状态订阅器
    rclcpp::Node::SharedPtr node_;                                         // ROS节点指针
    Eigen::Vector3f cmd_3;
    FSM_state FSM_state_;
    std::shared_ptr<DataStore> data_store_;
    rclcpp::TimerBase::SharedPtr state_timer_; // 新增：状态切换定时器
    // 新增：按键状态变量，用于去抖动
    bool lt_pressed_,rt_pressed_; // LT轴 > 0.4
    bool b_pressed_;  // B按钮 (buttons[1])
    bool a_pressed_;  // A按钮 (buttons[0])
    bool start_pressed_;  // start按钮 (buttons[11])
    bool back_pressed_;  // start按钮 (buttons[11])
    void stateTransitionCallback();
    mutable std::mutex mutex_; // 互斥锁，确保线程安全
    button_pressed button_pressed_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;  // 状态发布
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscriber_;            // 关节状态订阅器
    rclcpp::Service<q1_controller::srv::StateTransition>::SharedPtr state_transition_srv_;  // 替换为自定义
    bool last_transition_success_;  // service 响应结果
    void stateTransitionServiceCallback(
        const std::shared_ptr<q1_controller::srv::StateTransition::Request> request,
        std::shared_ptr<q1_controller::srv::StateTransition::Response> response);
    
  public:
    FSM_manager(rclcpp::Node *node, std::shared_ptr<DataStore> data_store);
    void set_FSM_state(FSM_state set_state)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        FSM_state_ = set_state;
    };
    FSM_state get_FSM_state() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return FSM_state_;
    }
    void joyCallback(const q1_controller::msg::XboxJoy::SharedPtr msg);
    void setStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
};
#endif