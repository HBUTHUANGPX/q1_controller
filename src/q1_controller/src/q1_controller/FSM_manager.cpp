#include "../../include/q1_controller/FSM_manager.hpp"

FSM_manager::FSM_manager(rclcpp::Node *node, std::shared_ptr<DataStore> data_store)
    : node_(node), data_store_(data_store)
{
    printf("FSM_manager:==============\r\n");
    joy_sub_ = node_->create_subscription<q1_controller::msg::XboxJoy>(
        "/gamepad_data", 10, std::bind(&FSM_manager::joyCallback, this, std::placeholders::_1));
    printf("FSM_manager: joy_sub_ okr\n");

    // 新增：创建定时器，周期100ms，用于状态切换检查
    state_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&FSM_manager::stateTransitionCallback, this));
    printf("FSM_manager: state_timer_ okr\n");
    cmd_3 << 0.0f, 0.0f, 0.0f;
    FSM_state_ = FSM_state::init_state;
    // 初始化按键状态
    lt_pressed_ = false;
    b_pressed_ = false;
    a_pressed_ = false;
    state_transition_srv_ = node_->create_service<q1_controller::srv::StateTransition>(
        "/state_transition",
        std::bind(&FSM_manager::stateTransitionServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    printf("FSM_manager: state_transition_srv_ okr\n");
    state_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("/robot_state", 10);    
    state_subscriber_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/set_robot_state", 10, std::bind(&FSM_manager::setStateCallback, this, std::placeholders::_1));
        
    last_transition_success_ = false;
    printf("FSM_manager: okr\n");
}
void FSM_manager::stateTransitionServiceCallback(const std::shared_ptr<q1_controller::srv::StateTransition::Request> request,
                                                 std::shared_ptr<q1_controller::srv::StateTransition::Response> response)
{
    int requested_state = request->target_state; // 示例：请求状态
    bool success = false;
    std::string message = "";

    // 处理切换逻辑
    if (requested_state == static_cast<int>(FSM_state::default_state) && FSM_state_ == FSM_state::init_state)
    {
        success = true;
    }
    if (requested_state == static_cast<int>(FSM_state::rl_run_state) && FSM_state_ == FSM_state::default_state)
    {
        success = true;
    }
    if (requested_state == static_cast<int>(FSM_state::init_state) && FSM_state_ == FSM_state::default_state)
    {
        success = true;
    }
    if (requested_state == static_cast<int>(FSM_state::init_state) && FSM_state_ == FSM_state::rl_run_state)
    {
        success = true;
    } 
    if (requested_state == static_cast<int>(FSM_state::default_state) && FSM_state_ == FSM_state::rl_run_state)
    {
        success = true;
    } 

    if (success)
    {
        FSM_state_ = static_cast<FSM_state>(requested_state);
        message = "切换成功";
    }
    else
    {
        message = "切换失败";
    }

    response->success = success;    // 示例响应
    response->message = message;    // 示例响应
    last_transition_success_ = success; // 存储结果，用于 callback
    RCLCPP_INFO(node_->get_logger(), "服务响应: %s", message.c_str());
}
void FSM_manager::setStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    FSM_state state = static_cast<FSM_state>(msg->data);
    if (state == FSM_state::init_state)
    {
        FSM_state_ = FSM_state::init_state;
        RCLCPP_INFO(node_->get_logger(), "change state to init_state.");
    }
    else if (state == FSM_state::default_state)
    {
        FSM_state_ = FSM_state::default_state;
        RCLCPP_INFO(node_->get_logger(), "change state to default_state.");
    }
    else if (state == FSM_state::default_state_wave)
    {
        FSM_state_ = FSM_state::default_state_wave;
        RCLCPP_INFO(node_->get_logger(), "change state to default_state_wave.");
    }
    else if (state == FSM_state::rl_run_state)
    {
        FSM_state_ = FSM_state::rl_run_state;
        RCLCPP_INFO(node_->get_logger(), "change state to rl_run_state.");
    }
    else if (state == FSM_state::err_state)
    {
        FSM_state_ = FSM_state::err_state;
        RCLCPP_INFO(node_->get_logger(), "change state to err_state.");
    }
    
}
void FSM_manager::joyCallback(const q1_controller::msg::XboxJoy::SharedPtr msg)
{
    // RCLCPP_INFO(node_->get_logger(), "joyCallback.");
    // std::cout << "joyCallback" << std::endl;
    cmd_3(0) = -msg->axes[1]; // x
    cmd_3(1) = -msg->axes[0]; // y
    cmd_3(2) = -msg->axes[2]; // w

    // std::cout << "joyCallback 1" << std::endl;
    // 应用死区滤除，阈值设为0.1f
    const float deadzone = 0.05f;
    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(cmd_3(i)) < deadzone)
        {
            cmd_3(i) = 0.0f;
        }
    }
    if (msg->buttons[6] > 0)
    {
        cmd_3(1) *= 0.f;
    }
    if (msg->buttons[7] > 0)
    {
        cmd_3(0) *= 0.f;
    }
    // std::cout << "joyCallback 2" << std::endl;
    data_store_->UpdateCmdVel(cmd_3);
    // std::cout << "joyCallback 3" << std::endl;
    // std::cout << cmd_3.transpose() << std::endl;
    button_pressed_.lt_pressed = (msg->axes[5] > 0.4f);
    button_pressed_.rt_pressed = (msg->axes[4] > 0.4f);
    button_pressed_.b_pressed = (msg->buttons[1] > 0);      // 假设buttons[1]为B按钮，按下时>0
    button_pressed_.a_pressed = (msg->buttons[0] > 0);      // 假设buttons[0]为A按钮，按下时>0
    button_pressed_.x_pressed = (msg->buttons[3] > 0);      // 假设buttons[3]为A按钮，按下时>0
    button_pressed_.y_pressed = (msg->buttons[4] > 0);      // 假设buttons[4]为A按钮，按下时>0
    button_pressed_.start_pressed = (msg->buttons[11] > 0); // 假设buttons[11]为start按钮，按下时>0
    button_pressed_.back_pressed = (msg->buttons[10] > 0);  // 假设buttons[10]为back按钮，按下时>0

    // RCLCPP_INFO(node_->get_logger(), "joyCallback ok.");
}

void FSM_manager::stateTransitionCallback()
{
    // 在此处检查按键组合并执行状态切换
    // RCLCPP_INFO(node_->get_logger(), "stateTransitionCallback.");
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_transition_success_) {
        // 示例：如果成功，确认更新（实际根据需要调整）
        RCLCPP_INFO(node_->get_logger(), "基于 service 响应确认状态更新。");
        last_transition_success_ = false;  // 重置
    }
    if (button_pressed_.lt_pressed && button_pressed_.b_pressed)
    {
        if (FSM_state_ == FSM_state::rl_run_state)
        {
            FSM_state_ = FSM_state::default_state;
            RCLCPP_INFO(node_->get_logger(), "change state from rl_run_state to default_state.");
        }
    }
    else if (button_pressed_.lt_pressed && button_pressed_.a_pressed)
    {
        if (FSM_state_ == FSM_state::default_state) // 替换为您的实际条件
        {
            FSM_state_ = FSM_state::rl_run_state;
            RCLCPP_INFO(node_->get_logger(), "change state from default_state to rl_run_state.");
        }
    }
    else if (button_pressed_.lt_pressed && button_pressed_.start_pressed)
    {
        if (FSM_state_ == FSM_state::init_state)
        {
            FSM_state_ = FSM_state::default_state;
            RCLCPP_INFO(node_->get_logger(), "change state from init_state to default_state.");
        }
    }
    else if (button_pressed_.lt_pressed && button_pressed_.back_pressed)
    {
        FSM_state_ = FSM_state::init_state;
        RCLCPP_INFO(node_->get_logger(), "change state to init_state.");
    }
    else if(button_pressed_.rt_pressed && button_pressed_.a_pressed)
    {
        if (FSM_state_ == FSM_state::default_state) // 替换为您的实际条件
        {
            FSM_state_ = FSM_state::default_state_wave;
            RCLCPP_INFO(node_->get_logger(), "change state from default_state to default_state_wave.");
        }
    }
    else if(button_pressed_.rt_pressed && button_pressed_.b_pressed)
    {
        if (FSM_state_ == FSM_state::default_state) // 替换为您的实际条件
        {
            FSM_state_ = FSM_state::default_state_greeting;
            RCLCPP_INFO(node_->get_logger(), "change state from default_state to default_state_greeting.");
        }
    }


    else if(button_pressed_.rt_pressed && button_pressed_.x_pressed)
    {
        if (FSM_state_ == FSM_state::init_state) // 替换为您的实际条件
        {
            FSM_state_ = FSM_state::default_xsens_gmr;
            RCLCPP_INFO(node_->get_logger(), "change state from default_state to default_xsens_gmr.");
        }
    }

    else if(button_pressed_.rt_pressed && button_pressed_.y_pressed)
    {
        if (FSM_state_ == FSM_state::init_state) // 替换为您的实际条件
        {
            FSM_state_ = FSM_state::default_vr_rp;
            RCLCPP_INFO(node_->get_logger(), "change state from default_state to default_vr_rp.");
        }
    }
    std_msgs::msg::Int32 state_msg;
    state_msg.data = static_cast<int>(FSM_state_);
    state_publisher_->publish(state_msg);  // 假设有状态发布器
    // 可选：添加日志或通知，例如RCLCPP_INFO(node_->get_logger(), "Current FSM state: %d",
    // static_cast<int>(FSM_state_));
}
