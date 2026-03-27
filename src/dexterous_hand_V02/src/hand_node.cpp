#include "dexterous_hand/hand_node.hpp"

// 类的构造函数实现 - 支持 单左手/单右手/双手 自适应模式
HandDriverNode::HandDriverNode() : Node("dexterous_hand_node")
{
    // ========== 1. 声明+读取ROS2节点参数，核心配置项 ==========
    // 参数名称：hand_mode，可选值：left_only / right_only / dual_hand，默认值：dual_hand
    this->declare_parameter<std::string>("hand_mode", "dual_hand");
    std::string hand_mode = this->get_parameter("hand_mode").as_string();

    bool enable_left = false;
    bool enable_right = false;

    // ========== 2. 根据参数初始化对应CAN总线 + 灵巧手 ==========
    if (hand_mode == "left_only" || hand_mode == "dual_hand")
    {
        enable_left = true;
        can0_ptr_ = std::make_shared<SocketCAN>(CAN_DEV_INDEX_0, CAN_BAUD_RATE);
        can0_ptr_->init();

        left_hand_ptr_ = std::make_shared<AoyiHand>(can0_ptr_, "left_hand", LEFT_HAND_ID, LEFT_MASTER_ID);
    }

    if (hand_mode == "right_only" || hand_mode == "dual_hand")
    {
        enable_right = true;
#ifdef USE_ONLY_ONE_CAN_BUS
        if (can0_ptr_ == nullptr)
        {
            can0_ptr_ = std::make_shared<SocketCAN>(CAN_DEV_INDEX_0, CAN_BAUD_RATE);
            can0_ptr_->init();
        }
        right_hand_ptr_ = std::make_shared<AoyiHand>(can0_ptr_, "right_hand", RIGHT_HAND_ID, RIGHT_MASTER_ID);
#endif
#ifdef USE_TWO_CAN_BUS
        can1_ptr_ = std::make_shared<SocketCAN>(CAN_DEV_INDEX_1, CAN_BAUD_RATE);
        can1_ptr_->init();
        right_hand_ptr_ = std::make_shared<AoyiHand>(can1_ptr_, "right_hand", RIGHT_HAND_ID, RIGHT_MASTER_ID);
#endif
    }

    // ========== 3. 灵巧手初始化 + 容错处理（单个失败不影响整体） ==========
    bool left_init_ok = true;
    bool right_init_ok = true;
    if (enable_left)
    {
        left_init_ok = left_hand_ptr_->init();
        if (!left_init_ok)
        {
            RCLCPP_ERROR(get_logger(), "Left Hand init failed!");
            left_hand_ptr_.reset(); // 初始化失败，重置为空指针
        }
    }
    if (enable_right)
    {
        right_init_ok = right_hand_ptr_->init();
        if (!right_init_ok)
        {
            RCLCPP_ERROR(get_logger(), "Right Hand init failed!");
            right_hand_ptr_.reset(); // 初始化失败，重置为空指针
        }
    }

    if ((enable_left && !left_init_ok) && (enable_right && !right_init_ok))
    {
        RCLCPP_ERROR(get_logger(), " All enabled hands init failed! Exit...");
        rclcpp::shutdown();
        return;
    }

    if (enable_left)
    {
        pub_left_feedback_ = this->create_publisher<HandFeedbackMsg>(PUB_FEEDBACK_HAND_LEFT, 10);
    }
    if (enable_right)
    {
        pub_right_feedback_ = this->create_publisher<HandFeedbackMsg>(PUB_FEEDBACK_HAND_RIGHT, 10);
    }

    if (enable_left)
    {
        sub_left_control_ = this->create_subscription<HandControlMsg>(
            SUB_CONTROL_HAND_LEFT, 10, std::bind(&HandDriverNode::left_control_cb, this, std::placeholders::_1));
    }
    if (enable_right)
    {
        sub_right_control_ = this->create_subscription<HandControlMsg>(
            SUB_CONTROL_HAND_RIGHT, 10, std::bind(&HandDriverNode::right_control_cb, this, std::placeholders::_1));
    }

    // ========== 6. 初始化定时器循环 ==========
    timer_ = create_wall_timer(1000ms, std::bind(&HandDriverNode::timer_callback, this));

    if (can0_ptr_)
    {
        can0_recv_thread_ = std::thread(&HandDriverNode::can0_recv_thread_func, this);
        HAND_LOG_INFO(this->get_logger(), "CAN0 receive thread started!");
    }
#ifdef USE_TWO_CAN_BUS
    if (can1_ptr_)
    {
        can1_recv_thread_ = std::thread(&HandDriverNode::can1_recv_thread_func, this);
        HAND_LOG_INFO(this->get_logger(), "CAN1 receive thread started!");
    }
#endif

#ifdef CONTROL_HAND_CUSTOM_FUN
    sub_custom_control_ = this->create_subscription<CustomControlMsg>(
        SUB_CONTROL_HAND_CUSTOM, 10, std::bind(&HandDriverNode::custom_control_cb, this, std::placeholders::_1));
#endif
}

HandDriverNode::~HandDriverNode()
{
    is_running_ = false;
    if (can0_recv_thread_.joinable())
    {
        can0_recv_thread_.join();
    }
#ifdef USE_TWO_CAN_BUS
    if (can1_recv_thread_.joinable())
    {
        can1_recv_thread_.join();
    }
#endif
    HAND_LOG_INFO(this->get_logger(), "All CAN receive threads stopped safely!");
    HAND_LOG_INFO(this->get_logger(), "HandDriverNode deinitialized!");
}

void HandDriverNode::can0_recv_thread_func()
{
    struct can_frame frame{};
    while (is_running_ && rclcpp::ok() && can0_ptr_)
    {
        // 阻塞读取CAN0帧，无数据休眠，有数据立即返回
        if (can0_ptr_->recv_frame(frame))
        {
            // 分发帧到【所有挂载在CAN0上的手部设备】做解析
            if (left_hand_ptr_)
            {
                left_hand_ptr_->handle_can_frame(frame);
            }
#ifdef USE_ONLY_ONE_CAN_BUS
            if (right_hand_ptr_)
            {
                right_hand_ptr_->handle_can_frame(frame);
            }
#endif
            // HAND_LOG_INFO(this->get_logger(), "CAN0 receive canframe!");
        }
        else
        {
            HAND_LOG_INFO(this->get_logger(), "CAN0 receive thread sleep 10s!");
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    HAND_LOG_INFO(this->get_logger(), "CAN0 receive thread exited!");
}

// CAN1 阻塞接收线程核心函数：读帧 → 分发解析
void HandDriverNode::can1_recv_thread_func()
{
    struct can_frame frame{};
    while (is_running_ && rclcpp::ok() && can1_ptr_)
    {
        if (can1_ptr_->recv_frame(frame))
        {
            // 分发帧到【所有挂载在CAN1上的手部设备】做解析
            if (right_hand_ptr_)
            {
                right_hand_ptr_->handle_can_frame(frame);
            }
            // HAND_LOG_INFO(this->get_logger(), "CAN0 receive canframe!");
        }
        else
        {
            HAND_LOG_INFO(this->get_logger(), "CAN1 receive thread sleep 10s!");
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
    }
    HAND_LOG_INFO(this->get_logger(), "CAN1 receive thread exited!");
}

void HandDriverNode::control_hand_cmd(const HandControlMsg::SharedPtr &msg, HandCmd &cmd, const std::shared_ptr<AoyiHand> &hand_ptr)
{
    std::vector<uint16_t> target_angle(NUM_MOTORS, 0);
    std::vector<float> target_force(NUM_FINGERS, 0);

    // 解析控制模式和对应的控制值
    cmd.control_mode = static_cast<ControlMode>(msg->mode);
    if ((cmd.control_mode != ControlMode::FORCE_CONTROL) && (cmd.control_mode != ControlMode::ANGLE_CONTROL))
    {
        HAND_LOG_WARN(this->get_logger(), "cmd.control_mode %d is not right, exit", (uint32_t)cmd.control_mode);
        return;
    }
    else
    {
        hand_ptr->set_hand_ctl_mode(cmd.control_mode);
    }

    for (size_t i = 0; i < NUM_MOTORS && i < msg->position.size(); ++i)
    {
        if (cmd.control_mode == ControlMode::ANGLE_CONTROL)
        {
            target_angle[i] = static_cast<uint16_t>(msg->position[i]);
        }

        if (cmd.control_mode == ControlMode::FORCE_CONTROL)
        {
            // 力控只需要控制5个指头
            if (i == NUM_FINGERS)
                break;
            target_force[i] = msg->force[i];
        }
    }

    if (cmd.control_mode == ControlMode::ANGLE_CONTROL)
    {
        // 角度控制逻辑
        hand_ptr->set_hand_angle(target_angle);
    }
    else if (cmd.control_mode == ControlMode::FORCE_CONTROL)
    {
        // 力控逻辑
        hand_ptr->set_hand_force(target_force);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "control_mode err mode: %d", (uint32_t)cmd.control_mode);
    }

    HAND_LOG_DEBUG(this->get_logger(), "%s hand recv control cmd, position size: %ld, mode: %u", cmd.frame_id.c_str(), msg->position.size(),
                   msg->mode);

    return;
}

// 左手控制回调实现 - 适配最新HandControl.msg
void HandDriverNode::left_control_cb(const HandControlMsg::SharedPtr msg)
{
    if (!left_hand_ptr_)
    {
        HAND_LOG_WARN(this->get_logger(), "left_hand_ptr_ is null, exit.");
        return; // 左手未启用/初始化失败，直接返回
    }

    HandCmd cmd;
    cmd.frame_id = "left_hand";
    cmd.control_mode = ControlMode::ANGLE_CONTROL;

    if (!msg->position.empty())
    {
        control_hand_cmd(msg, cmd, left_hand_ptr_);
    }
}

// 右手控制回调实现 - 适配自定义 HandControl.msg
void HandDriverNode::right_control_cb(const HandControlMsg::SharedPtr msg)
{
    if (!right_hand_ptr_)
    {
        HAND_LOG_WARN(this->get_logger(), "right_hand_ptr_ is null, exit.");
        return; // 右手未启用/初始化失败，直接返回
    }

    HandCmd cmd;
    cmd.frame_id = "right_hand";
    cmd.control_mode = ControlMode::ANGLE_CONTROL;
    if (!msg->position.empty())
    {
        control_hand_cmd(msg, cmd, right_hand_ptr_);
    }
}

#ifdef CONTROL_HAND_CUSTOM_FUN
void HandDriverNode::custom_control_cb(const CustomControlMsg::SharedPtr msg)
{
    // 根据接收到的数值打印不同日志
    switch (msg->data)
    {
        case 0:
        {
            HAND_LOG_INFO(this->get_logger(), "Received custom hand control command: %d (LOG INFO)", msg->data);
            std::vector<uint16_t> target_angle(NUM_MOTORS, 0);
            left_hand_ptr_->set_hand_angle(target_angle);
        }
        break;
        case 1:
        {
            HAND_LOG_INFO(this->get_logger(), "Received custom hand control command: %d (LOG INFO)", msg->data);
            std::vector<uint16_t> target_angle(NUM_MOTORS, 0);
            target_angle[THUMB_BEND_INDEX] = 30;
            target_angle[INDEX_BEND_INDEX] = 90;
            target_angle[MIDDLE_BEND_INDEX] = 90;
            target_angle[RING_BEND_INDEX] = 90;
            target_angle[PINKY_BEND_INDEX] = 90;

            left_hand_ptr_->set_hand_angle(target_angle);
        }
        break;
        case 2:
            HAND_LOG_INFO(this->get_logger(), "Received custom hand control command: %d (LOG WARN)", msg->data);
            break;
        default:
            HAND_LOG_WARN(this->get_logger(), "Received unknown command: %d (LOG ERROR)", msg->data);
            break;
    }
}
#endif

// 定时器循环实现
void HandDriverNode::timer_callback()
{
    HandState left_state, right_state;

    // 发送状态请求指令
#ifndef STOP_SEND_STATE_CMD
    if (left_hand_ptr_)
    {
        left_hand_ptr_->send_hand_state_cmd();
    }
    if (right_hand_ptr_)
    {
        right_hand_ptr_->send_hand_state_cmd();
    }
#endif
    // 获取灵巧手状态 ，并发布状态
    if (left_hand_ptr_ && left_hand_ptr_->get_hand_state(left_state))
    {
        publish_hand_state(left_state, pub_left_feedback_);
    }
    if (right_hand_ptr_ && right_hand_ptr_->get_hand_state(right_state))
    {
        publish_hand_state(right_state, pub_right_feedback_);
    }

    return;
}

// 发布灵巧手状态 适配自定义 HandFeedback.msg
void HandDriverNode::publish_hand_state(const HandState &state, rclcpp::Publisher<HandFeedbackMsg>::SharedPtr pub)
{
    HandFeedbackMsg fb_msg;

    // 填充ROS标准Header头信息【核心必写】
    fb_msg.header.stamp = this->get_clock()->now(); // ROS2节点当前时间戳，适配ROS规范
    fb_msg.header.frame_id = state.frame_id;        // 填充左右手标识: left_hand / right_hand

    // 清空消息中所有动态数组，防止残留旧数据（健壮性必备）
    fb_msg.name.clear();
    fb_msg.position.clear();
    fb_msg.current.clear();
    fb_msg.state.clear();

    // 核心：遍历赋值，将HandState的所有数据一一填充到ROS2消息的动态数组中
    for (size_t i = 0; i < NUM_MOTORS; ++i)
    {
        fb_msg.name.push_back(state.joint_names[i]);                  // 关节名称 string[]
        fb_msg.position.push_back(state.position[i]);                 // 关节位置百分比 float32[]
        fb_msg.current.push_back(state.current[i]);                   // 电机电流 mA float32[]
        fb_msg.state.push_back(static_cast<uint8_t>(state.state[i])); // 电机状态 uint8[] 强转匹配ROS msg类型
    }

    // 全发布ROS2消息（判空publisher，防止空指针崩溃）
    if (pub && pub->get_subscription_count() > 0)
    {
        pub->publish(fb_msg);
    }
}

// // 限幅函数
// float HandDriverNode::clamp_val(float val, float min_val, float max_val)
// {
//     return val < min_val ? min_val : (val > max_val ? max_val : val);
// }