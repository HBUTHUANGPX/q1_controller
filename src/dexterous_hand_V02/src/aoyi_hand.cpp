#include "dexterous_hand/aoyi_hand.hpp"
// #include "dexterous_hand/aoyi_protocal.hpp"

// 初始化函数
bool AoyiHand::init()
{

    // hand_id_ = hand_id;
    // 初始化 hand_motors赋值电机名、初始位置等
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        // hand_motors[i].motor_name = "motor_" + std::to_string(i);
        hand_motors[i].curr_pos_percent = 0.0f;
        hand_motors[i].tar_pos_percent = 0.0f;
        hand_motors[i].curr_pos_angle = 0.0f;
        hand_motors[i].tar_pos_angle = 0.0f;
        hand_motors[i].curr_pos_adc = 0.0f;
        hand_motors[i].tar_pos_adc = 0.0f;
        hand_motors[i].cur_current = 0.0f;
        hand_motors[i].cur_force = 0.0f;
        hand_motors[i].motor_state = MotorState::STATUS_POS_REACHED;
    }
    remote_err = 0;

    if (!can_ptr_)
    {
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "[CAN%d] receive thread started!", hand_id_);
        return false;
    }
    // return can_ptr_->init();

    // hand_ctx = nullptr;
    // port = nullptr;
    // setup();
    // std::cout << "[" << get_hand_id() << "] AoyiHand init success!" << std::endl;
    return true;
}


// ==================== 核心函数：角度转百分比 ====================
// 传入：手指索引(int) + 实际角度(float 如150.02)
// 返回：整型百分比【0 ~ 100】，自动四舍五入+边界钳位，完美匹配你的需求
int Angle2Percent(int finger_idx, float angle_val)
{
    // 1. 获取当前手指的角度配置参数
    const float zero_angle = handFingerDevs[finger_idx].zeroAngle;
    const float hundred_angle = handFingerDevs[finger_idx].hundredAngle;

    // 2. 核心转换公式，和宏定义逻辑完全一致，严格可逆PERCENT2ANGLE
    const float angle_diff = zero_angle - hundred_angle;
    float percent_val = ((zero_angle - angle_val) / angle_diff) * 100.0f;

    // 3. 边界钳位：强制限制在0~100之间，等价你的CLAMP_0_100宏
    if (percent_val < 0.0f)
        percent_val = 0.0f;
    if (percent_val > 100.0f)
        percent_val = 100.0f;

    // 关键修改：浮点转整型 + 四舍五入（工业控制最优选择，避免截断误差）
    return (int)(percent_val + 0.5f);
}

// ==================== 附赠：百分比转角度 函数版（配套完整） ====================
// 函数功能：传入手指索引 + 0~100百分比值 → 返回实际浮点角度值
float Percent2Angle(int finger_idx, float percent_val)
{
    const float zero_angle = handFingerDevs[finger_idx].zeroAngle;
    const float hundred_angle = handFingerDevs[finger_idx].hundredAngle;

    // 先钳位百分比，再计算角度
    if (percent_val < 0.0f)
        percent_val = 0.0f;
    if (percent_val > 100.0f)
        percent_val = 100.0f;

    float angle_val = zero_angle - ((zero_angle - hundred_angle) * percent_val / 100.0f);
    return angle_val;
}

/**
 * @brief
 *
 * 解析接收到的can帧数据，将解析后的数据赋值给hand_motors[]数组，
 * 方便上层获取灵巧手的状态。
 *
 * @return true/flase
 */
bool AoyiHand::parse_hand_ack_msg()
{

    uint8_t err, remote_err;
    uint8_t hand_id = get_hand_id();

    const uint8_t MAX_WR_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)    /* speed */
                                    + NUM_MOTORS * sizeof(uint16_t)  /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t); /* angle */

    const uint8_t MAX_RD_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)   /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t) /* angle */
                                    + NUM_MOTORS * sizeof(uint16_t) /* current */
                                    + NUM_MOTORS * sizeof(uint16_t) /* force */
                                    + NUM_MOTORS * sizeof(uint8_t); /* status */

    uint8_t data[MAX(MAX_WR_DATA_LEN, MAX_RD_DATA_LEN)];

    // uint8_t data_flag = SUB_CMD_GET_ANGLE;
    uint8_t data_flag = data_flag_state;

    uint8_t *p_data = data;

    uint8_t recv_data_size;
    recv_data_size = sizeof(data);

    if (is_whole_packet == 1)
    {
        // HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "angle received is_whole_packet: ");
        err = HAND_GetResponse(this, hand_id, HAND_CMD_SET_CUSTOM, 0, data, &recv_data_size, &remote_err);
        if (err == HAND_RESP_HAND_ERROR)
        {
            this->remote_err = remote_err;
            HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "hand_id %d, HAND_GetResponse err: %d, remote_err: %d\n", hand_id, err, remote_err);
            return false;
        }
        else if (err != HAND_RESP_SUCCESS)
        {
            HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "hand_id %d, HAND_GetResponse  err: %d\n",hand_id, err);
            return false;
        }

        p_data = data;

        if (data_flag & SUB_CMD_GET_POS)
        {
            uint16_t pos[NUM_MOTORS];

            // HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "pos received: ");
            for (int j = 0; j < NUM_MOTORS; j++)
            {
                pos[j] = *p_data++;
                pos[j] |= (*p_data++) << 8;

                // HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "%5d ", pos[j]);
            }

            // HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "\n");
        }

        if (data_flag & SUB_CMD_GET_ANGLE)
        {
            uint16_t angle[NUM_MOTORS];

            for (int j = 0; j < NUM_MOTORS; j++)
            {
                angle[j] = *p_data++;
                angle[j] |= (*p_data++) << 8;

                // printf("%5d ", angle[j]);
                hand_motors[j].curr_pos_angle = angle[j] / 100.0f;
                hand_motors[j].curr_pos_percent = Angle2Percent(j, angle[j] / 100.0f);
            }

#ifdef PRINT_PARSE_ACK_INFO
            HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "angle received: ");
            HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "hand_motors.curr_pos_angle[] %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f. \n",
                          hand_motors[THUMB_BEND_INDEX].curr_pos_angle, hand_motors[INDEX_BEND_INDEX].curr_pos_angle,
                          hand_motors[MIDDLE_BEND_INDEX].curr_pos_angle, hand_motors[RING_BEND_INDEX].curr_pos_angle,
                          hand_motors[PINKY_BEND_INDEX].curr_pos_angle, hand_motors[THUMB_ROTATE_INDEX].curr_pos_angle);
            HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()),
                          "hand_motors.curr_pos_percent[] %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f. \n",
                          hand_motors[THUMB_BEND_INDEX].curr_pos_percent, hand_motors[INDEX_BEND_INDEX].curr_pos_percent,
                          hand_motors[MIDDLE_BEND_INDEX].curr_pos_percent, hand_motors[RING_BEND_INDEX].curr_pos_percent,
                          hand_motors[PINKY_BEND_INDEX].curr_pos_percent, hand_motors[THUMB_ROTATE_INDEX].curr_pos_percent);
            printf("-------------------------------------------\n");
#endif
        }

        if (data_flag & SUB_CMD_GET_CURRENT)
        {
            uint16_t current[NUM_MOTORS];

            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "current received: ");
            for (int j = 0; j < NUM_MOTORS; j++)
            {
                current[j] = *p_data++;
                current[j] |= (*p_data++) << 8;

                HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "%5d ", current[j]);
                hand_motors[j].cur_current = current[j];
            }

            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "\n");
        }

        if (data_flag & SUB_CMD_GET_FORCE)
        {
            uint16_t force[NUM_MOTORS];

            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "force received: ");
            for (int j = 0; j < NUM_MOTORS; j++)
            {
                force[j] = *p_data++;
                force[j] |= (*p_data++) << 8;

                HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "%5d ", force[j]);
                hand_motors[j].cur_force = float(force[j]);
            }

            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "\n");
        }

        if (data_flag & SUB_CMD_GET_STATUS)
        {
            uint8_t status[NUM_MOTORS];

            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "status received: ");
            for (int j = 0; j < NUM_MOTORS; j++)
            {
                status[j] = *p_data++;

                HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "%5d ", status[j]);
                hand_motors[j].motor_state = static_cast<MotorState>(status[j]);
            }

#ifdef PRINT_PARSE_ACK_INFO
            HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "status received: ");
            HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "hand_motors.motor_state[] %u, %u, %u, %u, %u, %u. \n",
                          static_cast<uint8_t>(hand_motors[THUMB_BEND_INDEX].motor_state),
                          static_cast<uint8_t>(hand_motors[INDEX_BEND_INDEX].motor_state),
                          static_cast<uint8_t>(hand_motors[MIDDLE_BEND_INDEX].motor_state),
                          static_cast<uint8_t>(hand_motors[RING_BEND_INDEX].motor_state),
                          static_cast<uint8_t>(hand_motors[PINKY_BEND_INDEX].motor_state),
                          static_cast<uint8_t>(hand_motors[THUMB_ROTATE_INDEX].motor_state));
            printf("-------------------------------------------\n");
#endif
        }

        is_whole_packet = 0;
    }

    return true;
}

/**
 * @brief
 *
 * 初步处理过滤can帧数据
 * @param frame  can总线上接收到的can帧数据
 *
 * @return None
 */
bool AoyiHand::handle_can_frame(struct can_frame frame)
{
    // struct can_frame frame{};
    // if (!can_ptr_->recv_frame(frame))
    //     return false;

    ssize_t recv_len = frame.can_dlc;

    // ===================== 新增核心过滤逻辑 start =====================
    // 条件1: CAN帧数据域 第1字节 == 0x55  第2字节 == 0xAA
    // 条件2: CAN帧数据域 第6字节 == 0x00
    // 满足双条件 → 直接return false，不处理该帧
    // 位置角度控制 set_hand_angle(const std::vector<uint16_t> &joint_angle)
    // 信号反馈帧，不处理： 55 AA 01 02 5F 00 5C
    if ((frame.data[0] == 0x55) && (frame.data[1] == 0xAA) && (frame.data[5] == 0x00) && (recv_len == 7))
    {
        return false;
    }

    // ========== 完全对齐原代码的三种分支逻辑 ==========
    if (recv_len > 0)
    {
        // 成功接收数据，日志打印格式和原代码一模一样
        HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "Receive frame: ID=0x%03X, LEN=%d, DATA=", frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; i++)
        {
            HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "%02X ", frame.data[i]);
        }
        HAND_LOG_DEBUG(rclcpp::get_logger(get_hand_name()), "\n");

        // 核心：数据透传给机械手协议解析，逐字节调用HAND_OnData，和原逻辑一致
        for (int i = 0; i < frame.can_dlc; i++)
        {
            HAND_OnData(this, frame.data[i]);
        }
    }
    else if (recv_len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
        // 对应原代码的 PCAN_ERROR_QRCVEMPTY → 接收队列为空，直接返回，无报错
        return false;
    }
    else
    {
        // 其他错误，打印错误信息，和原代码逻辑一致
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "CAN_Read error, err: %s\n", strerror(errno));
        return false;
    }

    // 机械手协议数据包接收完整，调用解析函数
    if (is_whole_packet == 1)
    {
        parse_hand_ack_msg();
    }

    return true;
}

/**
 * @brief 灵巧手上层节点获取灵巧手状态
 *
 * 获取灵巧手状态，用于ros2 publish
 * @param state     返回获取灵巧手状态
 *
 * @return None
 */
bool AoyiHand::get_hand_state(HandState &state)
{
    state.frame_id = get_hand_name();
    state.timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    state.position.fill(0.0);
    state.current.fill(0.0);
    state.state.fill(MotorState::STATUS_POS_REACHED);
    // return true;

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        // 电机当前位置百分比 赋值给 状态position
        state.position[i] = hand_motors[i].curr_pos_percent;

        // 电机当前电流(mA) 赋值给 状态current
        state.current[i] = hand_motors[i].cur_current;

        // 电机当前状态码 赋值给 状态state
        state.state[i] = hand_motors[i].motor_state;
    }

    // handle_can_frame();

    return true;
}
/**
 * @brief 设置灵巧手设置模式
 *
 *  ANGLE_CONTROL = 1, // 角度控制 0.0~100.0%
 *  FORCE_CONTROL = 2  // 力度控制 0~65535 mN
 *
 * @return None
 */
void AoyiHand::set_hand_ctl_mode(ControlMode mode)
{
    control_mode = mode;
}

/**
 * @brief 设置手指张开度
 *
 * 设置所有手指张开度，范围0.0 ~ 100.0f，0表示完全张开，100表示完全握紧
 * @param joint_angle        参数joint_angle范围：0.0 ~ 100.0f。
 *
 * @return None
 */
void AoyiHand::set_hand_angle(const std::vector<uint16_t> &joint_angle)
{
    uint8_t err, remote_err;
    uint8_t hand_id = get_hand_id();

    const uint8_t MAX_WR_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)    /* speed */
                                    + NUM_MOTORS * sizeof(uint16_t)  /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t); /* angle */

    const uint8_t MAX_RD_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)   /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t) /* angle */
                                    + NUM_MOTORS * sizeof(uint16_t) /* current */
                                    + NUM_MOTORS * sizeof(uint16_t) /* force */
                                    + NUM_MOTORS * sizeof(uint8_t); /* status */

    uint8_t data[MAX(MAX_WR_DATA_LEN, MAX_RD_DATA_LEN)];

    uint8_t data_flag = SUB_CMD_SET_ANGLE;
    // uint8_t data_flag = SUB_CMD_SET_ANGLE | SUB_CMD_SET_SPEED;

    uint8_t send_data_size;
    uint8_t recv_data_size;

    uint8_t *p_data = data;
    *p_data++ = data_flag;

    // 安全校验：必须传入6个角度值，否则直接返回，防止数组越界访问崩溃
    if (joint_angle.size() != NUM_MOTORS)
    {
        return;
    }

    // 转换手指张开度百分比0 ~ 100 到手指实际角度值（单位：0.01度）
    uint16_t angle[NUM_MOTORS] = {static_cast<uint16_t>(Percent2Angle(THUMB_BEND_INDEX, joint_angle[THUMB_BEND_INDEX]) * ANGLE_SCALE),
                                  static_cast<uint16_t>(Percent2Angle(INDEX_BEND_INDEX, joint_angle[INDEX_BEND_INDEX]) * ANGLE_SCALE),
                                  static_cast<uint16_t>(Percent2Angle(MIDDLE_BEND_INDEX, joint_angle[MIDDLE_BEND_INDEX]) * ANGLE_SCALE),
                                  static_cast<uint16_t>(Percent2Angle(RING_BEND_INDEX, joint_angle[RING_BEND_INDEX]) * ANGLE_SCALE),
                                  static_cast<uint16_t>(Percent2Angle(PINKY_BEND_INDEX, joint_angle[PINKY_BEND_INDEX]) * ANGLE_SCALE),
                                  static_cast<uint16_t>(Percent2Angle(THUMB_ROTATE_INDEX, joint_angle[THUMB_ROTATE_INDEX]) * ANGLE_SCALE)};

#ifdef DEBUG_INFO
    HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "===== 手指角度数组 angle 打印 =====\r\n");
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        printf("angle[%d] = %d \r\n", i, angle[i]);
    }

    printf("\ndata_flag: 0x%02x\n", data_flag);
    printf("SUB_CMD_SET_SPEED   %d\n", (data_flag & SUB_CMD_SET_SPEED) ? 1 : 0);
    printf("SUB_CMD_SET_POS     %d\n", (data_flag & SUB_CMD_SET_POS) ? 1 : 0);
    printf("SUB_CMD_SET_ANGLE   %d\n", (data_flag & SUB_CMD_SET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_POS     %d\n", (data_flag & SUB_CMD_GET_POS) ? 1 : 0);
    printf("SUB_CMD_GET_ANGLE   %d\n", (data_flag & SUB_CMD_GET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_CURRENT %d\n", (data_flag & SUB_CMD_GET_CURRENT) ? 1 : 0);
    printf("SUB_CMD_GET_FORCE   %d\n", (data_flag & SUB_CMD_GET_FORCE) ? 1 : 0);
    printf("SUB_CMD_GET_STATUS  %d\n", (data_flag & SUB_CMD_GET_STATUS) ? 1 : 0);

    if (data_flag & SUB_CMD_SET_ANGLE)
    {

        printf("angle sent: ");
        uint8_t *p_data_tmp = p_data;
        for (int j = 0; j < NUM_MOTORS; j++)
        {
            *p_data_tmp++ = (uint8_t)angle[j];
            *p_data_tmp++ = (uint8_t)(angle[j] >> 8);

            printf("%5d ", angle[j]);
        }

        printf("\n");
    }
#endif
#if 0
    if (data_flag & SUB_CMD_SET_SPEED)
    {
        uint16_t speed[NUM_MOTORS] = { 1000, 1000, 1000, 1000, 1000 }; // 0 ~ 65535
        // uint16_t speed[NUM_MOTORS] = { 65535, 65535, 65535, 65535, 65535 }; // 0 ~ 65535

        printf("speed sent: ");

        for (int j = 0; j < NUM_MOTORS; j++)
        {
            *p_data++ = (uint8_t)speed[j];
            *p_data++ = (uint8_t)(speed[j] >> 8);

            printf("%5d ", speed[j]);
        }

        printf("\n");
    }
#endif

    if (data_flag & SUB_CMD_SET_ANGLE)
    {

        for (int j = 0; j < NUM_MOTORS; j++)
        {
            *p_data++ = (uint8_t)angle[j];
            *p_data++ = (uint8_t)(angle[j] >> 8);
        }
    }

    send_data_size = (uint8_t)(p_data - data);
    recv_data_size = sizeof(data);

    err = HAND_SetCustom(hand_id, data, send_data_size, &recv_data_size, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "HAND_SetCustom with hand id: %d returned failed, result: 0x%02x\n", hand_id,
                      err);
    }
}
/**
 * @brief 获取手指状态命令
 *
 * 获取手指状态命令
 * 在返回帧中包含手指的当前位置、当前角度、电流、力值和状态信息
 * @return None
 */
void AoyiHand::send_hand_state_cmd()
{
    uint8_t err, remote_err;
    uint8_t hand_id = get_hand_id();
    address_master = get_master_id();
    const uint8_t MAX_WR_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)    /* speed */
                                    + NUM_MOTORS * sizeof(uint16_t)  /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t); /* angle */

    const uint8_t MAX_RD_DATA_LEN = NUM_MOTORS * sizeof(uint16_t)   /* pos */
                                    + NUM_MOTORS * sizeof(uint16_t) /* angle */
                                    + NUM_MOTORS * sizeof(uint16_t) /* current */
                                    + NUM_MOTORS * sizeof(uint16_t) /* force */
                                    + NUM_MOTORS * sizeof(uint8_t); /* status */

    uint8_t data[MAX(MAX_WR_DATA_LEN, MAX_RD_DATA_LEN)];

    // uint8_t data_flag = SUB_CMD_GET_ANGLE | SUB_CMD_GET_CURRENT | SUB_CMD_GET_FORCE | SUB_CMD_GET_STATUS;
    uint8_t data_flag = GET_STATE_DATA_FLAG;
    data_flag_state = data_flag;

    uint8_t send_data_size;
    uint8_t recv_data_size;

    uint8_t *p_data = data;
    *p_data++ = data_flag;

#if 0 // def DEBUG_INFO
    HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "\ndata_flag: 0x%02x\n", data_flag);
    printf("SUB_CMD_SET_SPEED   %d\n", (data_flag & SUB_CMD_SET_SPEED) ? 1 : 0);
    printf("SUB_CMD_SET_POS     %d\n", (data_flag & SUB_CMD_SET_POS) ? 1 : 0);
    printf("SUB_CMD_SET_ANGLE   %d\n", (data_flag & SUB_CMD_SET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_POS     %d\n", (data_flag & SUB_CMD_GET_POS) ? 1 : 0);
    printf("SUB_CMD_GET_ANGLE   %d\n", (data_flag & SUB_CMD_GET_ANGLE) ? 1 : 0);
    printf("SUB_CMD_GET_CURRENT %d\n", (data_flag & SUB_CMD_GET_CURRENT) ? 1 : 0);
    printf("SUB_CMD_GET_FORCE   %d\n", (data_flag & SUB_CMD_GET_FORCE) ? 1 : 0);
    printf("SUB_CMD_GET_STATUS  %d\n", (data_flag & SUB_CMD_GET_STATUS) ? 1 : 0);
#endif
    send_data_size = (uint8_t)(p_data - data);
    recv_data_size = sizeof(data);

    // err = HAND_SetCustom(hand_ctx, hand_id, data, send_data_size, &recv_data_size, &remote_err);
    err = HAND_SetCustom(hand_id, data, send_data_size, &recv_data_size, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "hand_id id: %d returned failed, result: 0x%02x\n", hand_id, err);
    }

    return;
}

uint8_t AoyiHand::HAND_SetFingerForceTarget(uint8_t hand_id, uint8_t finger_id, uint16_t force_target, uint8_t *remote_err)
{
    uint8_t err;

    uint8_t data[sizeof(finger_id) + sizeof(force_target)];
    const uint8_t nb_data = sizeof(data);

    data[0] = finger_id;
    data[1] = (uint8_t)force_target;
    data[2] = (uint8_t)(force_target >> 8);

    err = HAND_SendCmd(hand_id, HAND_CMD_SET_FINGER_FORCE_TARGET, data, nb_data);

    return err;
}

/**
 * @brief 设置手指力控状态
 *
 * 设置手指力控状态
 * @param finger_id        指定手指ID
 * @param force_value      指定力控值 0~65535 mN
 *
 * @return None
 */
void AoyiHand::force_control(uint8_t finger_id, uint16_t force_value)
{
    uint8_t err, remote_err;
    uint8_t hand_id = get_hand_id();
    err = HAND_SetFingerForceTarget(hand_id, finger_id, force_value, &remote_err);
    if (err != HAND_RESP_SUCCESS)
    {
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "finger id: %d returned failed, result: 0x%02x\n", finger_id, err);
    }

#ifdef DEBUG_INFO
    HAND_LOG_INFO(rclcpp::get_logger(get_hand_name()), "===== force_control finger_id %d, force_value %d =====\r\n", finger_id,
                  force_value);
#endif
    // 退出力控模式，不需要通过位控方式退出，通过位置方式退出力控
    // if (force_value == 0)
    // {
    //     err = HAND_SetFingerPos(this, hand_id, finger_id, 0, 255, &remote_err);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(HAND_CMD_INTERVAL_TIME));
    //     if (err != HAND_RESP_SUCCESS)
    //     {
    //         HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "finger id: %d returned failed, result: 0x%02x\n", finger_id, err);
    //     }
    // }
}

/**
 * @brief 设置手指力控张开度
 *
 * 指定力控值 0~65535 mN
 * @param joint_force   参数joint_force范围：0~65535 mN
 *
 * @return None
 */
void AoyiHand::set_hand_force(const std::vector<float> &joint_force)
{
    uint8_t err = HAND_RESP_SUCCESS;
    uint8_t hand_id = get_hand_id();
    int16_t forceValue = 0;
    // 安全校验：必须传入6个角度值，否则直接返回，防止数组越界访问崩溃
    if (joint_force.size() != NUM_FINGERS)
    {
        // HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "joint_force.size: %ld not %d, exit.\n", joint_force.size(), NUM_FINGERS);
        return;
    }

    // reset force control
    err = HAND_SendCmd(hand_id, HAND_CMD_RESET_FORCE, NULL, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(HAND_CMD_INTERVAL_TIME));
    if (err != HAND_RESP_SUCCESS)
    {
        HAND_LOG_WARN(rclcpp::get_logger(get_hand_name()), "hand_id id: %d returned failed, result: 0x%02x\n", hand_id, err);
    }

    for (uint8_t i = 0; i < NUM_FINGERS; i++)
    {
        forceValue = static_cast<uint16_t>(clamp_val(joint_force[i], -1.0f, 65535.0f));
        if (forceValue != NON_FORCE_CONTROL_VALUE)
        {
            force_control(i, uint16_t(forceValue));
            // 添加1毫秒延时（C++11及以上标准，跨平台）
            std::this_thread::sleep_for(std::chrono::milliseconds(HAND_CMD_INTERVAL_TIME));
        }
    }
}


#if 0
std::string AoyiHand::GetDeviceErrorDesc(AoyiHand::DeviceErrorCode err_code)
{
    switch (err_code)
    {
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_PROTOCOL_WRONG_CRC:
            return "校验码错误";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_COMMAND_INVALID:
            return "无效的命令";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_COMMAND_INVALID_BYTE_COUNT:
            return "字节数不正确";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_COMMAND_INVALID_DATA:
            return "无效的值";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_STATUS_INIT:
            return "正在等待初始化命令或者正在初始化";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_STATUS_CALI:
            return "等待校正";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_STATUS_STUCK:
            return "电机堵转";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_OP_FAILED:
            return "操作失败";
        case AoyiHand::DeviceErrorCode::REMOTE_ERR_SAVE_FAILED:
            return "保存失败";
        default:
            return "未知错误";
    }
}

bool AoyiHand::remote_err()
{
    // 示例：模拟初始化失败，返回错误码
    DeviceErrorCode err = DeviceErrorCode::ERR_STATUS_INIT;
    if (err != DeviceErrorCode::ERR_PROTOCOL_WRONG_CRC)
    {
        RCLCPP_ERROR(logger_, "手爪初始化失败: %s", GetDeviceErrorDesc(err).c_str());
        return false;
    }
    return true;
}


// 发送指令函数
bool AoyiHand::send_cmd(const HandCmd &cmd)
{
    (void)cmd;
    // if(can_ptr_ == nullptr || hand_ctx == nullptr)
    // {
    //     std::cerr << "[" << get_hand_id() << "] send cmd failed, ctx null!" << std::endl;
    //     return false;
    // }

    // uint8_t buf[8] = {0};
    // buf[0] = static_cast<uint8_t>(cmd.control_mode);
    // buf[1] = static_cast<uint8_t>(cmd.joint_names.size());
    // float2bytes(cmd.values[0], &buf[2], cmd.control_mode == ControlMode::FORCE_CONTROL);
    // return can_ptr_->send_multi_frame(CAN_ID_CMD, buf, 8);

    uint8_t can_id = hand_id_;
    uint8_t master_id = master_id_;
    size_t data_len = 10;
    uint8_t data_reset[10] = {0x55, 0xAA, can_id, master_id, 0x54, 0x00, 0x57};

    uint8_t data_open[10] = {0x55, 0xAA, can_id, master_id, 0x47, 0x03, 0x00, 0xC8, 0x00, 0x8F};
    can_ptr_->send_multi_frame(can_id, data_reset, 7);
    can_ptr_->send_multi_frame(can_id, data_open, data_len);
    return true;
}
#endif