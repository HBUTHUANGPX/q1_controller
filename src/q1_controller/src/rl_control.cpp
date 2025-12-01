#include "../include/rl_control.hpp"

void rl_control::_init_deploy_module()
{   
    /* 初始化网络输入输出模块  */
    auto network_io = std::make_shared<MLPNetworkIO>(config_);
    printf("rl_control:network_io ok\r\n");
#if defined(USE_OPENVINO)
    inference_ = std::make_shared<OpenVINOInference>(config_, network_io, motor_manager_); // OpenVINO推理
    printf("rl_control:inference_ OpenVINOInference ok\r\n");
#elif defined(USE_ONNX)
    inference_ = std::make_shared<OnnxInference>(config_, network_io, motor_manager_); // Onnx推理
    printf("rl_control:inference_ OnnxInference ok\r\n");
#elif defined(USE_TENSORRT)
    inference_ = std::make_shared<TensorRTInference>(config_, network_io, motor_manager_); // TensorRT推理
    printf("rl_control:inference_  TensorRTInference ok\r\n");
#endif
    manager_ = std::make_shared<ObservationManager>(config_, data_store_); // 初始化ObservationManager
    printf("rl_control:manager_ ok\r\n");
}

void rl_control::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    /* 获取IMU 数据 角速度*/
    Eigen::VectorXf ang_vel(3);
    ang_vel(0) = msg->angular_velocity.x;
    ang_vel(1) = msg->angular_velocity.y;
    ang_vel(2) = msg->angular_velocity.z;
    data_store_->UpdateBaseAngularVelocities(ang_vel); // 更新Observation中角速度

        // RCLCPP_WARN(node_->get_logger(), "TransformStamped");
        // printf("to x:%7.4f y:%7.4f z:%7.4f w:%7.4f\n", transform_stamped.transform.rotation.x,
        //        transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
        // printf("pe x:%7.4f y:%7.4f z:%7.4f w:%7.4f\n", msg->orientation.x,
        //        msg->orientation.y, msg->orientation.z, msg->orientation.w);   
        
    Eigen::Quaternionf robot_quat(msg->orientation.w, msg->orientation.x,
                                    msg->orientation.y, msg->orientation.z);
        // Eigen::Matrix3f rotation_matrix_2 = robot_quat_2.toRotationMatrix();
        // Eigen::Vector3f rpy_2 = rotation_matrix_2.eulerAngles(0, 1, 2);  // Roll (X), Pitch (Y), Yaw (Z)，单位：弧度
        // printf("Roll: %7.4f, Pitch: %7.4f, Yaw: %7.4f (radians)\n", rpy_2.x(), rpy_2.y(), rpy_2.z());

        // Eigen::Quaternionf robot_quat(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
        //                               transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);
        // Eigen::Matrix3f rotation_matrix = robot_quat.toRotationMatrix();
        // Eigen::Vector3f rpy = rotation_matrix.eulerAngles(0, 1, 2);  // Roll (X), Pitch (Y), Yaw (Z)，单位：弧度
        // printf("Roll: %7.4f, Pitch: %7.4f, Yaw: %7.4f (radians)\n", rpy.x(), rpy.y(), rpy.z());


    // RCLCPP_WARN(node_->get_logger(), "robot_quat");
    if (fresh_reference_quat_flag)
    {
        // RCLCPP_WARN(node_->get_logger(), "fresh_reference_quat_flag");
        fresh_reference_quat_flag = false;
        reference_quat_ = robot_quat;
    }
    robot_quat = robot_quat.normalized(); // 确保四元数归一化
    // 计算参考四元数的Z轴旋转角度 theta = 2 * atan2(z, w)
    float theta = 2.0f * std::atan2(reference_quat_.z(), reference_quat_.w()); //计算参考四元数的偏航角yaw

    // 构造反向旋转四元数 q_comp = [cos(-theta/2), 0, 0, sin(-theta/2)]
    float cos_half = std::cos(-theta / 2.0f);
    float sin_half = std::sin(-theta / 2.0f);
    Eigen::Quaternionf q_comp(cos_half, 0.0f, 0.0f, sin_half); // 构建补偿四元数

    // 应用补偿：robot_quat = q_comp * robot_quat
    robot_quat = q_comp * robot_quat; //补偿旋转： 确保机器人当前朝向与参考方向一致

    // 确保结果归一化
    robot_quat = robot_quat.normalized();

    data_store_->UpdateRobotQuat(robot_quat); // 更新Observation中四元数
    // RCLCPP_WARN(node_->get_logger(), "data_store_");

    // RCLCPP_INFO(this->get_logger(), "Updated base angular velocities and quaternions from /mujoco_imu.");
}

void rl_control::setMujocoStateByDataset(size_t index)
{
    q1_controller::msg::Dataset msg;
    /* 填充数据集消息 */
    auto body_pos = motion_loader_->getBodyPosW(index);             // 获取body 位置数据 世界坐标系
    auto body_quat = motion_loader_->getBodyQuatW(index);           // 获取body 四元数数据 世界坐标系
    auto body_lin_vel = motion_loader_->getBodyLinVelW(index);      // 获取body 线速度数据 世界坐标系
    auto body_ang_vel = motion_loader_->getBodyAngVelW(index);      // 获取body 角速度数据 世界坐标系 xyzw
    auto joint_pos = motion_loader_->getJointPos(index);            // 获取关节位置数据
    auto joint_vel = motion_loader_->getJointVel(index);            // 获取关节速度数据
    auto ref_root_pos = body_pos.row(0);                            // 根节点位置
    auto ref_root_quat = body_quat.row(0);                          // 根节点四元数
    auto ref_root_lin_vel = body_lin_vel.row(0);                    // 根节点线速度
    auto ref_root_ang_vel = body_ang_vel.row(0);                    // 根节点角速度

    msg.root_position.x = ref_root_pos(0);                          // 根节点位置 x
    msg.root_position.y = ref_root_pos(1);                          // 根节点位置 y
    msg.root_position.z = ref_root_pos(2);                          // 根节点位置 z
    // std::cout << "ref_root_pos: " << ref_root_pos << std::endl;

    msg.root_quat.w = ref_root_quat(3) * 0 + 1;                     // 根节点四元数 w
    msg.root_quat.x = ref_root_quat(0) * 0;                         // 根节点四元数 x
    msg.root_quat.y = ref_root_quat(1) * 0;                         // 根节点四元数 y
    msg.root_quat.z = ref_root_quat(2) * 0;                         // 根节点四元数 z

    msg.root_velocity.x = ref_root_lin_vel(0) * 0;                  // 根节点线速度 x
    msg.root_velocity.y = ref_root_lin_vel(1) * 0;                  // 根节点线速度 y
    msg.root_velocity.z = ref_root_lin_vel(2) * 0;                  // 根节点线速度 z

    msg.root_angular_velocity.x = ref_root_ang_vel(0) * 0;          // 根节点角速度 x
    msg.root_angular_velocity.y = ref_root_ang_vel(1) * 0;          // 根节点角速度 y
    msg.root_angular_velocity.z = ref_root_ang_vel(2) * 0;          // 根节点角速度 z

    /* 填充关节状态 */
    msg.motor_state.position.reserve(joint_pos.size());
    for (Eigen::Index i = 0; i < joint_pos.size(); ++i)
    {
        msg.motor_state.position.push_back(static_cast<double>(joint_pos(i))); // 显式转换以确保清晰
    }
    /* 填充关节速度 */
    msg.motor_state.velocity.reserve(joint_vel.size());
    for (Eigen::Index i = 0; i < joint_vel.size(); ++i)
    {
        msg.motor_state.velocity.push_back(static_cast<double>(joint_vel(i))); // 显式转换以确保清晰
    }

    mocap_dataset_pub_->publish(msg); // 发布数据集消息
}

void rl_control::UpdateObs(float time_step)
{
    // RCLCPP_INFO(this->get_logger(), "UpdateObs.");
    try
    {
        // RCLCPP_INFO(node_->get_logger(), "try UpdateObservations");
        observations_ = manager_->UpdateObservations(time_step); // 更新Observations
        // std::cout << "行数: " << observations_.rows() << ", 列数: " << observations_.cols() << std::endl;
        // std::cout << observations_ << std::endl;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to update observations: %s", e.what());
    }
}

Eigen::VectorXf rl_control::inference()
{
    // RCLCPP_INFO(this->get_logger(), "Starting test update,time_step_ is :%d", static_cast<int>(time_step_));
    UpdateObs(time_step_); // 更新观测数据
    // RCLCPP_INFO(this->get_logger(), "UpdateObs");
    actions_ = inference_->UpdateAction(observations_, time_step_); // 获取Policy动作
    // RCLCPP_INFO(this->get_logger(), "actions_");
    data_store_->UpdateLastActions(actions_); // 更新Observation中最后动作
    // RCLCPP_INFO(this->get_logger(), "data_store_");
    // 使用MotorManager发布
    scaled_action = inference_->Scale_and_clamp_Action(actions_); // 缩放和裁剪动作
    // RCLCPP_INFO(this->get_logger(), "motor_manager_");
    time_step_ += 1.0; 
    // if (time_step_ >= 100.0f)
    if (time_step_ >= motion_loader_->getTimeStepTotal()) // 重置time_step_ 如果超过轨迹总时间步
    {
        time_step_ *= 0.0;
    }
    return scaled_action;
}

rl_control::rl_control(rclcpp::Node *node, const YAML::Node &config, std::shared_ptr<FSM_manager> fsm_manager,
                       std::shared_ptr<MotionLoader> motion_loader, std::shared_ptr<DataStore> data_store,
                       std::shared_ptr<MotorManager> motor_manager)
    : node_(node), fsm_manager_(fsm_manager), motion_loader_(motion_loader), data_store_(data_store),
      motor_manager_(motor_manager), time_step_(0), config_(config)
{
    printf("rl_control:rl_control ok\r\n");
    _init_deploy_module(); // 初始化部署模块
    reset_orientation_z_axis(); // 重置Z轴方向
    num_single_obs_ = config_["num_single_obs"].as<int>(); // 单帧观测维度
    frame_stack_ = config_["frame_stack"].as<int>();      // 帧堆叠数
    num_actions_ = config_["num_actions"].as<int>();    // 动作维度

    /* 初始化观测和动作 */
    observations_ = Eigen::MatrixXf::Zero(1, num_single_obs_ * frame_stack_);
    actions_ = Eigen::VectorXf::Zero(num_actions_); 
    scaled_action = Eigen::VectorXf::Zero(num_actions_); 

    /* 订阅IMU话题 */
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&rl_control::ImuCallback, this, std::placeholders::_1));
    if (!imu_sub_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create subscription: imu_sub_ is null");
    }
    
    
    reset_zero_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/reset_zero", 10); // 发布重置零位话题
    mocap_dataset_pub_ = node_->create_publisher<q1_controller::msg::Dataset>("/Dataset", 10); // 发布数据集话题
    std_msgs::msg::Bool flag;
    flag.data = true;
    reset_zero_pub_->publish(flag);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock()); // 创建TF缓冲区
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); //创建共享TF监听器
}

rl_control::~rl_control()
{
}
