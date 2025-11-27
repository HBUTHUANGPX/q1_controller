// 文件: motor_manager.cpp
#include "../../include/q1_controller/motor_manager.hpp"
#include <rclcpp/logger.hpp>
/**
 * @brief MotorManager构造函数实现。
 */
MotorManager::MotorManager(rclcpp::Node *node, const YAML::Node &config, std::shared_ptr<DataStore> data_store)
    : node_(node), data_store_(data_store)
{
    printf("MotorManager:start init\r\n");
    // 从YAML加载motors
    if (!config["motors"].IsSequence())
    {
        throw std::runtime_error("YAML 'motors' must be a map.");
    }

    printf("MotorManager:config ok\r\n");
    motors_.reserve(config["motors"].size());
    std::cout << "motor size: " << config["motors"].size() << std::endl;
    ankle_pitch_add_angle_ = config["ankle_pitch_add_angle"].as<float>();

    size_t index = 0;
    for (const auto &item : config["motors"])
    {
        // 检查item是否为Map且仅有一个键值对
        if (!item.IsMap() || item.size() != 1)
        {
            throw std::runtime_error("Each 'motors' item must be a map with exactly one key-value pair.");
        }

        auto map_iter = item.begin();
        std::string name = map_iter->first.as<std::string>(); // 键作为名称
        YAML::Node params = map_iter->second;                 // 值作为参数节点

        /* 导入motor 参数 */
        float kp = params["kp"].as<float>();
        float kd = params["kd"].as<float>();
        float max_torque = params["torque_max"].as<float>();
        float default_pos = params["default_pos"].as<float>();
        int id = params["id"].as<int>();
        int ec_id = params["ec_id"].as<int>();
        int direction = params["direction"].as<int>();

        /* 创建电机对象并添加到列表 */
        auto motor = std::make_shared<MotorBase>(name, kp, kd, max_torque, default_pos, id, ec_id, direction);
        motors_.push_back(motor);
        name_to_index_[name] = index++;
    }
    motors_in_id_ = motors_;
    // 按ID升序排序motors_in_id_
    std::sort(motors_in_id_.begin(), motors_in_id_.end(),
              [](const std::shared_ptr<MotorBase> &a, const std::shared_ptr<MotorBase> &b) {
                  return a->getID() < b->getID();
              });
    indices_from_motors = findMutualIndices(motors_, motors_in_id_);
    indices_from_motors_in_id = findMutualIndices(motors_in_id_, motors_);

    // for (size_t i = 0; i < motors_.size(); ++i)
    // {
    //     std::cout << "motor " << motors_[i]->getName() << ": " << motors_[i]->getID() << ", " <<
    //     indices_from_motors[i]
    //               << std::endl;
    // }
    index = 0;
    for (size_t i = 0; i < motors_in_id_.size(); ++i)
    {
        name_to_index_in_id_[motors_in_id_[i]->getName()] = index++;
        // std::cout << "motor " << motors_in_id_[i]->getName() << ": " << motors_in_id_[i]->getID() << ", "
        //           << indices_from_motors_in_id[i] << std::endl;
    }

    // 读取YAML中的joint_index_in_need字段（字符串数组）
    if (!config["joint_index_in_need"].IsSequence())
    {
        throw std::runtime_error("YAML 'joint_index_in_need' must be a sequence.");
    }
    for (const auto &item : config["joint_index_in_need"])
    {
        joint_index_in_need.push_back(item.as<std::string>());
    }

    // 创建map存放索引：键为关节名称，值为对应名字的motor在motors_中的索引
    std::cout << "joint_index_in_need: [";
    for (const auto &name : joint_index_in_need)
    {
        auto it = name_to_index_.find(name);
        if (it != name_to_index_.end())
        {
            joint_indices_in_motors[name] = it->second;
            std::cout << it->second << " ";
        }
        else
        {
            throw std::runtime_error("Motor name '" + name + "' in 'joint_index_in_need' not found in motors.");
        }
    }
    std::cout << "]\n";
    current_pos_ = Eigen::VectorXf::Zero(joint_indices_in_motors.size()); 
    current_vel_ = Eigen::VectorXf::Zero(joint_indices_in_motors.size());
    std::cout << "motor size: " << joint_indices_in_motors.size() << std::endl;
#if defined(USE_TENSORRT)
    control_data.resize(motors_.size());
    // 初始化每个元素（示例）
    for (size_t i = 0; i < control_data.size(); ++i)
    {
        control_data[i].kp = motors_in_id_[i]->getKp(); // 获取P增益
        control_data[i].kd = motors_in_id_[i]->getKd(); // 获取D增益
    }
#endif
    printf("MotorManager:config joint ok\r\n");
    std::string deploy_mode_ = config["deploy_mode"].as<std::string>(); // sim2sim or sim2real
    std::string sim2sim = "sim2sim";
    std::string sim2real = "sim2real";
    if (sim2sim == deploy_mode_) // mujoco仿真
    {
        // 创建订阅器
        joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/mujoco_joint_states", 10, std::bind(&MotorManager::jointCallback, this, std::placeholders::_1));
        if (!joint_sub_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create subscription: joint_sub_ is null");
        }
        state_ctrl_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_ctrls", 10);
    }
    else if (sim2real == deploy_mode_)
    {
        /* code */
        state_recv_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); //创建真实电机状态的发布器
        state_ctrl_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_ctrls", 10);
    }
    else
    {
        throw std::runtime_error("MotorManager: 参数文件错误配置 deploy mode为 " + deploy_mode_ +
                                 ",请检查并核对.yaml文件中的 'deploy_mode' 字段");
    }
    // 创建发布器
    target_pos_pub_ = node_->create_publisher<q1_controller::msg::MultiMotorCommand>("/target_pos", 10); //创建目标位置发布器
}

/**
 * @brief 通过名称获取指定电机。
 */
std::shared_ptr<MotorBase> MotorManager::getMotorByName(const std::string &name) const
{
    auto it = name_to_index_.find(name);
    if (it != name_to_index_.end())
    {
        return motors_[it->second];
    }
    return nullptr;
}
/**
 * @brief 通过名称获取指定电机。
 */
std::shared_ptr<MotorBase> MotorManager::getMotorByName_in_id(const std::string &name) const
{
    auto it = name_to_index_in_id_.find(name);
    if (it != name_to_index_in_id_.end())
    {
        return motors_in_id_[it->second];
    }
    return nullptr;
}
std::shared_ptr<MotorBase> MotorManager::getMotorByID(int ID) const
{
    // TODO：不是很想写，有需要再弄 ok
}
/**
 * @brief 通过索引获取电机。
 * @param index 索引
 */
std::shared_ptr<MotorBase> MotorManager::getMotorByIndex(size_t index) const
{
    if (index < motors_.size())
    {
        return motors_[index];
    }
    return nullptr;
}

/**
 * @brief 处理JointState消息，更新当前状态。
 * @param message JointState消息字符串。
 * @param from_port 消息来源端口。
 */
void MotorManager::jointStateUpdate(const std::string &message, int from_port)
{
    // 转换为二进制数据
    std::vector<uint8_t> binary_data(message.begin(), message.end());

    // 反序列化
    // printf("MotorManager:反序列化\r\n");
#if defined(USE_TENSORRT)
    std::vector<MotorInfo> received_motors;
    if (MotorSerializer::deserialize_array(binary_data, received_motors)) //反序列化成功
    {
        for (size_t i = 0; i < received_motors.size(); i++)
        {
            motors_in_id_[i]->setCurrentPos(received_motors[i].pos);
            motors_in_id_[i]->setCurrentVel(received_motors[i].vel);
            motors_in_id_[i]->setCurrentFFT(received_motors[i].tor);
        }
        // sensor_msgs::msg::JointState msg;
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        // received_motors存放的是真实关节电机的关节角度和角速度
        // 现在需要将MotorBase中的“ankle_pitch“ 和 “ankle_roll“ 中存放的AB电机的数据拿出来
        // 然后通过FK计算得到虚拟关节“ankle_pitch“和 “ankle_roll“的关节角度和角速度
        // 现在假设.yaml中的 _ankle_pitch_joint 放的是A电机，也就是theta
        // 现在假设.yaml中的 _ankle_roll_joint 放的是B电机，也就是phi
        // 上A下B
        /* 踝关节前向动力学 */
        std::shared_ptr<MotorBase> L_ankle_pitch_joint_ = getMotorByName("L_ankle_pitch_joint");
        std::shared_ptr<MotorBase> L_ankle_roll_joint_ = getMotorByName("L_ankle_roll_joint");
        std::shared_ptr<MotorBase> R_ankle_pitch_joint_ = getMotorByName("R_ankle_pitch_joint");
        std::shared_ptr<MotorBase> R_ankle_roll_joint_ = getMotorByName("R_ankle_roll_joint");
        float L_theta_ = L_ankle_pitch_joint_->getCurrentPos() + ankle_pitch_add_angle_ * 0;
        float L_phi_ = -L_ankle_roll_joint_->getCurrentPos() + ankle_pitch_add_angle_ * 0;
        float L_theta_dot_ = L_ankle_pitch_joint_->getCurrentVel();
        float L_phi_dot_ = -L_ankle_roll_joint_->getCurrentVel();

        float R_theta_ = -R_ankle_pitch_joint_->getCurrentPos() + ankle_pitch_add_angle_ * 0;
        float R_phi_ = R_ankle_roll_joint_->getCurrentPos() + ankle_pitch_add_angle_ * 0;
        float R_theta_dot_ = -R_ankle_pitch_joint_->getCurrentVel();
        float R_phi_dot_ = R_ankle_roll_joint_->getCurrentVel();

        Eigen::Vector2f fk_L_ankle = avm_.ankle_fk(L_phi_, L_theta_);
        Eigen::Vector2f fk_R_ankle = avm_.ankle_fk(R_phi_, R_theta_);
        Eigen::Vector2f vm_L_ankle =
            avm_.velocity_mapping(L_phi_, L_theta_, fk_L_ankle(0), fk_L_ankle(1), L_phi_dot_, L_theta_dot_);
        Eigen::Vector2f vm_R_ankle =
            avm_.velocity_mapping(R_phi_, R_theta_, fk_R_ankle(0), fk_R_ankle(1), R_phi_dot_, R_theta_dot_);
        L_ankle_pitch_joint_->setCurrentPos(fk_L_ankle(0) - ankle_pitch_add_angle_ * 0);
        L_ankle_pitch_joint_->setCurrentVel(vm_L_ankle(0));
        L_ankle_roll_joint_->setCurrentPos(-fk_L_ankle(1));
        L_ankle_roll_joint_->setCurrentVel(-vm_L_ankle(1));

        R_ankle_pitch_joint_->setCurrentPos(fk_R_ankle(0) + ankle_pitch_add_angle_ * 0);
        R_ankle_pitch_joint_->setCurrentVel(vm_R_ankle(0));
        R_ankle_roll_joint_->setCurrentPos(fk_R_ankle(1));
        R_ankle_roll_joint_->setCurrentVel(vm_R_ankle(1));

        for (size_t i = 0; i < motors_.size(); i++)
        {
            msg->position.push_back(motors_[i]->getCurrentPos());
            msg->velocity.push_back(motors_[i]->getCurrentVel());
            msg->effort.push_back(motors_[i]->getCurrentFFT());
            msg->name.push_back(motors_[i]->getName());
        }
        size_t i = 0;
        for (const auto &name : joint_index_in_need)
        {
            size_t motor_index = joint_indices_in_motors[name];
            current_pos_[i] = motors_[motor_index]->getCurrentPos();
            current_vel_[i] = motors_[motor_index]->getCurrentVel();
            ++i;
        }

        msg->header.stamp = node_->now();
        // std::cout<<msg.header.stamp<<std::endl;
        data_store_->UpdateJointPositions(current_pos_);
        data_store_->UpdateJointVelocities(current_vel_);
        state_recv_pub_->publish(std::move(msg));
        // printf("MotorManager:state_recv_pub_\r\n");
    }
    else
    {
        std::cerr << "Failed to deserialize motor data" << std::endl;
    }
#endif
}

/**
 * @brief 发布目标位置命令。
 * @param actions 动作向量（目标位置）。
 * @param zero_kp 是否将P增益置零。
 * @param zero_kd 是否将D增益置零。
 */
void MotorManager::publishTargetPos(const Eigen::VectorXf &actions, bool zero_kp, bool zero_kd)
{
    // RCLCPP_WARN(node_->get_logger(), "publishTargetPos.");
    if (static_cast<size_t>(actions.size()) != joint_indices_in_motors.size())
    {
        RCLCPP_WARN(node_->get_logger(), "Actions size mismatch for joint_index_in_need.");
        return;
    }
    // std::cout<<actions<<std::endl;
    auto msg = std::make_unique<q1_controller::msg::MultiMotorCommand>();
    msg->commands.reserve(motors_.size());
    size_t i = 0;
    for (const auto &name : joint_index_in_need)
    {
        size_t motor_index = joint_indices_in_motors[name];
        motors_[motor_index]->setTargetPos(actions(i));
        ++i;
    }

    // RCLCPP_WARN(node_->get_logger(), "踝关节ik计算.");
    { // 踝关节ik计算
        std::shared_ptr<MotorBase> L_ankle_pitch_joint_ = getMotorByName_in_id("L_ankle_pitch_joint");
        std::shared_ptr<MotorBase> L_ankle_roll_joint_ = getMotorByName_in_id("L_ankle_roll_joint");
        std::shared_ptr<MotorBase> R_ankle_pitch_joint_ = getMotorByName_in_id("R_ankle_pitch_joint");
        std::shared_ptr<MotorBase> R_ankle_roll_joint_ = getMotorByName_in_id("R_ankle_roll_joint");
        float L_alpha = L_ankle_pitch_joint_->getTargetPos();
        float L_beta = -L_ankle_roll_joint_->getTargetPos();
        auto L_result = avm_.ankle_ik(L_alpha, L_beta);
        float L_ik_phi = L_result.first;
        float L_ik_theta = L_result.second;
        L_ankle_pitch_joint_->resetTargetPos(L_ik_theta); // A
        L_ankle_roll_joint_->resetTargetPos(-L_ik_phi);   // B

        float R_alpha = R_ankle_pitch_joint_->getTargetPos();
        float R_beta = R_ankle_roll_joint_->getTargetPos();
        auto R_result = avm_.ankle_ik(R_alpha, R_beta);
        float R_ik_phi = R_result.first;
        float R_ik_theta = R_result.second;
        R_ankle_pitch_joint_->resetTargetPos(-R_ik_theta); // A
        R_ankle_roll_joint_->resetTargetPos(R_ik_phi);     // B
        // control_data的pos存放的是虚拟关节ankle_pitch和ankle_roll的数据
        // 现在需要将数据拿出来进行ik计算得到实际关节电机的pos
        // 现在假设.yaml中的 _ankle_pitch_joint 放的是A电机，也就是theta
        // 现在假设.yaml中的 _ankle_roll_joint 放的是B电机，也就是phi
        // TODO： 更新文档时，将
    }
    // RCLCPP_WARN(node_->get_logger(), "踝关节ik计算 ok.");

    auto JointState_msg = std::make_unique<sensor_msgs::msg::JointState>();
    for (size_t i = 0; i < motors_.size(); ++i)
    {
        JointState_msg->position.push_back(motors_[i]->getTargetPos());
        JointState_msg->name.push_back(motors_[i]->getName());
    }
    JointState_msg->header.stamp = node_->now();
    state_ctrl_pub_->publish(std::move(JointState_msg));

    // RCLCPP_WARN(node_->get_logger(), "state_ctrl_pub_ ok.");

    std::string L_ankle_pitch, L_ankle_roll, R_ankle_pitch, R_ankle_roll;
    L_ankle_pitch = "L_ankle_pitch_joint";
    L_ankle_roll = "L_ankle_roll_joint";
    R_ankle_pitch = "R_ankle_pitch_joint";
    R_ankle_roll = "R_ankle_roll_joint";
    for (size_t i = 0; i < motors_in_id_.size(); ++i)
    {
        q1_controller::msg::SingleMotorCommand cmd;
        auto urdf_name = motors_in_id_[i]->getName();
        if (urdf_name == L_ankle_pitch)
        {
            cmd.motor_name = "L_Flange_A_joint";
        }
        else if (urdf_name == L_ankle_roll)
        {
            cmd.motor_name = "L_Flange_B_joint";
        }
        else if (urdf_name == R_ankle_pitch)
        {
            cmd.motor_name = "R_Flange_A_joint";
        }
        else if (urdf_name == R_ankle_roll)
        {
            cmd.motor_name = "R_Flange_B_joint";
        }
        else
        {
            cmd.motor_name = urdf_name;
        }

        cmd.target_pos = motors_in_id_[i]->getTargetPos();
        cmd.target_vel = 0.0; // 默认0
        if (zero_kp)
        {
            cmd.p_gain = 0;
        }
        else
        {
            cmd.p_gain = motors_in_id_[i]->getKp();
        }
        if (zero_kd)
        {
            cmd.d_gain = 0;
        }
        else
        {
            cmd.d_gain = motors_in_id_[i]->getKd();
        }
        cmd.ff_effort = 0.0; // 默认0
        msg->commands.push_back(cmd);
#if defined(USE_TENSORRT)
        control_data.at(i) = motors_in_id_[i]->getMotorInfo();
#endif
    }
    target_pos_pub_->publish(std::move(msg));
}

/**
 * @brief 生成关节命令字符串。
 * @param actions 动作向量（目标位置）。
 * @param zero_kp 是否将P增益置零。
 * @param zero_kd 是否将D增益置零。
 * @return std::string 序列化的关节命令字符串。
 */
std::string MotorManager::jointCommand(const Eigen::VectorXf &actions, bool zero_kp, bool zero_kd)
{
    publishTargetPos(actions, zero_kp, zero_kd);
#if defined(USE_TENSORRT)
    for (size_t i = 0; i < motors_in_id_.size(); ++i)
    {
        control_data.at(i) = motors_in_id_[i]->getMotorInfo();
        if (zero_kp)
        {
            control_data[i].kp = 0;
        }
        if (zero_kd)
        {
            control_data[i].kd = 0;
        }
    }
    auto serialized_data = MotorSerializer::serialize_array(control_data.data(), control_data.size());
    // RCLCPP_WARN(node_->get_logger(), "jointCommand.");
    return std::string(serialized_data.begin(), serialized_data.end());
#else
    // RCLCPP_WARN(node_->get_logger(), "A.");
    return std::string("A");
#endif
}

/**
 * @brief JointState回调函数实现，更新当前状态。
 * @param msg JointState消息指针。
 */
void MotorManager::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() != motors_.size() || msg->velocity.size() != motors_.size())
    {
        RCLCPP_WARN(node_->get_logger(), "JointState size mismatch.");
        return;
    }
    // 创建msg中name到索引的映射
    std::map<std::string, size_t> msg_name_to_index;
    for (size_t j = 0; j < msg->name.size(); ++j)
    {
        msg_name_to_index[msg->name[j]] = j;
    }

    size_t i = 0;
    for (const auto &name : joint_index_in_need)
    {
        auto it = msg_name_to_index.find(name);
        if (it != msg_name_to_index.end())
        {
            size_t j = it->second;
            current_pos_[i] = msg->position[j];
            current_vel_[i] = msg->velocity[j];
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Joint name '%s' not found in message.", name.c_str());
            // 可选：设置默认值，如 current_pos_[i] = 0.0; current_vel_[i] = 0.0;
        }
        ++i;
    }
    data_store_->UpdateJointPositions(current_pos_);
    data_store_->UpdateJointVelocities(current_vel_);
}

/**
 * @brief 计算源向量中每个元素在目标向量中的索引。
 * @param src 源向量（例如 motors_）。
 * @param target 目标向量（例如 motors_in_id_）。
 * @return  索引向量
 * @note std::vector<size_t> 未找到的位置为 std::numeric_limits<size_t>::max()。
 */
std::vector<size_t> MotorManager::findMutualIndices(const std::vector<std::shared_ptr<MotorBase>> &src,
                                                    const std::vector<std::shared_ptr<MotorBase>> &target) const
{
    std::vector<size_t> indices(src.size(), std::numeric_limits<size_t>::max());
    for (size_t i = 0; i < src.size(); ++i)
    {
        auto it = std::find(target.begin(), target.end(), src[i]);
        if (it != target.end())
        {
            indices[i] = static_cast<size_t>(std::distance(target.begin(), it));
        }
    }
    return indices;
}