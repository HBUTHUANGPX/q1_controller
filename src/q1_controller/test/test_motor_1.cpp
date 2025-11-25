// 文件: test_motor_manager.cpp
#include "../include/q1_controller/FSM_manager.hpp"
#include "../include/q1_controller/data_store.hpp"
#include "../include/q1_controller/inference_base.hpp"
#include "../include/q1_controller/mlp_network_io.hpp"
#include "../include/q1_controller/motion_loader.hpp"
#include "../include/q1_controller/motor_manager.hpp" // 新增：Motor管理类
#include "../include/q1_controller/observation_manager.hpp"
#if defined(USE_OPENVINO)
#include "../include/q1_controller/openvino_inference.hpp"
#elif defined(USE_ONNX)
#include "../include/q1_controller/onnx_inference.hpp"
#elif defined(USE_TENSORRT)
#include "../include/q1_controller/tensorrt_inference.hpp"
#endif
#include "q1_controller/msg/dataset.hpp" // 自定义消息
#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
/**
 * @brief 测试MotorManager的ROS节点类。
 *
 * 此类集成MotorManager，处理观测、推理和目标位置发布。
 */

class TestMotorManager : public rclcpp::Node
{
  public:
    /**
     * @brief 构造函数，加载配置并初始化组件。
     */
    TestMotorManager() : Node("test_motor_manager_node"), time_step_(0.0)
    {
        printf("TestMotor:YAML \r\n");
        config_ = YAML::LoadFile("src/q1_controller/config/h1.yaml");

        policy_path_ = config_["policy_path"].as<std::string>();
        num_single_obs_ = config_["num_single_obs"].as<int>();
        frame_stack_ = config_["frame_stack"].as<int>();
        num_actions_ = config_["num_actions"].as<int>();
        dt_ = config_["dt"].as<float>();
        decimation_ = config_["decimation"].as<int>();
        printf("TestMotor:YAML ok\r\n");

        // 初始化推理
        motion_loader_ = std::make_shared<MotionLoader>(config_);
        printf("TestMotor:motion_loader_ ok\r\n");
        data_store_ = std::make_shared<DataStore>(config_, motion_loader_); // 初始化DataStore（假设joint_dim从config）
        printf("TestMotor:data_store_ ok\r\n");
        fsm_manager_ = std::make_shared<FSM_manager>(this, data_store_);
        printf("TestMotor:fsm_manager_ ok\r\n");
        motor_manager_ =
            std::make_shared<MotorManager>(this, config_, data_store_); // 初始化MotorManager（订阅/发布集成在内）
        printf("TestMotor:motor_manager_ ok\r\n");
        auto network_io = std::make_shared<MLPNetworkIO>(config_);
        printf("TestMotor:network_io ok\r\n");
#if defined(USE_OPENVINO)
        inference_ = std::make_shared<OpenVINOInference>(config_, network_io, motor_manager_);
        printf("TestMotor:inference_ OpenVINOInference ok\r\n");
#elif defined(USE_ONNX)
        inference_ = std::make_shared<OnnxInference>(config_, network_io, motor_manager_);
        printf("TestMotor:inference_ OnnxInference ok\r\n");
#elif defined(USE_TENSORRT)
        inference_ = std::make_shared<TensorRTInference>(config_, network_io, motor_manager_);
        printf("TestMotor:inference_  TensorRTInference ok\r\n");
#endif

        manager_ = std::make_shared<ObservationManager>(config_, data_store_); // 初始化ObservationManager
        printf("TestMotor:manager_ ok\r\n");

        observations_ = Eigen::MatrixXf::Zero(1, num_single_obs_ * frame_stack_); // 初始化观测和动作
        actions_ = Eigen::VectorXf::Zero(num_actions_);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&TestMotorManager::ImuCallback, this, std::placeholders::_1));

        reset_zero_pub_ = this->create_publisher<std_msgs::msg::Bool>("/reset_zero", 10);

        mocap_dataset_pub_ = this->create_publisher<q1_controller::msg::Dataset>("/Dataset", 10);
        RCLCPP_INFO(this->get_logger(), "TestMotorManager node initialized.");
        std_msgs::msg::Bool flag;
        flag.data = true;
        reset_zero_pub_->publish(flag);
        RCLCPP_INFO(this->get_logger(), "TestMotorManager reset zero.");

        std::string deploy_mode_ = config_["deploy_mode"].as<std::string>();
        std::string sim2sim = "sim2sim";
        std::string sim2real = "sim2real";
        if (deploy_mode_ == sim2real)
        {
#if defined(USE_TENSORRT)
            robotcontrol_server = std::make_unique<RTServer>(ServerPorts::RobotControl_Server);
            auto server_callback = [this_manager = motor_manager_.get()](const std::string &message, int from_port) {
                this_manager->jointStateUpdate(message, from_port);
            };
            if (!robotcontrol_server->start(server_callback))
            {
                std::cerr << "Failed to start robotcontrol server" << std::endl;
            }

            robotcontrol_client =
                std::make_unique<RTClient>(ServerPorts::ECMaster1_Server, ClientPorts::RobotControl_Client + 1, 20000);
            auto client_callback = [this]() { return this->ECClientUpdate(); };
            if (!robotcontrol_client->start(nullptr, client_callback))
            {
                std::cerr << "Failed to start robotcontrol client1" << std::endl;
            }
#else
            std::cerr << "非TensorRT编译不可使用 'sim2real' 模式" << std::endl;
#endif
        }
        else if (deploy_mode_ == sim2sim)
        {
            // 创建定时器
            timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * decimation_ * 1000.0)),
                                             std::bind(&TestMotorManager::testUpdate, this));
        }
        else
        {
            throw std::runtime_error("MotorManager: 参数文件错误配置 deploy mode为 " + deploy_mode_ +
                                     ",请检查并核对.yaml文件中的 'deploy_mode' 字段");
        }
    };

    /**
     * @brief 测试更新函数。
     */
    void testUpdate()
    {
        if (static_cast<int>(time_step_) <= 1)
        {
            setMujocoStateByDataset(static_cast<int>(time_step_));
        }
        auto serialize_ = ECClientUpdate();
    }

    Eigen::VectorXf inference()
    {
        UpdateObs(time_step_);
        // RCLCPP_INFO(this->get_logger(), "UpdateObs");
        actions_ = inference_->UpdateAction(observations_, time_step_);
        // RCLCPP_INFO(this->get_logger(), "actions_");
        data_store_->UpdateLastActions(actions_);
        // RCLCPP_INFO(this->get_logger(), "data_store_");
        Eigen::VectorXf scaled_action = inference_->Scale_and_clamp_Action(actions_);
        // std::string serialize_ = motor_manager_->jointCommand(scaled_action);
        // RCLCPP_INFO(this->get_logger(), "motor_manager_");
        time_step_ += 1.0;
        if (time_step_ >= motion_loader_->getTimeStepTotal())
        {
            time_step_ *= 0.0;
        }
        return scaled_action;
    }
    
    std::string ECClientUpdate()
    {
        RCLCPP_INFO(this->get_logger(), "Starting test update,time_step_ is :%d", static_cast<int>(time_step_));
        
        // RCLCPP_INFO(this->get_logger(), "ECClientUpdate.");
        auto state = fsm_manager_->get_FSM_state();
        // RCLCPP_INFO(this->get_logger(), "get_FSM_state ok.");
        Eigen::VectorXf scaled_action;
        scaled_action = Eigen::VectorXf::Zero(29);
        if (state == FSM_state::init_state)
        {
            scaled_action = inference();
            /* code */
        }
        else if (state == FSM_state::default_state)
        {
            /* code */
        }
        else if (state == FSM_state::rl_run_state)
        {
            RCLCPP_INFO(this->get_logger(), "inference.");
            scaled_action = inference();
            RCLCPP_INFO(this->get_logger(), "inference ok.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "else.");
        }
        return motor_manager_->jointCommand(scaled_action);
    }

  private:
    void UpdateObs(float time_step)
    {
        // RCLCPP_INFO(this->get_logger(), "UpdateObs.");
        try
        {
            // RCLCPP_INFO(this->get_logger(), "try UpdateObservations");
            observations_ = manager_->UpdateObservations(time_step);
            // std::cout << "行数: " << observations_.rows() << ", 列数: " << observations_.cols() << std::endl;
            // std::cout << observations_ << std::endl;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to update observations: %s", e.what());
        }
    }

    void setMujocoStateByDataset(size_t index)
    {
        q1_controller::msg::Dataset msg;
        auto body_pos = motion_loader_->getBodyPosW(index);
        auto body_quat = motion_loader_->getBodyQuatW(index);
        auto body_lin_vel = motion_loader_->getBodyLinVelW(index);
        auto body_ang_vel = motion_loader_->getBodyAngVelW(index);
        auto joint_pos = motion_loader_->getJointPos(index);
        auto joint_vel = motion_loader_->getJointVel(index);
        auto ref_root_pos = body_pos.row(0);
        auto ref_root_quat = body_quat.row(0);
        auto ref_root_lin_vel = body_lin_vel.row(0);
        auto ref_root_ang_vel = body_ang_vel.row(0);

        msg.root_position.x = ref_root_pos(0);
        msg.root_position.y = ref_root_pos(1);
        msg.root_position.z = ref_root_pos(2);
        // std::cout << "ref_root_pos: " << ref_root_pos << std::endl;
        msg.root_quat.w = ref_root_quat(3) * 0 + 1;
        msg.root_quat.x = ref_root_quat(0) * 0;
        msg.root_quat.y = ref_root_quat(1) * 0;
        msg.root_quat.z = ref_root_quat(2) * 0;

        msg.root_velocity.x = ref_root_lin_vel(0) * 0;
        msg.root_velocity.y = ref_root_lin_vel(1) * 0;
        msg.root_velocity.z = ref_root_lin_vel(2) * 0;

        msg.root_angular_velocity.x = ref_root_ang_vel(0) * 0;
        msg.root_angular_velocity.y = ref_root_ang_vel(1) * 0;
        msg.root_angular_velocity.z = ref_root_ang_vel(2) * 0;

        msg.motor_state.position.reserve(joint_pos.size());
        for (Eigen::Index i = 0; i < joint_pos.size(); ++i)
        {
            msg.motor_state.position.push_back(static_cast<double>(joint_pos(i))); // 显式转换以确保清晰
        }

        msg.motor_state.velocity.reserve(joint_vel.size());
        for (Eigen::Index i = 0; i < joint_vel.size(); ++i)
        {
            msg.motor_state.velocity.push_back(static_cast<double>(joint_vel(i))); // 显式转换以确保清晰
        }

        mocap_dataset_pub_->publish(msg);
    }

    /**
     * @brief Imu话题回调函数，更新DataStore中的基座角速度和参考四元数。
     * @param msg Imu消息指针。
     */
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ImuCallback.");
        Eigen::VectorXf ang_vel(3);
        ang_vel(0) = msg->angular_velocity.x;
        ang_vel(1) = msg->angular_velocity.y;
        ang_vel(2) = msg->angular_velocity.z;
        data_store_->UpdateBaseAngularVelocities(ang_vel);

        // 更新机器人四元数
        Eigen::Quaternionf robot_quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        data_store_->UpdateRobotQuat(robot_quat);

        // RCLCPP_INFO(this->get_logger(), "Updated base angular velocities and quaternions from /mujoco_imu.");
    }

    YAML::Node config_;
    float time_step_;
    std::shared_ptr<ObservationManager> manager_;
    std::shared_ptr<InferenceBase> inference_;
    std::shared_ptr<DataStore> data_store_;
    std::shared_ptr<MotionLoader> motion_loader_;
    std::shared_ptr<MotorManager> motor_manager_; // 新增：Motor管理器
    std::shared_ptr<FSM_manager> fsm_manager_;    // 新增：Motor管理器

    int num_single_obs_;
    int frame_stack_;
    int num_actions_;
    int decimation_;
    float dt_;
    std::string policy_path_, mocap_path_;
    Eigen::MatrixXf observations_;
    Eigen::VectorXf actions_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // Imu订阅器
    rclcpp::CallbackGroup::SharedPtr policy_group_;
    rclcpp::CallbackGroup::SharedPtr imu_group_; // IMU回调组
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_zero_pub_;
    rclcpp::Publisher<q1_controller::msg::Dataset>::SharedPtr mocap_dataset_pub_; // 目标位置发布器
#if defined(USE_TENSORRT)
    std::unique_ptr<RTServer> robotcontrol_server;
    std::unique_ptr<RTClient> robotcontrol_client;
#endif
};

int main(int argc, char **argv)
{
    printf("main:main ok\r\n");
    rclcpp::init(argc, argv);
    printf("main:rclcpp::init ok\r\n");
    auto node = std::make_shared<TestMotorManager>();
    printf("main:std::make_shared<TestMotorManager> ok\r\n");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}