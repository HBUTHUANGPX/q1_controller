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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "../include/rl_control.hpp"
class LowlevelManager : public rclcpp::Node
{
  public:
    /**
     * @brief 构造函数，加载配置并初始化组件。
     */
    LowlevelManager() : Node("low_level_manager_node"), time_step_(0.0)
    {
        printf("TestMotor:YAML \r\n");
        config_ = YAML::LoadFile("src/q1_controller/config/h1.yaml");
        dt_ = config_["dt"].as<float>();
        decimation_ = config_["decimation"].as<int>();
        num_actions_ = config_["num_actions"].as<int>();
        printf("TestMotor:YAML ok\r\n");
        scaled_action = Eigen::VectorXf::Zero(num_actions_);

        // 初始化推理
        motion_loader_ = std::make_shared<MotionLoader>(config_);
        printf("TestMotor:motion_loader_ ok\r\n");
        data_store_ = std::make_shared<DataStore>(config_, motion_loader_); // 初始化DataStore（假设joint_dim从config）
        printf("TestMotor:data_store_ ok\r\n");
        fsm_manager_ = std::make_shared<FSM_manager>(this, data_store_);
        printf("TestMotor:fsm_manager_ ok\r\n");
        motor_manager_ =
            std::make_shared<MotorManager>(this, config_, data_store_); // 初始化MotorManager（订阅/发布集成在内）
        // printf("TestMotor:motor_manager_ ok\r\n");

        rl_control_ =
            std::make_shared<rl_control>(this, config_, fsm_manager_, motion_loader_, data_store_, motor_manager_);
        RCLCPP_INFO(this->get_logger(), "TestMotor:motor_manager_ ok.");
        _init_deploy_mode();
        RCLCPP_INFO(this->get_logger(), "TestMotor:_init_deploy_mode ok.");
    };

    /**
     * @brief 测试更新函数。
     */
    void mujocoUpdate()
    {
        // if (static_cast<int>(rl_control_->get_time_step()) <= 1)
        // {
        //     rl_control_->setMujocoStateByDataset(static_cast<int>(time_step_));
        // }
        auto serialize_ = ECClientUpdate();
    }
    std::string ECClientUpdate()
    {
        // RCLCPP_INFO(this->get_logger(), "ECClientUpdate.");
        auto state = fsm_manager_->get_FSM_state();
        // RCLCPP_INFO(this->get_logger(), "get_FSM_state ok.");

        if (state == FSM_state::init_state)
        {
            /* code */
            rl_control_->reset_orientation_z_axis();
            return motor_manager_->jointCommand(scaled_action * 0,false,false);
        }
        else if (state == FSM_state::default_state)
        {
            rl_control_->set_time_step(1.f);
            scaled_action = rl_control_->inference();
            // std::cout<<rl_control_->get_time_step()<<"\n";
            // rl_control_->reset_orientation_z_axis();
        //     scaled_action << 
        //     0.1370222727457682, -0.0116672529114617, -0.1760668321089311, 0.2973577788381865,
        //     0.0170401624270848, -0.0211557047707694, -0.1605172051323785, -0.0258289204703437,
        //    -0.2191283717299953, 0.2765242836692117, -0.0620036125183105, -0.0110673648970468,
        //    -0.0157581942422049, 0.0027599581650325, 0.0920980521610805, -0.1061962468283517,
        //    -0.1719848224094936, 0.1178578019142151, -0.0346140116453171, 0.234131383895874,
        //    0.0496864591326032, -0.0266990440232413, 0.1476495061601911, -0.1784127507890974,
        //    -0.2115069150924683, -0.1131683230400085, 0.1862287759780884, -0.0471697747707367,
        //    -0.0851979851722717;
            // scaled_action[0] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[1] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[2] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[3] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[4] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[5] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );

            // scaled_action[6] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[7] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[8] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[9] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[10] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[11] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );

            // scaled_action[12] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );

            // scaled_action[13] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[14] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[15] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[16] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[17] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[18] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            // scaled_action[19] = 0.15*std::sin(rl_control_->get_time_step()/50.0 * M_PI );
            return motor_manager_->jointCommand(scaled_action,false,false);
        }
        else if (state == FSM_state::rl_run_state)
        {
            RCLCPP_INFO(this->get_logger(), "inference.");
            scaled_action = rl_control_->inference();
            return motor_manager_->jointCommand(scaled_action,false,false);
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "else.");
            return motor_manager_->jointCommand(scaled_action * 0,true,true);
        }
    }

    void _init_deploy_mode();

  private:
    YAML::Node config_;
    float time_step_;
    std::shared_ptr<DataStore> data_store_;
    std::shared_ptr<MotionLoader> motion_loader_;
    std::shared_ptr<MotorManager> motor_manager_; // 新增：Motor管理器
    std::shared_ptr<FSM_manager> fsm_manager_;    // 新增：Motor管理器

    std::shared_ptr<rl_control> rl_control_;

    int decimation_;
    float dt_;
    int num_actions_;
    Eigen::VectorXf scaled_action;

    rclcpp::TimerBase::SharedPtr timer_;
#if defined(USE_TENSORRT)
    std::unique_ptr<RTServer> robotcontrol_server;
    std::unique_ptr<RTClient> robotcontrol_client;
#endif
};
void LowlevelManager::_init_deploy_mode()
{
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
                                         std::bind(&LowlevelManager::mujocoUpdate, this));
    }
    else
    {
        throw std::runtime_error("MotorManager: 参数文件错误配置 deploy mode为 " + deploy_mode_ +
                                 ",请检查并核对.yaml文件中的 'deploy_mode' 字段");
    }
}
int main(int argc, char **argv)
{
    printf("main:main ok\r\n");
    rclcpp::init(argc, argv);
    printf("main:rclcpp::init ok\r\n");
    auto node = std::make_shared<LowlevelManager>();
    printf("main:std::make_shared<LowlevelManager> ok\r\n");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}