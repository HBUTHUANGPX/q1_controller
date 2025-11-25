
#if defined(USE_OPENVINO)
#include "../include/q1_controller/openvino_inference.hpp"
#elif defined(USE_ONNX)
#include "../include/q1_controller/onnx_inference.hpp"
#elif defined(USE_TENSORRT)
#include "../include/q1_controller/tensorrt_inference.hpp"
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "q1_controller/msg/dataset.hpp" // 自定义消息
#include "../include/q1_controller/FSM_manager.hpp"
#include "../include/q1_controller/motion_loader.hpp"
#include "../include/q1_controller/data_store.hpp"
#include "../include/q1_controller/motor_manager.hpp" // 新增：Motor管理类
#include "../include/q1_controller/observation_manager.hpp"
#include "../include/q1_controller/inference_base.hpp"
#include "../include/q1_controller/mlp_network_io.hpp"

class rl_control
{
  private:
    rclcpp::Node::SharedPtr node_; // ROS节点指针

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // Imu订阅器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_zero_pub_;
    rclcpp::Publisher<q1_controller::msg::Dataset>::SharedPtr mocap_dataset_pub_; // 目标位置发布器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<FSM_manager> fsm_manager_; // 新增：Motor管理器
    std::shared_ptr<MotionLoader> motion_loader_;
    std::shared_ptr<DataStore> data_store_;
    std::shared_ptr<MotorManager> motor_manager_; // 新增：Motor管理器

    std::shared_ptr<ObservationManager> manager_;
    std::shared_ptr<InferenceBase> inference_;

    YAML::Node config_;

    int num_single_obs_;
    int frame_stack_;
    int num_actions_;
    int decimation_;
    float dt_;
    std::string policy_path_, mocap_path_;
    Eigen::MatrixXf observations_;
    Eigen::VectorXf actions_;
    Eigen::VectorXf scaled_action;
    Eigen::Quaternionf reference_quat_;
    bool fresh_reference_quat_flag;
    float time_step_;

#if defined(USE_TENSORRT)
    std::unique_ptr<RTServer> robotcontrol_server;
    std::unique_ptr<RTClient> robotcontrol_client;
#endif
  public:
    rl_control(rclcpp::Node *node, const YAML::Node &config, std::shared_ptr<FSM_manager> fsm_manager,
               std::shared_ptr<MotionLoader> motion_loader, std::shared_ptr<DataStore> data_store,
               std::shared_ptr<MotorManager> motor_manager);
    ~rl_control();
    void _init_deploy_module();
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void setMujocoStateByDataset(size_t index);
    void UpdateObs(float time_step);
    Eigen::VectorXf inference();
    void set_time_step(float time_step)
    {
        time_step_ = time_step;
    }
    float get_time_step()
    {
        return time_step_;
    };
    void reset_orientation_z_axis()
    {
      fresh_reference_quat_flag = true;
    }
};
