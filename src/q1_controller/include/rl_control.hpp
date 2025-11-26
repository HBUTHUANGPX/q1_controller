
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

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;              // Imu订阅器
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_zero_pub_;            // 重置零点发布器
    rclcpp::Publisher<q1_controller::msg::Dataset>::SharedPtr mocap_dataset_pub_; // 目标位置发布器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                  // TF缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                     // TF监听器

    std::shared_ptr<FSM_manager> fsm_manager_;                                    // 新增：FSM管理器
    std::shared_ptr<MotionLoader> motion_loader_;                                 // 运动数据加载器
    std::shared_ptr<DataStore> data_store_;
    std::shared_ptr<MotorManager> motor_manager_;                                 // Motor管理器

    std::shared_ptr<ObservationManager> manager_;                                 // 观测管理器
    std::shared_ptr<InferenceBase> inference_;                                    // 推理模块

    YAML::Node config_;                                                           // 配置节点

    int num_single_obs_;                                                          // 单帧观测维度
    int frame_stack_;                                                             // 帧堆叠数
    int num_actions_;                                                             // 动作维度
    int decimation_;                                                              // 决策间隔
    float dt_;                                                                    // 时间步长
    std::string policy_path_, mocap_path_;                                        // 模型路径，运动路径
    Eigen::MatrixXf observations_;                                                // 观测矩阵
    Eigen::VectorXf actions_;                                                     // 动作向量
    Eigen::VectorXf scaled_action;                                                // 缩放后的动作向量
    Eigen::Quaternionf reference_quat_;                                           // 参考四元数
    bool fresh_reference_quat_flag;                                               // 刷新参考四元数标志
    float time_step_;                                                             // 当前时间步

#if defined(USE_TENSORRT)
    std::unique_ptr<RTServer> robotcontrol_server;                                // RT服务器
    std::unique_ptr<RTClient> robotcontrol_client;                                // RT客户端
#endif
  public:
    /**
     * @brief 构造函数，初始化 rl_control 组件。
     * @param node ROS 节点指针。
     * @param config YAML 配置节点。
     * @param fsm_manager FSM 管理器共享指针。
     * @param motion_loader 运动数据加载器共享指针。
     * @param data_store 数据存储共享指针。
     * @param motor_manager Motor 管理器共享指针。
     */
    rl_control(rclcpp::Node *node, const YAML::Node &config, std::shared_ptr<FSM_manager> fsm_manager,
               std::shared_ptr<MotionLoader> motion_loader, std::shared_ptr<DataStore> data_store,
               std::shared_ptr<MotorManager> motor_manager);

    /**
     * @brief 析构函数，清理资源。
     */
    ~rl_control();

    /**
     * @brief 初始化部署模块
     */
    void _init_deploy_module();

    /**
     * @brief IMU消息回调函数
     * @param msg IMU消息指针
     */
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief 根据数据集设置MuJoCo状态
     * @param index 数据集索引
     */
    void setMujocoStateByDataset(size_t index);

    /**
     * @brief 更新观测数据
     * @param time_step 当前时间步
     */
    void UpdateObs(float time_step);

    /**
     * @brief 推理
     * @return 推理结果
     */
    Eigen::VectorXf inference();

    /**
     * @brief 设置当前时间步
     * @param time_step 当前时间步
     */
    void set_time_step(float time_step)
    {
        time_step_ = time_step;
    }

    /**
     * @brief 获取当前时间步
     * @return 当前时间步
     */
    float get_time_step()
    {
        return time_step_;
    }

    /**
     * @brief 重置参考四元数，确保Z轴对齐
     */
    void reset_orientation_z_axis()
    {
      fresh_reference_quat_flag = true;
    }
};
