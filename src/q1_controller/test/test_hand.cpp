#include <chrono>
#include <dexterous_hand/msg/hand_control.hpp>
#include <dexterous_hand/msg/hand_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TestHandNode : public rclcpp::Node
{
  public:
    TestHandNode() : Node("test_hand_node")
    {
        // 创建左手发布者和订阅者
        pub_left_ = this->create_publisher<dexterous_hand::msg::HandControl>("/left_hand_control", 10);
        sub_left_ = this->create_subscription<dexterous_hand::msg::HandFeedback>(
            "/left_hand_feedback", 10, std::bind(&TestHandNode::left_feedback_callback, this, std::placeholders::_1));

        // 创建右手发布者和订阅者
        pub_right_ = this->create_publisher<dexterous_hand::msg::HandControl>("/right_hand_control", 10);
        sub_right_ = this->create_subscription<dexterous_hand::msg::HandFeedback>(
            "/right_hand_feedback", 10, std::bind(&TestHandNode::right_feedback_callback, this, std::placeholders::_1));

        // 初始化关节名称
        joint_names_ = {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};

        // 初始化默认位置和力
        default_positions_ = std::vector<float>(6, 0.0f);
        default_forces_ = std::vector<float>(5, 0.0f); // 仅前5个关节有力控

        // 运行测试
        run_test();
    }

  private:
    rclcpp::Publisher<dexterous_hand::msg::HandControl>::SharedPtr pub_left_;
    rclcpp::Publisher<dexterous_hand::msg::HandControl>::SharedPtr pub_right_;
    rclcpp::Subscription<dexterous_hand::msg::HandFeedback>::SharedPtr sub_left_;
    rclcpp::Subscription<dexterous_hand::msg::HandFeedback>::SharedPtr sub_right_;

    dexterous_hand::msg::HandFeedback latest_left_feedback_;
    dexterous_hand::msg::HandFeedback latest_right_feedback_;

    std::vector<std::string> joint_names_;
    std::vector<float> default_positions_;
    std::vector<float> default_forces_;

    bool left_feedback_received_ = false;
    bool right_feedback_received_ = false;

    void left_feedback_callback(const dexterous_hand::msg::HandFeedback::SharedPtr msg)
    {
        latest_left_feedback_ = *msg;
        left_feedback_received_ = true;
    }

    void right_feedback_callback(const dexterous_hand::msg::HandFeedback::SharedPtr msg)
    {
        latest_right_feedback_ = *msg;
        right_feedback_received_ = true;
    }

    void run_test()
    {
        rclcpp::Rate rate(10); // 10 Hz 轮询率

        // 测试左手
        RCLCPP_INFO(this->get_logger(), "Testing left hand...");
        test_hand(pub_left_, latest_left_feedback_, left_feedback_received_, rate);

        // 测试右手
        RCLCPP_INFO(this->get_logger(), "Testing right hand...");
        test_hand(pub_right_, latest_right_feedback_, right_feedback_received_, rate);

        RCLCPP_INFO(this->get_logger(), "Test completed.");
    }

    void test_hand(const rclcpp::Publisher<dexterous_hand::msg::HandControl>::SharedPtr &pub,
                   dexterous_hand::msg::HandFeedback &feedback, bool &feedback_received, rclcpp::Rate &rate)
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            std::string joint = joint_names_[i];
            RCLCPP_INFO(this->get_logger(), "Testing joint: %s", joint.c_str());

            // 设置位置到100.0 (握紧)
            auto msg = create_control_msg(1, i, 100.0f); // mode=1, ANGLE_CONTROL
            pub->publish(msg);

            // 等待位置到位 (state[i] == 2) 或超时
            if (!wait_for_state(feedback, feedback_received, i, 2, rate))
            {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for joint %s to reach position 100.0", joint.c_str());
            }

            // 设置位置到0.0 (张开)
            msg = create_control_msg(1, i, 0.0f);
            pub->publish(msg);

            // 等待位置到位
            if (!wait_for_state(feedback, feedback_received, i, 2, rate))
            {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for joint %s to reach position 0.0", joint.c_str());
            }
        }
    }

    dexterous_hand::msg::HandControl create_control_msg(uint8_t mode, size_t joint_index, float target_pos)
    {
        dexterous_hand::msg::HandControl msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;
        msg.mode = mode;
        msg.position = default_positions_;
        msg.position[joint_index] = target_pos; // 只修改指定关节的位置
        msg.force = default_forces_;            // 力控全0
        return msg;
    }

    bool wait_for_state(dexterous_hand::msg::HandFeedback &feedback, bool &feedback_received, size_t joint_index,
                        uint8_t target_state, rclcpp::Rate &rate)
    {
        auto start_time = this->now();
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface()); // 处理回调
            if (feedback_received && feedback.state.size() > joint_index && feedback.state[joint_index] == target_state)
            {
                return true;
            }
            if ((this->now() - start_time).seconds() > 10.0)
            { // 超时10秒
                return false;
            }
            rate.sleep();
        }
        return false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestHandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}