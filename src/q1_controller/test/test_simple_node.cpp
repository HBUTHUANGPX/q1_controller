#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher"), count_(0)
  {
    // 创建 publisher，话题名为 /chatter，队列深度 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

    // 每 1 秒（1000ms）触发一次回调
    timer_ = this->create_wall_timer(
        1000ms,
        std::bind(&SimplePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Simple publisher node has been started.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2 Humble! count: " + std::to_string(count_++);

    // 正确设置时间戳（自动兼容 use_sim_time）
    // 如果消息有 header，可使用 message.header.stamp = this->now();

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}