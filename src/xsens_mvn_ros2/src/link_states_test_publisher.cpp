#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <xsens_mvn_ros2_msgs/msg/link_state_array.hpp>

class LinkStatesTestPublisher : public rclcpp::Node
{
public:
  LinkStatesTestPublisher()
  : Node("link_states_test_publisher"),
    publish_count_(0),
    target_publish_count_(declare_parameter<int>("publish_count", 30))
  {
    publisher_ = create_publisher<xsens_mvn_ros2_msgs::msg::LinkStateArray>("/link_states", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LinkStatesTestPublisher::publishMessage, this));
  }

private:
  void publishMessage()
  {
    if (publisher_->get_subscription_count() == 0)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for /link_states subscribers before publishing test data");
      return;
    }

    xsens_mvn_ros2_msgs::msg::LinkStateArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";

    xsens_mvn_ros2_msgs::msg::LinkState pelvis;
    pelvis.header.stamp = msg.header.stamp;
    pelvis.header.frame_id = "pelvis";
    pelvis.pose.position.x = 1.1;
    pelvis.pose.position.y = 2.2;
    pelvis.pose.position.z = 3.3;
    pelvis.pose.orientation.x = 0.1;
    pelvis.pose.orientation.y = 0.2;
    pelvis.pose.orientation.z = 0.3;
    pelvis.pose.orientation.w = 0.4;
    pelvis.twist.linear.x = 4.4;
    pelvis.twist.linear.y = 5.5;
    pelvis.twist.linear.z = 6.6;
    pelvis.twist.angular.x = 7.7;
    pelvis.twist.angular.y = 8.8;
    pelvis.twist.angular.z = 9.9;
    pelvis.accel.linear.x = 10.1;
    pelvis.accel.linear.y = 11.1;
    pelvis.accel.linear.z = 12.1;
    pelvis.accel.angular.x = 13.1;
    pelvis.accel.angular.y = 14.1;
    pelvis.accel.angular.z = 15.1;
    msg.states.push_back(pelvis);

    xsens_mvn_ros2_msgs::msg::LinkState hand;
    hand.header.stamp = msg.header.stamp;
    hand.header.frame_id = "left_hand";
    hand.pose.position.x = -1.0;
    hand.pose.position.y = -2.0;
    hand.pose.position.z = -3.0;
    hand.pose.orientation.w = 1.0;
    msg.states.push_back(hand);

    publisher_->publish(msg);
    publish_count_++;
    RCLCPP_INFO(get_logger(), "Published test /link_states message #%d", publish_count_);

    if (publish_count_ >= target_publish_count_)
    {
      rclcpp::shutdown();
    }
  }

  int publish_count_;
  int target_publish_count_;
  rclcpp::Publisher<xsens_mvn_ros2_msgs::msg::LinkStateArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkStatesTestPublisher>());
  return 0;
}
