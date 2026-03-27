#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <xsens_mvn_ros2_msgs/msg/link_state_array.hpp>
#include <zmq.hpp>

#include "xsens_mvn_ros2/LinkStateProtoSerializer.h"

class LinkStatesZmqBridge : public rclcpp::Node
{
public:
  LinkStatesZmqBridge()
  : Node("link_states_zmq_bridge"),
    zmq_context_(1),
    pub_socket_(zmq_context_, zmq::socket_type::pub),
    sent_messages_(0)
  {
    const auto ros_topic = declare_parameter<std::string>("ros_topic", "/link_states");
    const auto zmq_bind_address =
      declare_parameter<std::string>("zmq_bind_address", "tcp://*:5555");
    zmq_topic_ = declare_parameter<std::string>("zmq_topic", "xsens.link_states.v1");
    const auto sndhwm = declare_parameter<int>("sndhwm", 5);
    const auto conflate = declare_parameter<bool>("conflate", false);

    pub_socket_.set(zmq::sockopt::sndhwm, static_cast<int>(sndhwm));
    pub_socket_.set(zmq::sockopt::linger, 0);
    if (conflate)
    {
      RCLCPP_WARN(
        get_logger(),
        "ZMQ conflate is enabled. This is not recommended with multipart PUB/SUB messages.");
      pub_socket_.set(zmq::sockopt::conflate, true);
    }
    pub_socket_.bind(zmq_bind_address);

    subscription_ = create_subscription<xsens_mvn_ros2_msgs::msg::LinkStateArray>(
      ros_topic,
      rclcpp::QoS(10),
      std::bind(&LinkStatesZmqBridge::handleLinkStates, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %s to %s on topic %s",
      ros_topic.c_str(),
      zmq_bind_address.c_str(),
      zmq_topic_.c_str());
  }

private:
  void handleLinkStates(const xsens_mvn_ros2_msgs::msg::LinkStateArray::SharedPtr msg)
  {
    try
    {
      const std::string payload = xsens_mvn_ros2::serializeLinkStateArrayProto(*msg);
      pub_socket_.send(zmq::buffer(zmq_topic_), zmq::send_flags::sndmore);
      pub_socket_.send(zmq::buffer(payload), zmq::send_flags::none);
      sent_messages_++;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Sent %zu ZMQ link_states messages, last payload size=%zu bytes",
        sent_messages_,
        payload.size());
    }
    catch (const std::exception& err)
    {
      RCLCPP_ERROR(get_logger(), "Failed to publish link states over ZMQ: %s", err.what());
    }
  }

  zmq::context_t zmq_context_;
  zmq::socket_t pub_socket_;
  std::string zmq_topic_;
  std::size_t sent_messages_;
  rclcpp::Subscription<xsens_mvn_ros2_msgs::msg::LinkStateArray>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkStatesZmqBridge>());
  rclcpp::shutdown();
  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
