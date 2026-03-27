#include <xsens_mvn_ros2/XSensClient.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xsens_mvn_ros2_msgs/msg/link_state_array.hpp>
#include <geometry_msgs/msg/point.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("xsens_client", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  // 参数初始化
  std::string model_name, reference_frame;
  int xsens_udp_port;
  node->get_parameter_or("model_name", model_name, std::string("skeleton"));
  node->get_parameter_or("reference_frame", reference_frame, std::string("world"));
  node->get_parameter_or("udp_port", xsens_udp_port, 8001);

  // ROS2发布者
  auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  auto link_state_publisher = node->create_publisher<xsens_mvn_ros2_msgs::msg::LinkStateArray>("link_states", 10);
  auto com_publisher = node->create_publisher<geometry_msgs::msg::Point>("com", 10);

  std::shared_ptr<XSensClient> xsens_client_ptr;
  try
  {
    xsens_client_ptr = std::make_shared<XSensClient>(xsens_udp_port);
  }
  catch (const std::exception& err)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), err.what());
    return -1;
  }

  if (!xsens_client_ptr->init())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "XSens client initialization failed.");
    return -1;
  }

  rclcpp::Rate loop_rate(240); // 匹配XSens MVN UDP软件的频率
  while (rclcpp::ok())
  {
    // 发布关节状态
    if (joint_state_publisher->get_subscription_count() > 0)
    {
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = node->now();
      auto joints = xsens_client_ptr->getHumanData()->getJoints();

      // 调试输出
      std::cout << "Publishing joint states, total joints: " << joints.size() << std::endl;
      int finger_joint_count = 0;
      for (auto joint_it = joints.begin(); joint_it != joints.end(); ++joint_it)
      {
        // 检查是否为手指关节
        if (joint_it->first.find("first") != std::string::npos ||
            joint_it->first.find("second") != std::string::npos ||
            joint_it->first.find("third") != std::string::npos ||
            joint_it->first.find("fourth") != std::string::npos ||
            joint_it->first.find("fifth") != std::string::npos)
        {
          finger_joint_count++;
          std::cout << "Finger joint: " << joint_it->first
                    << ", angles: [" << joint_it->second.state.angles[0] << ", "
                    << joint_it->second.state.angles[1] << ", "
                    << joint_it->second.state.angles[2] << "]" << std::endl;
        }
        joint_state_msg.name.push_back(model_name + "_" + joint_it->first + "_x");
        joint_state_msg.name.push_back(model_name + "_" + joint_it->first + "_y");
        joint_state_msg.name.push_back(model_name + "_" + joint_it->first + "_z");
        joint_state_msg.position.push_back(joint_it->second.state.angles[0] / 180 * 3.1415);
        joint_state_msg.position.push_back(joint_it->second.state.angles[1] / 180 * 3.1415);
        joint_state_msg.position.push_back(joint_it->second.state.angles[2] / 180 * 3.1415);
      }
      std::cout << "Found " << finger_joint_count << " finger joints" << std::endl;
      joint_state_publisher->publish(joint_state_msg);
    }

    // 发布链接TF和状态
    xsens_mvn_ros2_msgs::msg::LinkStateArray link_state_msg;
    auto links = xsens_client_ptr->getHumanData()->getLinks();
    for (auto link_it = links.begin(); link_it != links.end(); ++link_it)
    {
      // 发布链接TF
      if (!(link_it->second.state.orientation.x() == 0 && link_it->second.state.orientation.y() == 0 &&
            link_it->second.state.orientation.z() == 0 && link_it->second.state.orientation.w() == 0))
      {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = node->now();
        transform_stamped.header.frame_id = reference_frame;
        transform_stamped.child_frame_id = model_name + "_" + link_it->first;
        transform_stamped.transform.translation.x = link_it->second.state.position[0];
        transform_stamped.transform.translation.y = link_it->second.state.position[1];
        transform_stamped.transform.translation.z = link_it->second.state.position[2];
        transform_stamped.transform.rotation.x = link_it->second.state.orientation.x();
        transform_stamped.transform.rotation.y = link_it->second.state.orientation.y();
        transform_stamped.transform.rotation.z = link_it->second.state.orientation.z();
        transform_stamped.transform.rotation.w = link_it->second.state.orientation.w();
        tf_broadcaster->sendTransform(transform_stamped);
      }

      if (link_state_publisher->get_subscription_count() > 0)
      {
        // 发布链接状态
        xsens_mvn_ros2_msgs::msg::LinkState link_state;
        link_state.header.frame_id = link_it->first;
        link_state.header.stamp = node->now();
        link_state.pose.position = tf2::toMsg(link_it->second.state.position);
        link_state.pose.orientation = tf2::toMsg(link_it->second.state.orientation);
        Eigen::Matrix<double, 6, 1> link_twist;
        link_twist << link_it->second.state.velocity.linear, link_it->second.state.velocity.angular;
        link_state.twist = tf2::toMsg(link_twist);
        link_state.accel.linear.x = link_it->second.state.acceleration.linear[0];
        link_state.accel.linear.y = link_it->second.state.acceleration.linear[1];
        link_state.accel.linear.z = link_it->second.state.acceleration.linear[2];
        link_state.accel.angular.x = link_it->second.state.acceleration.angular[0];
        link_state.accel.angular.y = link_it->second.state.acceleration.angular[1];
        link_state.accel.angular.z = link_it->second.state.acceleration.angular[2];
        link_state_msg.states.push_back(link_state);
      }
    }

    if (link_state_publisher->get_subscription_count() > 0)
      link_state_publisher->publish(link_state_msg);

    if (com_publisher->get_subscription_count() > 0)
    {
      geometry_msgs::msg::Point com_msg;
      com_msg = tf2::toMsg(xsens_client_ptr->getHumanData()->getCOM());
      com_publisher->publish(com_msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}