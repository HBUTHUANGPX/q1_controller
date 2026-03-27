#ifndef XSENS_MVN_ROS2_LINK_STATE_PROTO_SERIALIZER_H
#define XSENS_MVN_ROS2_LINK_STATE_PROTO_SERIALIZER_H

#include <string>

#include "xsens_mvn_ros2_msgs/msg/link_state_array.hpp"
#include "link_states.pb.h"

namespace xsens_mvn_ros2
{
constexpr unsigned int kLinkStatesSchemaVersion = 1U;

xsens::transport::LinkStateArray buildLinkStateArrayProto(
  const xsens_mvn_ros2_msgs::msg::LinkStateArray& ros_msg);

std::string serializeLinkStateArrayProto(
  const xsens_mvn_ros2_msgs::msg::LinkStateArray& ros_msg);
}  // namespace xsens_mvn_ros2

#endif
