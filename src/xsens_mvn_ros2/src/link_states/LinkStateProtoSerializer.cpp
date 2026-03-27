#include "xsens_mvn_ros2/LinkStateProtoSerializer.h"

#include <stdexcept>

namespace xsens_mvn_ros2
{
namespace
{
void fillVector3(
  xsens::transport::Vector3* proto_vector,
  double x,
  double y,
  double z)
{
  proto_vector->set_x(x);
  proto_vector->set_y(y);
  proto_vector->set_z(z);
}
}  // namespace

xsens::transport::LinkStateArray buildLinkStateArrayProto(
  const xsens_mvn_ros2_msgs::msg::LinkStateArray& ros_msg)
{
  xsens::transport::LinkStateArray proto_msg;

  auto* header = proto_msg.mutable_header();
  header->set_schema_version(kLinkStatesSchemaVersion);
  header->set_stamp_sec(ros_msg.header.stamp.sec);
  header->set_stamp_nanosec(ros_msg.header.stamp.nanosec);
  header->set_frame_id(ros_msg.header.frame_id);

  for (const auto& ros_state : ros_msg.states)
  {
    auto* proto_state = proto_msg.add_states();
    proto_state->set_name(ros_state.header.frame_id);

    auto* pose = proto_state->mutable_pose();
    fillVector3(
      pose->mutable_position(),
      ros_state.pose.position.x,
      ros_state.pose.position.y,
      ros_state.pose.position.z);
    pose->mutable_orientation()->set_x(ros_state.pose.orientation.x);
    pose->mutable_orientation()->set_y(ros_state.pose.orientation.y);
    pose->mutable_orientation()->set_z(ros_state.pose.orientation.z);
    pose->mutable_orientation()->set_w(ros_state.pose.orientation.w);

    auto* twist = proto_state->mutable_twist();
    fillVector3(
      twist->mutable_linear(),
      ros_state.twist.linear.x,
      ros_state.twist.linear.y,
      ros_state.twist.linear.z);
    fillVector3(
      twist->mutable_angular(),
      ros_state.twist.angular.x,
      ros_state.twist.angular.y,
      ros_state.twist.angular.z);

    auto* accel = proto_state->mutable_accel();
    fillVector3(
      accel->mutable_linear(),
      ros_state.accel.linear.x,
      ros_state.accel.linear.y,
      ros_state.accel.linear.z);
    fillVector3(
      accel->mutable_angular(),
      ros_state.accel.angular.x,
      ros_state.accel.angular.y,
      ros_state.accel.angular.z);
  }

  return proto_msg;
}

std::string serializeLinkStateArrayProto(
  const xsens_mvn_ros2_msgs::msg::LinkStateArray& ros_msg)
{
  const auto proto_msg = buildLinkStateArrayProto(ros_msg);
  std::string payload;
  if (!proto_msg.SerializeToString(&payload))
  {
    throw std::runtime_error("Failed to serialize LinkStateArray protobuf payload");
  }
  return payload;
}
}  // namespace xsens_mvn_ros2
