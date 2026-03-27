#include <gtest/gtest.h>

#include "xsens_mvn_ros2/LinkStateProtoSerializer.h"

TEST(LinkStateProtoSerializerTest, ConvertsRosMessageToProtobuf)
{
  xsens_mvn_ros2_msgs::msg::LinkStateArray ros_msg;
  ros_msg.header.frame_id = "world";
  ros_msg.header.stamp.sec = 123;
  ros_msg.header.stamp.nanosec = 456;

  xsens_mvn_ros2_msgs::msg::LinkState state;
  state.header.frame_id = "pelvis";
  state.pose.position.x = 1.0;
  state.pose.position.y = 2.0;
  state.pose.position.z = 3.0;
  state.pose.orientation.x = 0.1;
  state.pose.orientation.y = 0.2;
  state.pose.orientation.z = 0.3;
  state.pose.orientation.w = 0.4;
  state.twist.linear.x = 4.0;
  state.twist.linear.y = 5.0;
  state.twist.linear.z = 6.0;
  state.twist.angular.x = 7.0;
  state.twist.angular.y = 8.0;
  state.twist.angular.z = 9.0;
  state.accel.linear.x = 10.0;
  state.accel.linear.y = 11.0;
  state.accel.linear.z = 12.0;
  state.accel.angular.x = 13.0;
  state.accel.angular.y = 14.0;
  state.accel.angular.z = 15.0;
  ros_msg.states.push_back(state);

  const auto proto_msg = xsens_mvn_ros2::buildLinkStateArrayProto(ros_msg);

  EXPECT_EQ(proto_msg.header().schema_version(), xsens_mvn_ros2::kLinkStatesSchemaVersion);
  EXPECT_EQ(proto_msg.header().frame_id(), "world");
  EXPECT_EQ(proto_msg.header().stamp_sec(), 123);
  EXPECT_EQ(proto_msg.header().stamp_nanosec(), 456U);
  ASSERT_EQ(proto_msg.states_size(), 1);

  const auto& proto_state = proto_msg.states(0);
  EXPECT_EQ(proto_state.name(), "pelvis");
  EXPECT_DOUBLE_EQ(proto_state.pose().position().x(), 1.0);
  EXPECT_DOUBLE_EQ(proto_state.pose().position().y(), 2.0);
  EXPECT_DOUBLE_EQ(proto_state.pose().position().z(), 3.0);
  EXPECT_DOUBLE_EQ(proto_state.pose().orientation().x(), 0.1);
  EXPECT_DOUBLE_EQ(proto_state.pose().orientation().y(), 0.2);
  EXPECT_DOUBLE_EQ(proto_state.pose().orientation().z(), 0.3);
  EXPECT_DOUBLE_EQ(proto_state.pose().orientation().w(), 0.4);
  EXPECT_DOUBLE_EQ(proto_state.twist().linear().x(), 4.0);
  EXPECT_DOUBLE_EQ(proto_state.twist().angular().z(), 9.0);
  EXPECT_DOUBLE_EQ(proto_state.accel().linear().z(), 12.0);
  EXPECT_DOUBLE_EQ(proto_state.accel().angular().z(), 15.0);
}
