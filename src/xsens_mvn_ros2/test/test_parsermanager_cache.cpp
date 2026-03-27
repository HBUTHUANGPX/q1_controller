#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "xsens_mvn_sdk/parsermanager.h"

namespace
{
void appendUint32(std::vector<char>& buffer, uint32_t value)
{
  buffer.push_back(static_cast<char>((value >> 24) & 0xFF));
  buffer.push_back(static_cast<char>((value >> 16) & 0xFF));
  buffer.push_back(static_cast<char>((value >> 8) & 0xFF));
  buffer.push_back(static_cast<char>(value & 0xFF));
}

void appendUint8(std::vector<char>& buffer, uint8_t value)
{
  buffer.push_back(static_cast<char>(value));
}

void appendFloat(std::vector<char>& buffer, float value)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "unexpected float size");
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  appendUint32(buffer, bits);
}

std::vector<char> makeHeader(const std::string& message_type, uint8_t data_count)
{
  std::vector<char> buffer;
  buffer.insert(buffer.end(), message_type.begin(), message_type.end());
  appendUint32(buffer, 1U);
  appendUint8(buffer, 0x80);
  appendUint8(buffer, data_count);
  appendUint32(buffer, 0U);
  appendUint8(buffer, 0U);
  buffer.insert(buffer.end(), 7, '\0');
  return buffer;
}

std::vector<char> makeQuaternionDatagram()
{
  auto buffer = makeHeader("MXTP02", 1);
  appendUint32(buffer, 1U);
  appendFloat(buffer, 1.0f);
  appendFloat(buffer, 2.0f);
  appendFloat(buffer, 3.0f);
  appendFloat(buffer, 0.0f);
  appendFloat(buffer, 0.0f);
  appendFloat(buffer, 0.0f);
  appendFloat(buffer, 1.0f);
  return buffer;
}

std::vector<char> makeJointAnglesDatagram()
{
  auto buffer = makeHeader("MXTP20", 1);
  appendUint32(buffer, 1U << 8);
  appendUint32(buffer, 2U << 8);
  appendFloat(buffer, 10.0f);
  appendFloat(buffer, 20.0f);
  appendFloat(buffer, 30.0f);
  return buffer;
}

std::vector<char> makeCenterOfMassDatagram()
{
  auto buffer = makeHeader("MXTP24", 1);
  appendFloat(buffer, 0.1f);
  appendFloat(buffer, 0.2f);
  appendFloat(buffer, 0.3f);
  return buffer;
}
}  // namespace

TEST(ParserManagerCacheTest, KeepsLatestQuaternionAcrossOtherDatagrams)
{
  ParserManager parser_manager;

  const auto quaternion_datagram = makeQuaternionDatagram();
  parser_manager.readDatagram(quaternion_datagram.data());

  ASSERT_NE(parser_manager.getQuaternionDatagram(), nullptr);
  EXPECT_EQ(parser_manager.getQuaternionDatagram()->getData().size(), 1U);

  const auto com_datagram = makeCenterOfMassDatagram();
  parser_manager.readDatagram(com_datagram.data());

  ASSERT_NE(parser_manager.getCenterOfMassDatagram(), nullptr);
  ASSERT_NE(parser_manager.getQuaternionDatagram(), nullptr);
  EXPECT_EQ(parser_manager.getQuaternionDatagram()->getData().size(), 1U);
  EXPECT_EQ(parser_manager.getQuaternionDatagram()->getItem(1).segmentId, 1);
}

TEST(ParserManagerCacheTest, KeepsLatestJointAnglesAcrossOtherDatagrams)
{
  ParserManager parser_manager;

  const auto joint_angles_datagram = makeJointAnglesDatagram();
  parser_manager.readDatagram(joint_angles_datagram.data());

  ASSERT_NE(parser_manager.getJointAnglesDatagram(), nullptr);
  EXPECT_EQ(parser_manager.getJointAnglesDatagram()->getData().size(), 1U);

  const auto quaternion_datagram = makeQuaternionDatagram();
  parser_manager.readDatagram(quaternion_datagram.data());

  ASSERT_NE(parser_manager.getQuaternionDatagram(), nullptr);
  ASSERT_NE(parser_manager.getJointAnglesDatagram(), nullptr);
  const auto joint = parser_manager.getJointAnglesDatagram()->getItem(1, 2);
  EXPECT_FLOAT_EQ(joint.rotation[0], 10.0f);
  EXPECT_FLOAT_EQ(joint.rotation[1], 20.0f);
  EXPECT_FLOAT_EQ(joint.rotation[2], 30.0f);
}
