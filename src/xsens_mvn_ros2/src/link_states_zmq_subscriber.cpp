#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

#include <google/protobuf/stubs/common.h>
#include <zmq.hpp>

#include "link_states.pb.h"

int main(int argc, char** argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  const std::string connect_address = argc > 1 ? argv[1] : "tcp://127.0.0.1:5555";
  const std::string topic = argc > 2 ? argv[2] : "xsens.link_states.v1";
  const int timeout_ms = argc > 3 ? std::atoi(argv[3]) : 5000;

  zmq::context_t context(1);
  zmq::socket_t subscriber(context, zmq::socket_type::sub);
  subscriber.set(zmq::sockopt::subscribe, topic);
  subscriber.set(zmq::sockopt::rcvtimeo, timeout_ms);
  subscriber.connect(connect_address);

  zmq::message_t topic_frame;
  zmq::message_t payload_frame;
  if (!subscriber.recv(topic_frame, zmq::recv_flags::none))
  {
    std::cerr << "Timed out waiting for ZMQ topic frame" << std::endl;
    return 1;
  }
  if (!subscriber.recv(payload_frame, zmq::recv_flags::none))
  {
    std::cerr << "Timed out waiting for ZMQ payload frame" << std::endl;
    return 1;
  }

  xsens::transport::LinkStateArray proto_msg;
  if (!proto_msg.ParseFromArray(payload_frame.data(), static_cast<int>(payload_frame.size())))
  {
    std::cerr << "Failed to parse protobuf payload" << std::endl;
    return 1;
  }

  std::cout << "topic=" << topic_frame.to_string() << std::endl;
  std::cout << "schema_version=" << proto_msg.header().schema_version() << std::endl;
  std::cout << "frame_id=" << proto_msg.header().frame_id() << std::endl;
  std::cout << "states_size=" << proto_msg.states_size() << std::endl;

  for (int i = 0; i < proto_msg.states_size(); ++i)
  {
    const auto& state = proto_msg.states(i);
    std::cout << "state[" << i << "].name=" << state.name() << std::endl;
    std::cout << "state[" << i << "].position="
              << state.pose().position().x() << ","
              << state.pose().position().y() << ","
              << state.pose().position().z() << std::endl;
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
