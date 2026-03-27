# XSens Link States ZMQ LAN Test Guide

## Scope

This guide covers:

- Python subscriber setup on a receiver machine
- Cross-machine LAN validation for the `xsens.link_states.v1` ZMQ stream
- Acceptance criteria for protobuf payload correctness

## Transport Summary

- ROS2 topic: `/link_states`
- ZMQ pattern: `PUB/SUB`
- ZMQ topic frame: `xsens.link_states.v1`
- Payload frame: protobuf `xsens.transport.LinkStateArray`
- Recommended bridge setting: `conflate:=false`

## Sender Machine Setup

Build the package:

```bash
source /opt/ros/humble/setup.bash
cd /home/hpx/HPX_Loco_3/pack/Q1_control_0203_1944
colcon build --packages-select xsens_mvn_ros2
source install/setup.bash
```

Run the bridge:

```bash
ros2 run xsens_mvn_ros2 link_states_zmq_bridge --ros-args \
  -p zmq_bind_address:=tcp://*:5555 \
  -p zmq_topic:=xsens.link_states.v1 \
  -p conflate:=false
```

If you want to launch it together with the XSens node:

```bash
ros2 launch xsens_mvn_ros2 xsens_client.launch.py \
  launch_zmq_bridge:=true \
  zmq_bind_address:=tcp://*:5555 \
  zmq_conflate:=false
```

## Receiver Machine Setup

Install Python runtime dependencies:

```bash
python3 -m pip install --user pyzmq protobuf
```

Generate the Python protobuf module from the sender repository copy or a copied `link_states.proto`:

```bash
mkdir -p /tmp/xsens_proto
protoc \
  --proto_path /path/to/Q1_control_0203_1944/src/xsens_mvn_ros2/proto \
  --python_out /tmp/xsens_proto \
  /path/to/Q1_control_0203_1944/src/xsens_mvn_ros2/proto/link_states.proto
```

Run the Python subscriber:

```bash
python3 /path/to/Q1_control_0203_1944/src/xsens_mvn_ros2/examples/recv_link_states.py \
  --connect tcp://192.168.1.10:5555 \
  --topic xsens.link_states.v1 \
  --proto-python-dir /tmp/xsens_proto \
  --count 3
```

Replace `192.168.1.10` with the sender machine IP.

## Local Loopback Result

Validated on the sender machine with:

```bash
ros2 run xsens_mvn_ros2 link_states_zmq_bridge --ros-args \
  -p zmq_bind_address:=tcp://127.0.0.1:5555 \
  -p zmq_topic:=xsens.link_states.v1 \
  -p conflate:=false

ros2 run xsens_mvn_ros2 link_states_zmq_subscriber \
  tcp://127.0.0.1:5555 xsens.link_states.v1 15000

ros2 run xsens_mvn_ros2 link_states_test_publisher --ros-args -p publish_count:=10
```

Observed subscriber output:

```text
topic=xsens.link_states.v1
schema_version=1
frame_id=world
states_size=2
state[0].name=pelvis
state[0].position=1.1,2.2,3.3
state[1].name=left_hand
state[1].position=-1,-2,-3
```

## LAN Test Procedure

1. Confirm the sender can bind to the chosen interface and port.
2. Confirm the receiver can ping the sender IP.
3. Start the bridge on the sender with `conflate:=false`.
4. Start the Python subscriber on the receiver and wait until it blocks on receive.
5. Start the XSens stream or the test publisher on the sender.
6. Check that the receiver prints:
   - `topic=xsens.link_states.v1`
   - `schema_version=1`
   - a non-zero `states_size`
7. Verify at least one known link name and position triplet.
8. Stop and restart the receiver subscriber and confirm it can reconnect and receive again.
9. Start a second subscriber and confirm both receive frames independently.

## Acceptance Checklist

- [ ] Receiver machine can connect to `tcp://<sender-ip>:5555`
- [ ] Python subscriber imports `link_states_pb2.py` successfully
- [ ] Received topic equals `xsens.link_states.v1`
- [ ] Received `schema_version` equals `1`
- [ ] `frame_id` matches sender data
- [ ] `states_size` is greater than `0`
- [ ] At least one expected link name is correct
- [ ] At least one expected position triplet is correct
- [ ] Two subscribers can receive simultaneously
- [ ] Restarting one subscriber does not interrupt the other

## Known Constraint

Do not enable `ZMQ_CONFLATE` with the current multipart format (`topic frame + protobuf frame`). In local testing this caused subscribers to miss all data. Keep `conflate:=false` unless you redesign the wire format to a single frame.
