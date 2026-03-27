# Link States ZMQ Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Broadcast `/link_states` over ZMQ PUB/SUB using a Protobuf payload that Python and C++ subscribers can decode across a LAN.

**Architecture:** Add a dedicated ROS2 bridge node in `xsens_mvn_ros2` that subscribes to `xsens_mvn_ros2_msgs/LinkStateArray`, converts the message into a versioned Protobuf schema, and publishes it through a ZMQ `PUB` socket as a two-frame message: topic + payload. Keep the conversion logic in a small helper unit so it can be tested without spinning ROS or ZMQ.

**Tech Stack:** ROS2 Humble, Protobuf, ZeroMQ, CMake, gtest

---

### Task 1: Define the transport schema and code generation

**Files:**
- Create: `src/xsens_mvn_ros2/proto/link_states.proto`
- Modify: `src/xsens_mvn_ros2/CMakeLists.txt`

- [ ] **Step 1: Add the Protobuf schema**
- [ ] **Step 2: Generate C++ sources from the schema in CMake**
- [ ] **Step 3: Link generated sources into reusable bridge code**

### Task 2: Add a testable ROS-to-Protobuf conversion layer

**Files:**
- Create: `src/xsens_mvn_ros2/include/xsens_mvn_ros2/LinkStateProtoSerializer.h`
- Create: `src/xsens_mvn_ros2/src/link_states/LinkStateProtoSerializer.cpp`
- Test: `src/xsens_mvn_ros2/test/test_link_state_proto_serializer.cpp`

- [ ] **Step 1: Write a failing serializer unit test**
- [ ] **Step 2: Run the focused test and confirm failure**
- [ ] **Step 3: Implement minimal conversion helpers**
- [ ] **Step 4: Re-run the focused test and confirm pass**

### Task 3: Implement the ZMQ bridge node

**Files:**
- Create: `src/xsens_mvn_ros2/src/link_states_zmq_bridge.cpp`
- Modify: `src/xsens_mvn_ros2/CMakeLists.txt`

- [ ] **Step 1: Add the bridge executable and dependencies**
- [ ] **Step 2: Subscribe to `/link_states` and serialize the payload**
- [ ] **Step 3: Publish topic + payload frames on a ZMQ `PUB` socket**

### Task 4: Wire launch support and verify

**Files:**
- Modify: `src/xsens_mvn_ros2/launch/xsens_client.launch.py`
- Modify: `src/xsens_mvn_ros2/CMakeLists.txt`

- [ ] **Step 1: Add launch arguments for enabling and configuring the bridge**
- [ ] **Step 2: Run package build and focused tests**
- [ ] **Step 3: Fix any integration issues surfaced by build/test**
