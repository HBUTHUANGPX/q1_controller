cmake_minimum_required(VERSION 3.14)
project(xsens_mvn_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rcpputils REQUIRED)
find_package(xsens_mvn_ros2_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread chrono)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_subdirectory(xsens_mvn_sdk)

set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_FILES "${PROTO_DIR}/link_states.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(${PROJECT_NAME}_LinkStateProto SHARED
  src/link_states/LinkStateProtoSerializer.cpp
  ${PROTO_SRCS}
)
target_include_directories(${PROJECT_NAME}_LinkStateProto PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${Protobuf_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_LinkStateProto
  rclcpp xsens_mvn_ros2_msgs
)
target_link_libraries(${PROJECT_NAME}_LinkStateProto
  protobuf::libprotobuf
)

add_library(${PROJECT_NAME}_HumanDataHandler SHARED
  src/xsens_client/HumanDataHandler.cpp
)
target_include_directories(${PROJECT_NAME}_HumanDataHandler PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${CMAKE_CURRENT_SOURCE_DIR}/xsens_mvn_sdk/include"
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_HumanDataHandler
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs urdf tf2_eigen rcpputils xsens_mvn_ros2_msgs Eigen3
)

add_library(${PROJECT_NAME}_XSensClient SHARED
  src/xsens_client/XSensClient.cpp
  src/xsens_client/Socket.cpp
)
target_include_directories(${PROJECT_NAME}_XSensClient PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${CMAKE_CURRENT_SOURCE_DIR}/xsens_mvn_sdk/include"
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_XSensClient
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs urdf tf2_eigen rcpputils xsens_mvn_ros2_msgs Eigen3
)
target_link_libraries(${PROJECT_NAME}_XSensClient
  ${PROJECT_NAME}_HumanDataHandler
  xsens_mvn_sdk
  Boost::system
  Boost::thread
  Boost::chrono
)

add_executable(xsens_client
  src/xsens_client_node.cpp
)
target_include_directories(xsens_client PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${CMAKE_CURRENT_SOURCE_DIR}/xsens_mvn_sdk/include"
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(xsens_client
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs urdf tf2_eigen rcpputils xsens_mvn_ros2_msgs Eigen3
)
target_link_libraries(xsens_client
  ${PROJECT_NAME}_XSensClient
  Boost::system
  Boost::thread
  Boost::chrono
)

add_executable(link_states_zmq_bridge
  src/link_states_zmq_bridge.cpp
)
target_include_directories(link_states_zmq_bridge PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${ZMQ_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)
ament_target_dependencies(link_states_zmq_bridge
  rclcpp xsens_mvn_ros2_msgs
)
target_link_libraries(link_states_zmq_bridge
  ${PROJECT_NAME}_LinkStateProto
  protobuf::libprotobuf
  ${ZMQ_LIBRARIES}
)

add_executable(link_states_zmq_subscriber
  src/link_states_zmq_subscriber.cpp
)
target_include_directories(link_states_zmq_subscriber PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${ZMQ_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(link_states_zmq_subscriber
  ${PROJECT_NAME}_LinkStateProto
  protobuf::libprotobuf
  ${ZMQ_LIBRARIES}
)

add_executable(link_states_test_publisher
  src/link_states_test_publisher.cpp
)
target_include_directories(link_states_test_publisher PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(link_states_test_publisher
  rclcpp xsens_mvn_ros2_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}_HumanDataHandler ${PROJECT_NAME}_XSensClient ${PROJECT_NAME}_LinkStateProto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS xsens_client link_states_zmq_bridge link_states_zmq_subscriber link_states_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${PROTO_FILES}
  DESTINATION share/${PROJECT_NAME}/proto
)

install(DIRECTORY launch/ rviz/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
  FILES_MATCHING PATTERN "*.py"
)
install(PROGRAMS
  scripts/sub.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs urdf tf2_eigen rcpputils xsens_mvn_ros2_msgs rviz2 Eigen3 Boost
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_HumanDataHandler ${PROJECT_NAME}_XSensClient)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_parsermanager_cache
    test/test_parsermanager_cache.cpp
  )
  if(TARGET test_parsermanager_cache)
    target_include_directories(test_parsermanager_cache PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/xsens_mvn_sdk/include"
    )
    target_link_libraries(test_parsermanager_cache
      xsens_mvn_sdk
    )
  endif()

  ament_add_gtest(test_link_state_proto_serializer
    test/test_link_state_proto_serializer.cpp
  )
  if(TARGET test_link_state_proto_serializer)
    target_include_directories(test_link_state_proto_serializer PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
      "${CMAKE_CURRENT_BINARY_DIR}"
    )
    ament_target_dependencies(test_link_state_proto_serializer
      rclcpp xsens_mvn_ros2_msgs
    )
    target_link_libraries(test_link_state_proto_serializer
      ${PROJECT_NAME}_LinkStateProto
      protobuf::libprotobuf
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
