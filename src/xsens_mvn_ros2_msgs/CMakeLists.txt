cmake_minimum_required(VERSION 3.14)
project(xsens_mvn_ros2_msgs)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LinkState.msg"
  "msg/LinkStateArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 可选：添加代码风格检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出包
ament_package()