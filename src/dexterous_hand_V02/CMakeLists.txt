cmake_minimum_required(VERSION 3.8)
project(dexterous_hand)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-parameter)
endif()

# 依赖包
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ROSIDL_TYPESUPPORT_CPP_BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate custom messages
set(msg_files
  "msg/HandFeedback.msg"
  "msg/HandControl.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 将msg生成的头文件目录，添加到编译器的包含路径，解决找不到头文件问题
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_typesupport_c)
ament_export_dependencies(rosidl_typesupport_cpp)

# 头文件路径
include_directories(
  include
)

# 新增 src/main.cpp 即可，其他源文件保留
add_executable(hand_node
  src/socket_can.cpp
  #src/aoyi_can.cpp
  src/aoyi_hand.cpp
  #src/OHandSerialAPI.c
  src/aoyi_protocal.cpp
  src/hand_node.cpp
  src/main.cpp
)

# ========== ament_target_dependencies 补全所有依赖 ==========
ament_target_dependencies(hand_node
  rclcpp
  std_msgs
  rosidl_typesupport_c
  rosidl_typesupport_cpp
)

# 将自定义msg的生成目标 链接 到你的节点，编译时自动关联msg头文件
# 这行是解决 "找不到msg头文件" 的终极关键！
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hand_node "${cpp_typesupport_target}")

# 安装节点
install(TARGETS
  hand_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()