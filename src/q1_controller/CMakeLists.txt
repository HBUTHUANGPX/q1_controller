cmake_minimum_required(VERSION 3.8)
project(q1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(USE_BACKEND "TensorRT" CACHE STRING "Select inference backend: OpenVINO, ONNX, TensorRT")

set(INFERENCE_LIBS "")
set(INFERENCE_INCLUDE_DIRS "")
if(USE_BACKEND STREQUAL "OpenVINO")
  add_definitions(-DUSE_OPENVINO)
  find_package(OpenVINO REQUIRED)
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  set(INFERENCE_LIBS ${OpenVINO_LIBRARIES} /usr/lib/libopenvino.so)
elseif(USE_BACKEND STREQUAL "ONNX")
  add_definitions(-DUSE_ONNX)
  set(ONNXRUNTIME_ROOTDIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime)
  include_directories(${ONNXRUNTIME_ROOTDIR}/include)
  link_directories(${ONNXRUNTIME_ROOTDIR}/lib)
  set(INFERENCE_LIBS ${ONNXRUNTIME_ROOTDIR}/lib/libonnxruntime.so)
  set(INFERENCE_INCLUDE_DIRS ${ONNXRUNTIME_ROOTDIR}/include)
elseif(USE_BACKEND STREQUAL "TensorRT")
  set(CMAKE_C_FLAGS "-D_GNU_SOURCE -D_REENTRANT -fasynchronous-unwind-tables -D__QIUNIU__")
  add_definitions(-DUSE_TENSORRT)
  # TensorRT路径（Jetson JetPack专用）
  set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
  set(TENSORRT_LIB_DIR /usr/lib/aarch64-linux-gnu)
  
  # CUDA路径（添加头文件包含）
  set(CUDA_INCLUDE_DIR /usr/local/cuda/include)
  set(CUDA_LIB_DIR /usr/local/cuda/lib64)

  include_directories(
    ${TENSORRT_ROOTDIR}/include
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIR}
  )
  link_directories(${TENSORRT_ROOTDIR}/lib)
  link_directories(
    ${TENSORRT_LIB_DIR}
    ${CUDA_LIB_DIR}
  )
  set(INFERENCE_LIBS 
    nvinfer
    nvonnxparser
    cudart
  )
  set(INFERENCE_INCLUDE_DIRS ${TENSORRT_ROOTDIR}/include)
  message(STATUS "Using TensorRT from: ${TENSORRT_ROOTDIR}")

  find_package(communication REQUIRED)
  find_package(Necro REQUIRED)
  find_package(Threads REQUIRED)
else()
  message(FATAL_ERROR "Invalid backend: ${USE_BACKEND}")
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # 支持Python
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)  # 添加 zlib 依赖
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${INFERENCE_INCLUDE_DIRS}
)

add_library(cnpy STATIC
  src/cnpy/cnpy.cpp 
)
target_include_directories(cnpy PUBLIC
  include/cnpy  
)
target_link_libraries(cnpy ZLIB::ZLIB) 


set(rl_include
  include/q1_controller/network_io.hpp
  include/q1_controller/inference_base.hpp
  include/q1_controller/mlp_network_io.hpp
  include/q1_controller/observation_base.hpp
  include/q1_controller/observation_manager.hpp
  include/q1_controller/data_store.hpp
  include/q1_controller/motor_base.hpp
  include/q1_controller/motor_manager.hpp
  include/q1_controller/motion_loader.hpp
  include/q1_controller/actual_virtual_map.hpp
  include/q1_controller/FSM_manager.hpp

  include/q1_controller/obs/base_ang_vel.hpp
  include/q1_controller/obs/joint_pos.hpp
  include/q1_controller/obs/joint_vel.hpp
  include/q1_controller/obs/last_actions.hpp
  include/q1_controller/obs/motion_joint_pos_command.hpp
  include/q1_controller/obs/motion_joint_vel_command.hpp
  include/q1_controller/obs/motion_ref_ori_b.hpp
  include/q1_controller/obs/gravity_orientation.hpp
  include/q1_controller/obs/sin_cos.hpp
  include/q1_controller/obs/cmd_vel.hpp
  include/rl_control.hpp
  
  src/q1_controller/network_io.cpp
  src/q1_controller/inference_base.cpp
  src/q1_controller/mlp_network_io.cpp
  src/q1_controller/observation_base.cpp
  src/q1_controller/observation_manager.cpp
  src/q1_controller/data_store.cpp
  src/q1_controller/motor_base.cpp
  src/q1_controller/motor_manager.cpp
  src/q1_controller/motion_loader.cpp
  src/q1_controller/actual_virtual_map.cpp
  src/q1_controller/FSM_manager.cpp
  src/rl_control.cpp
)
# 条件添加后端特定源文件和头文件
if(USE_BACKEND STREQUAL "OpenVINO")
  list(APPEND rl_include 
    include/q1_controller/openvino_inference.hpp
    src/q1_controller/openvino_inference.cpp
  )
  message(WARN "Using OpenVINO")
elseif(USE_BACKEND STREQUAL "ONNX")
  list(APPEND rl_include 
    include/q1_controller/onnx_inference.hpp
    src/q1_controller/onnx_inference.cpp
  )
  message(WARN "Using ONNX Runtime from: ${ONNXRUNTIME_ROOTDIR}")
elseif(USE_BACKEND STREQUAL "TensorRT")
  list(APPEND rl_include 
    include/q1_controller/tensorrt_inference.hpp
    src/q1_controller/tensorrt_inference.cpp  # 假设您将新代码置于此文件
  )
else()
  message(FATAL_ERROR "no")
endif()


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SingleMotorCommand.msg"
  "msg/MultiMotorCommand.msg"
  "msg/Dataset.msg"
  "msg/XboxJoy.msg"
  "srv/StateTransition.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs # 如果依赖其他消息包
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(rl_motor ${rl_include} test/test_motor.cpp)

if(USE_BACKEND STREQUAL "ONNX")
  set_target_properties(rl_motor PROPERTIES
    INSTALL_RPATH "${ONNXRUNTIME_ROOTDIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
elseif(USE_BACKEND STREQUAL "TensorRT")
  set_target_properties(rl_motor PROPERTIES
    INSTALL_RPATH "${TENSORRT_ROOTDIR}/lib:/usr/local/cuda/lib64"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  # 设置C编译标志
  target_compile_definitions(rl_motor PRIVATE
      _GNU_SOURCE
      _REENTRANT
      __QIUNIU__
  )
  target_compile_options(rl_motor PRIVATE
      -fasynchronous-unwind-tables
  )
  # 包含头文件路径
  target_include_directories(rl_motor PRIVATE
    ${communication_INCLUDE_DIRS}
  )
target_link_libraries(rl_motor
  ${communication_LIBRARIES}
  Threads::Threads
  Necro::qiuniu
)
necro_target_bootstrap(rl_motor)
endif()
target_link_libraries(rl_motor
    ${INFERENCE_LIBS}
    yaml-cpp 
    "${cpp_typesupport_target}"
    ZLIB::ZLIB  # 显式链接 zlib
    cnpy  # 链接本地 cnpy 库
)
ament_target_dependencies(rl_motor 
  rclcpp 
  sensor_msgs 
  std_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS rl_motor DESTINATION lib/${PROJECT_NAME})


if(USE_BACKEND STREQUAL "ONNX")
  add_executable(rl_onnx test/test_onnx.cpp)
  set_target_properties(rl_onnx PROPERTIES
    INSTALL_RPATH "${ONNXRUNTIME_ROOTDIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  target_link_libraries(rl_onnx
      ${INFERENCE_LIBS}
  )
  ament_target_dependencies(rl_onnx rclcpp sensor_msgs std_msgs)
  install(TARGETS rl_onnx DESTINATION lib/${PROJECT_NAME})
elseif(USE_BACKEND STREQUAL "TensorRT")
  add_executable(rl_tensorrt test/test_trt.cpp)  # 新可执行文件，用于测试TensorRT
  set_target_properties(rl_tensorrt PROPERTIES
    INSTALL_RPATH "${TENSORRT_ROOTDIR}/lib:/usr/local/cuda/lib64"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  target_link_libraries(rl_tensorrt
    ${INFERENCE_LIBS}
    # 其他原有库，如yaml-cpp等
  )
  ament_target_dependencies(rl_tensorrt rclcpp sensor_msgs std_msgs)
  install(TARGETS rl_tensorrt DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(simple_node test/test_simple_node.cpp)
ament_target_dependencies(simple_node rclcpp sensor_msgs std_msgs)
install(TARGETS rl_motor DESTINATION lib/${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  q1_controller/bringup_mujoco.py
  q1_controller/bringup_joystick.py
  q1_controller/bringup_webserver.py
  q1_controller/gamepad_publisher.py
  q1_controller/tts_say.py
  q1_controller/voice_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)
# 安装目录
install(DIRECTORY
  launch config msg urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()