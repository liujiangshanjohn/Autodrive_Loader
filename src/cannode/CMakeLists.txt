# CMake version
cmake_minimum_required(VERSION 3.8.2)
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w -O3 -g -Wreturn-type")

project(cannode)

# CMake root display
message(STATUS "CMAKE_ROOT: " ${CMAKE_ROOT})

# Turn on the verbose
set(CMAKE_VERBOSE_MAKEFILE ON)

#add_compile_options("-w")

# module name
set(MODULE_NAME "cannode")

set(COLCON_COMPILE "on")

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

file(GLOB_RECURSE
  SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

# 添加自定义 CMake 模块路径（查找 absl）
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${WORKSPACE_ROOT}/cmake_modules")
list(APPEND CMAKE_PREFIX_PATH "${WORKSPACE_ROOT}/cmake_modules")

# find package
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cyber_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

#set root dir
set(root_dir ${CMAKE_CURRENT_SOURCE_DIR})
# include(${root_dir}/cmake/function.cmake)
if(NOT EXISTS ${root_dir}/protobuf/out)
  make_directory(${root_dir}/protobuf/out)
endif()

if(PROTOBUF_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  message(STATUS "Protobuf include dirs: ${PROTOBUF_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries: ${PROTOBUF_LIBRARIES}")
  message(STATUS "Protobuf protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")
else()
  message(FATAL_ERROR "Protobuf not found")
endif()

# 显式加入新增加的车辆协议源文件，避免 CMake GLOB 未重新评估导致未参与链接
list(APPEND SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vehicle_protocol/xiaosong.cc
)

# 排除 multi_tilt_imu_node.cpp、danfoss_rotary_encoder_node.cpp 和 engine_feedback_node.cpp，
# 因为它们有自己独立的可执行文件
list(REMOVE_ITEM SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/multi_tilt_imu/multi_tilt_imu_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/danfoss_rotary_encoder/danfoss_rotary_encoder_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/engine_feedback_node.cpp
)

# 先生成消息接口（必须在定义可执行文件之前）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TiltRPY.msg"
  "msg/RotaryEncoderAngle.msg"
)

# 现在定义可执行文件（使用不同的目标名称避免与消息接口库冲突）
add_executable(${MODULE_NAME}_exe ${SOURCES})

# compile proto file to generate cpp file
set(proto_dir ${CMAKE_CURRENT_SOURCE_DIR}/protobuf)
file(GLOB_RECURSE MSG_PROTOS "${proto_dir}/proto/*.proto")
foreach(proto_file ${MSG_PROTOS})
  execute_process(
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    --proto_path=${proto_dir}/proto/
    --cpp_out=${proto_dir}/out/
    ${proto_file}
  )
  # 同时生成 Python 文件
  execute_process(
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    --proto_path=${proto_dir}/proto/
    --python_out=${proto_dir}/out/
    ${proto_file}
  )
endforeach()

# include_sub_directories_recursively(${root_dir})


# set compile option
target_compile_options(${MODULE_NAME}_exe PRIVATE -Wall)

ament_target_dependencies(${MODULE_NAME}_exe
  PUBLIC
  rclcpp
  std_msgs
  cyber_msgs
  geometry_msgs
  # sensor_msgs
)

target_include_directories(${MODULE_NAME}_exe
  PRIVATE
  ${OSAL_INC_DIR}
  ${SHARED_INC_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/protobuf/out
)

# 让主可执行文件能够使用自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target_main
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${MODULE_NAME}_exe PRIVATE "${cpp_typesupport_target_main}")


# message(FATAL_ERROR ${PROJ_LIBRARIES})
# link lib
target_link_libraries(
  ${MODULE_NAME}_exe
  PRIVATE
  Threads::Threads
  ${PROTOBUF_LIBRARIES}
  ${absl_LIBRARIES}
  yaml-cpp
)

# 添加 multi_tilt_imu_node 可执行文件
add_executable(multi_tilt_imu_node
  src/multi_tilt_imu/multi_tilt_imu_node.cpp
)

ament_target_dependencies(multi_tilt_imu_node
  rclcpp
  geometry_msgs
)

target_include_directories(multi_tilt_imu_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/multi_tilt_imu/include
)

# 让 multi_tilt_imu_node 能够使用本包中的自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target_multi
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# 注意：必须使用 plain signature，因为 ament_target_dependencies 已经使用了 plain signature
target_link_libraries(multi_tilt_imu_node "${cpp_typesupport_target_multi}")

# 添加 danfoss_rotary_encoder_node 可执行文件
add_executable(danfoss_rotary_encoder_node
  src/danfoss_rotary_encoder/danfoss_rotary_encoder_node.cpp
)

ament_target_dependencies(danfoss_rotary_encoder_node
  rclcpp
)

target_include_directories(danfoss_rotary_encoder_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/danfoss_rotary_encoder/include
)

# 让 danfoss_rotary_encoder_node 能够使用本包中的自定义消息类型
rosidl_get_typesupport_target(cpp_typesupport_target_danfoss
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(danfoss_rotary_encoder_node "${cpp_typesupport_target_danfoss}")

# 添加 engine_feedback_node 可执行文件（独立节点，从 CAN1 解析发动机/VCU 反馈并发布到 ROS topic）
add_executable(engine_feedback_node
  src/engine_feedback_node.cpp
  src/socket_can_interface.cc
)

ament_target_dependencies(engine_feedback_node
  rclcpp
  std_msgs
)

target_include_directories(engine_feedback_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS
  ${MODULE_NAME}_exe
  multi_tilt_imu_node
  danfoss_rotary_encoder_node
  engine_feedback_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 倾角/编码器 CSV 记录脚本（python3 .../share/cannode/scripts/tilt_encoder_csv_logger.py）
install(PROGRAMS
  scripts/tilt_encoder_csv_logger.py
  scripts/record_vehicle_session.sh
  scripts/f710_joy_controller.py
  scripts/run_f710_joy_controller.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(PROGRAMS
  scripts/f710_joy_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()








