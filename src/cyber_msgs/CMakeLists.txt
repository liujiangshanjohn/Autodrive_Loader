cmake_minimum_required(VERSION 3.5)
project(cyber_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0048 NEW)

# =======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
# =======================================
# set(COMPILE_METHOD CATKIN)
# set(COMPILE_METHOD COLCON)

# ROS#
# find_package(roscpp 1.12 QUIET)
find_package(roscpp QUIET)

if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found. ROS Support is turned On!")
  message(=============================================================)

  add_definitions(-DROS1_FOUND)

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
  )
  file(GLOB_RECURSE MSG_FILES_ABS "msg/*.msg")
  set(MSG_FILES "")

  foreach(abs_path ${MSG_FILES_ABS})
    file(RELATIVE_PATH rel_path "${CMAKE_CURRENT_LIST_DIR}/msg" "${abs_path}")
    list(APPEND MSG_FILES "${rel_path}")
  endforeach()

  message(STATUS MSG_FILES ${MSG_FILES})

  add_message_files(FILES ${MSG_FILES})
  generate_messages(
    DEPENDENCIES
    std_msgs sensor_msgs geometry_msgs nav_msgs
  )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
  catkin_package(CATKIN_DEPENDS roscpp
    std_msgs
    geometry_msgs
    nav_msgs)

else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)

# ROS2#
find_package(rclcpp QUIET)

if(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On!")
  message(=============================================================)
  
  execute_process(
    COMMAND cp ${CMAKE_CURRENT_LIST_DIR}/package_ros2.xml ${CMAKE_CURRENT_LIST_DIR}/package.xml
    RESULT_VARIABLE result
    ERROR_VARIABLE error_output
  )
  add_definitions(-DROS2_FOUND)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  file(GLOB MSG_FILES_ABS "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")
  set(MSG_FILES "")

  foreach(abs_path ${MSG_FILES_ABS})
    file(RELATIVE_PATH rel_path "${CMAKE_CURRENT_SOURCE_DIR}" "${abs_path}")
    list(APPEND MSG_FILES "${rel_path}")
  endforeach()

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    DEPENDENCIES std_msgs geometry_msgs nav_msgs
  )
  ament_package()

else(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Not Found. Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND)
