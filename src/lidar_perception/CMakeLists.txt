cmake_minimum_required(VERSION 3.8)
project(lidar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cyber_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cannode REQUIRED)

add_library(bucket_point_grid_publisher_component SHARED
  src/bucket_point_grid_publisher_clean.cpp
)
target_compile_features(bucket_point_grid_publisher_component PUBLIC cxx_std_17)
ament_target_dependencies(bucket_point_grid_publisher_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cyber_msgs
  cannode
)

rclcpp_components_register_nodes(bucket_point_grid_publisher_component
  "lidar_perception::BucketPointGridPublisher")

add_executable(bucket_point_grid_publisher
  src/main_bucket_point_grid_publisher.cpp
)
target_compile_features(bucket_point_grid_publisher PUBLIC cxx_std_17)
target_link_libraries(bucket_point_grid_publisher bucket_point_grid_publisher_component)
ament_target_dependencies(bucket_point_grid_publisher rclcpp)

add_executable(camera_lidar_calib_publisher
  src/camera_lidar_calib_publisher.cpp
)
target_compile_features(camera_lidar_calib_publisher PUBLIC cxx_std_17)
ament_target_dependencies(camera_lidar_calib_publisher
  rclcpp
  sensor_msgs
)

add_executable(lidar_cloud_to_point_grid
  src/lidar_cloud_to_point_grid.cpp
)
target_compile_features(lidar_cloud_to_point_grid PUBLIC cxx_std_17)
ament_target_dependencies(lidar_cloud_to_point_grid
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  cyber_msgs
)

add_executable(rslidar_points_fixed
  src/rslidar_points_fixed.cpp
)
target_compile_features(rslidar_points_fixed PUBLIC cxx_std_17)
ament_target_dependencies(rslidar_points_fixed
  rclcpp
  sensor_msgs
)

add_executable(lidar_arm_bucket_2d_grid
  src/lidar_arm_bucket_2d_grid.cpp
)
target_compile_features(lidar_arm_bucket_2d_grid PUBLIC cxx_std_17)
ament_target_dependencies(lidar_arm_bucket_2d_grid
  rclcpp
  sensor_msgs
  geometry_msgs
  cyber_msgs
  visualization_msgs
)

install(TARGETS
  bucket_point_grid_publisher_component
  bucket_point_grid_publisher
  camera_lidar_calib_publisher
  lidar_cloud_to_point_grid
  rslidar_points_fixed
  lidar_arm_bucket_2d_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

