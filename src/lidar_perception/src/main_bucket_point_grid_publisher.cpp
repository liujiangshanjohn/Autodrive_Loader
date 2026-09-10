#include <rclcpp/rclcpp.hpp>

#include "bucket_point_grid_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_perception::BucketPointGridPublisher>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

