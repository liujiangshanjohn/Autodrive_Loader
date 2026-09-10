#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lidar_perception
{

/// 订阅 /rslidar_points，原样转发点云数据，仅将 header.stamp 统一为节点当前时间（便于与 TF / 其它传感器对齐）。
class RslidarPointsFixedNode final : public rclcpp::Node
{
public:
  explicit RslidarPointsFixedNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("rslidar_points_fixed", options)
  {
    const std::string in_topic = this->declare_parameter<std::string>("input_topic", "/rslidar_points");
    const std::string out_topic = this->declare_parameter<std::string>("output_topic", "/rslidar_points_fixed");
    const int qos_depth = this->declare_parameter<int>("qos_depth", 10);

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth))));
    qos.reliable();

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, qos);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 out = *msg;
        out.header.stamp = this->now();
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_perception::RslidarPointsFixedNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
