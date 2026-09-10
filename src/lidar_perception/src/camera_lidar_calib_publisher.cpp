#include <chrono>
#include <array>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace lidar_perception
{

class CameraLidarCalibPublisher final : public rclcpp::Node
{
public:
  explicit CameraLidarCalibPublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("camera_lidar_calib_publisher", options)
  {
    topic_camera_info_ = this->declare_parameter<std::string>("camera_info_topic", "/camera_main/camera_info");
    camera_frame_id_ = this->declare_parameter<std::string>("camera_frame_id", "camera_main");

    width_ = this->declare_parameter<int>("width", 1920);
    height_ = this->declare_parameter<int>("height", 1080);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 10.0);
    distortion_model_ = this->declare_parameter<std::string>("distortion_model", "equidistant");

    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      topic_camera_info_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_)));
    timer_ = this->create_wall_timer(period, [this]() { publish_camera_info(); });
  }

private:
  void publish_camera_info()
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = camera_frame_id_;
    msg.width = static_cast<uint32_t>(std::max(0, width_));
    msg.height = static_cast<uint32_t>(std::max(0, height_));

    // From cam_geac/cam_geac/src/camera_undistort.cpp:72-78
    // fish_left_parak:
    // fx=506.54441832512219, fy=506.86881935332809, cx=961.89672515531925, cy=536.50503927715340
    const double fx = 5.0654441832512219e+02;
    const double fy = 5.0686881935332809e+02;
    const double cx = 9.6189672515531925e+02;
    const double cy = 5.3650503927715340e+02;

    msg.distortion_model = distortion_model_;

    msg.d = std::vector<double>{
      1.3862807471232097e-01,
      -3.3476673081807767e-02,
      -3.0428483646121299e-03,
      1.3030662790972547e-03};

    msg.k = std::array<double, 9>{
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0};

    msg.r = std::array<double, 9>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

    msg.p = std::array<double, 12>{
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0};

    camera_info_pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string topic_camera_info_;
  std::string camera_frame_id_;
  std::string distortion_model_;

  int width_{1920};
  int height_{1080};
  double publish_hz_{10.0};
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_perception::CameraLidarCalibPublisher>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

