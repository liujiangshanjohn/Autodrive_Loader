#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
 
#include <geometry_msgs/msg/point32.hpp>
#include <cyber_msgs/msg/point_grid.hpp>
 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
 
namespace lidar_perception
{
 
class LidarCloudToPointGrid : public rclcpp::Node
{
public:
  explicit LidarCloudToPointGrid(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lidar_cloud_to_point_grid", options)
  {
    declareAndLoadParams();
 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2));
    qos.best_effort();
 
    pub_ = create_publisher<cyber_msgs::msg::PointGrid>(output_topic_, rclcpp::QoS(2));
 
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(std::move(msg)); });
  }
 
private:
  struct Pinhole
  {
    std::string camera_frame{"camera_main"};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int img_width{1920};
    int img_height{1080};
    std::string pixel_origin{"top_left"};  // top_left or bottom_right
    std::string forward_axis{"z"};         // "z" (optical) or "x"
    double forward_sign{1.0};
    double down_sign{1.0};
    bool clamp_to_image{false};
    double u_offset_px{0.0};
    double v_offset_px{0.0};
  };
 
  struct StaticTf
  {
    std::array<double, 3> xyz_m{0.0, 0.0, 0.0};
    std::array<double, 3> rpy_deg{0.0, 0.0, 0.0};
  };

  // static_tfs.1 描述的变换方向：
  // - "lidar_to_camera": static_tfs.1 is lidar_T_camera, so we need camera_T_lidar = inverse.
  // - "camera_to_lidar": static_tfs.1 is camera_T_lidar, so we can use it directly.
  std::string extrinsic_direction_{"lidar_to_camera"};
 
  void declareAndLoadParams()
  {
    declare_parameter("cloud_topic", cloud_topic_);
    declare_parameter("output_topic", output_topic_);
    get_parameter("cloud_topic", cloud_topic_);
    get_parameter("output_topic", output_topic_);
 
    declare_parameter("camera_frame", pin_.camera_frame);
    declare_parameter("fx", pin_.fx);
    declare_parameter("fy", pin_.fy);
    declare_parameter("cx", pin_.cx);
    declare_parameter("cy", pin_.cy);
    declare_parameter("img_width", pin_.img_width);
    declare_parameter("img_height", pin_.img_height);
    declare_parameter("pixel_origin", pin_.pixel_origin);
    declare_parameter("forward_axis", pin_.forward_axis);
    declare_parameter("forward_sign", pin_.forward_sign);
    declare_parameter("down_sign", pin_.down_sign);
    declare_parameter("clamp_to_image", pin_.clamp_to_image);
    declare_parameter("u_offset_px", pin_.u_offset_px);
    declare_parameter("v_offset_px", pin_.v_offset_px);
    get_parameter("camera_frame", pin_.camera_frame);
    get_parameter("fx", pin_.fx);
    get_parameter("fy", pin_.fy);
    get_parameter("cx", pin_.cx);
    get_parameter("cy", pin_.cy);
    get_parameter("img_width", pin_.img_width);
    get_parameter("img_height", pin_.img_height);
    get_parameter("pixel_origin", pin_.pixel_origin);
    get_parameter("forward_axis", pin_.forward_axis);
    get_parameter("forward_sign", pin_.forward_sign);
    get_parameter("down_sign", pin_.down_sign);
    get_parameter("clamp_to_image", pin_.clamp_to_image);
    get_parameter("u_offset_px", pin_.u_offset_px);
    get_parameter("v_offset_px", pin_.v_offset_px);

    declare_parameter("extrinsic_direction", extrinsic_direction_);
    get_parameter("extrinsic_direction", extrinsic_direction_);
 
    // Use the same keys as bucket_point_grid_params.yaml static_tfs.1.*
    declare_parameter("static_tfs.1.xyz_m", std::vector<double>{});
    declare_parameter("static_tfs.1.rpy_deg", std::vector<double>{});
    std::vector<double> xyz, rpy;
    get_parameter("static_tfs.1.xyz_m", xyz);
    get_parameter("static_tfs.1.rpy_deg", rpy);
    if (xyz.size() == 3) {
      ext_.xyz_m = {xyz[0], xyz[1], xyz[2]};
    }
    if (rpy.size() == 3) {
      ext_.rpy_deg = {rpy[0], rpy[1], rpy[2]};
    }
 
    declare_parameter("max_points", max_points_);
    declare_parameter("stride", stride_);
    declare_parameter("min_range_m", min_range_m_);
    declare_parameter("max_range_m", max_range_m_);
    declare_parameter("use_aabb_roi", use_aabb_roi_);
    declare_parameter("roi_center_xyz_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("roi_size_xyz_m", std::vector<double>{10.0, 10.0, 10.0});
    get_parameter("max_points", max_points_);
    get_parameter("stride", stride_);
    get_parameter("min_range_m", min_range_m_);
    get_parameter("max_range_m", max_range_m_);
    get_parameter("use_aabb_roi", use_aabb_roi_);
    std::vector<double> roi_center, roi_size;
    get_parameter("roi_center_xyz_m", roi_center);
    get_parameter("roi_size_xyz_m", roi_size);
    if (roi_center.size() == 3) roi_center_xyz_m_ = {roi_center[0], roi_center[1], roi_center[2]};
    if (roi_size.size() == 3) roi_size_xyz_m_ = {roi_size[0], roi_size[1], roi_size[2]};
    max_points_ = std::max(0, max_points_);
    stride_ = std::max(1, stride_);
  }
 
  static tf2::Quaternion quatFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg)
  {
    constexpr double kDeg2Rad = M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(roll_deg * kDeg2Rad, pitch_deg * kDeg2Rad, yaw_deg * kDeg2Rad);
    q.normalize();
    return q;
  }
 
  bool projectCameraPointToPixel(const tf2::Vector3 & cam_p, float & u, float & v) const
  {
    const double x = cam_p.x();
    const double y = cam_p.y();
    const double z = cam_p.z();
 
    double fwd = z;
    double right = x;
    double down = y;
    if (pin_.forward_axis == "x") {
      fwd = x;
      right = -y;
      down = -z;
    }
 
    const double s = (pin_.forward_sign >= 0.0) ? 1.0 : -1.0;
    fwd *= s;
    right *= s;
    down *= s;
    down *= (pin_.down_sign >= 0.0) ? 1.0 : -1.0;
 
    if (!(fwd > 1e-6)) return false;
 
    double uu = pin_.fx * (right / fwd) + pin_.cx;
    double vv = pin_.fy * (down / fwd) + pin_.cy;
 
    if (pin_.pixel_origin == "bottom_right") {
      uu = (static_cast<double>(pin_.img_width) - 1.0) - uu;
      vv = (static_cast<double>(pin_.img_height) - 1.0) - vv;
    }
 
    uu += pin_.u_offset_px;
    vv += pin_.v_offset_px;
 
    if (pin_.clamp_to_image) {
      uu = std::min(std::max(uu, 0.0), static_cast<double>(pin_.img_width - 1));
      vv = std::min(std::max(vv, 0.0), static_cast<double>(pin_.img_height - 1));
    }
 
    u = static_cast<float>(uu);
    v = static_cast<float>(vv);
    return true;
  }
 
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) return;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "onCloud: got PointCloud2 frame_id=%s w=%u h=%u fields=%zu",
      msg->header.frame_id.c_str(), msg->width, msg->height, msg->fields.size());
 
    // Build transform from static_tfs.1.
    tf2::Transform lidar_T_cam;
    lidar_T_cam.setOrigin(tf2::Vector3{ext_.xyz_m[0], ext_.xyz_m[1], ext_.xyz_m[2]});
    lidar_T_cam.setRotation(quatFromRpyDeg(ext_.rpy_deg[0], ext_.rpy_deg[1], ext_.rpy_deg[2]));

    // cam_T_lidar is camera_T_lidar.
    // If static_tfs.1 is lidar_T_camera, camera_T_lidar is inverse.
    // If static_tfs.1 is camera_T_lidar, use it directly.
    const tf2::Transform cam_T_lidar =
      (extrinsic_direction_ == "camera_to_lidar") ? lidar_T_cam : lidar_T_cam.inverse();
 
    cyber_msgs::msg::PointGrid out;
    out.points.clear();
 
    // Iterators will throw if fields are missing; guard explicitly.
    auto has_field = [msg](const char * name) -> bool {
      for (const auto & f : msg->fields) {
        if (f.name == name) return true;
      }
      return false;
    };
    if (!has_field("x") || !has_field("y") || !has_field("z")) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 缺少 x/y/z 字段，无法投影");
      return;
    }
 
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
 
    int projected = 0;
    int in_image = 0;
    float umin = std::numeric_limits<float>::infinity();
    float vmin = std::numeric_limits<float>::infinity();
    float umax = -std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();

    int kept = 0;
    int idx = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++idx) {
      if ((idx % stride_) != 0) continue;
      const float lx = *it_x;
      const float ly = *it_y;
      const float lz = *it_z;
      if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) continue;
 
      if (use_aabb_roi_) {
        const double cx = roi_center_xyz_m_[0];
        const double cy = roi_center_xyz_m_[1];
        const double cz = roi_center_xyz_m_[2];
        const double sx = std::max(0.0, roi_size_xyz_m_[0]);
        const double sy = std::max(0.0, roi_size_xyz_m_[1]);
        const double sz = std::max(0.0, roi_size_xyz_m_[2]);
        const double hx = 0.5 * sx;
        const double hy = 0.5 * sy;
        const double hz = 0.5 * sz;
        if (lx < cx - hx || lx > cx + hx || ly < cy - hy || ly > cy + hy || lz < cz - hz || lz > cz + hz) {
          continue;
        }
      }

      const double r = std::sqrt(static_cast<double>(lx) * lx + static_cast<double>(ly) * ly + static_cast<double>(lz) * lz);
      if (r < min_range_m_ || r > max_range_m_) continue;
 
      const tf2::Vector3 cam_p = cam_T_lidar * tf2::Vector3{lx, ly, lz};
      float u = 0.0f, v = 0.0f;
      if (!projectCameraPointToPixel(cam_p, u, v)) continue;
      projected++;
      umin = std::min(umin, u);
      vmin = std::min(vmin, v);
      umax = std::max(umax, u);
      vmax = std::max(vmax, v);
      if (u >= 0.0f && v >= 0.0f && u < static_cast<float>(pin_.img_width) &&
          v < static_cast<float>(pin_.img_height)) {
        in_image++;
      }
 
      geometry_msgs::msg::Point32 p;
      p.x = u;
      p.y = v;
      p.z = 0.0f;
      out.points.push_back(p);
      kept++;
      if (max_points_ > 0 && kept >= max_points_) break;
    }
 
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "proj: input=%u stride=%d projected=%d kept=%d in_img=%d bbox=[%.1f,%.1f]-[%.1f,%.1f] "
      "fwd_axis=%s fwd_sign=%.0f down_sign=%.0f extrinsic_direction=%s",
      msg->width * msg->height, stride_, projected, kept, in_image,
      std::isfinite(umin) ? umin : 0.0f, std::isfinite(vmin) ? vmin : 0.0f,
      std::isfinite(umax) ? umax : 0.0f, std::isfinite(vmax) ? vmax : 0.0f,
      pin_.forward_axis.c_str(), pin_.forward_sign, pin_.down_sign, extrinsic_direction_.c_str());

    pub_->publish(out);
  }
 
  std::string cloud_topic_{"/rslidar_points_fixed"};
  std::string output_topic_{"/cyber_msgs/point_grid"};
 
  Pinhole pin_;
  StaticTf ext_;
 
  int max_points_{2000};   // 0 -> unlimited (not recommended for HUD)
  int stride_{5};          // take every Nth point
  double min_range_m_{0.3};
  double max_range_m_{80.0};
  bool use_aabb_roi_{true};
  std::array<double, 3> roi_center_xyz_m_{0.0, 0.0, 0.0};
  std::array<double, 3> roi_size_xyz_m_{10.0, 10.0, 10.0};
 
  rclcpp::Publisher<cyber_msgs::msg::PointGrid>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
 
}  // namespace lidar_perception
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_perception::LidarCloudToPointGrid>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

