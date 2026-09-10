#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cyber_msgs/msg/point_grid.hpp>

#include <cannode/msg/rotary_encoder_angle.hpp>
#include <cannode/msg/tilt_rpy.hpp>

namespace lidar_perception
{

class BucketPointGridPublisher : public rclcpp::Node
{
public:
  explicit BucketPointGridPublisher(const rclcpp::NodeOptions & options);

private:
  struct Frames
  {
    std::string base_link{"base_link"};
    std::string upper_link{"upper_link"};
    std::string boom_link{"boom_link"};
    std::string stick_link{"stick_link"};
    std::string bucket_link{"bucket_link"};
    std::string bucket_open{"bucket_open"};
  };

  struct PointGridConfig
  {
    std::string camera_frame{"camera_main"};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    // Post-projection pixel translation (for quick alignment/debug).
    // Applied after pinhole projection and pixel_origin handling, before clamp.
    double u_offset_px{0.0};
    double v_offset_px{0.0};
    // Publish modes:
    // - use_ground_quad: publish 4 projected pixel corners of a ground rectangle (for filled overlay)
    // - use_pixel_grid: publish a fixed pixel-spacing N×N grid around projected center (visual debug)
    // - default: publish a fixed-metric ground grid (10×10) projected to pixels
    bool use_ground_quad{false};
    double ground_quad_length_m{3.0};
    double ground_quad_width_m{3.0};
    // Align ground quad orientation with upper_link yaw (swing), to make view more stable.
    bool ground_quad_align_upper_yaw{true};
    // Distance normalization (approx): scale ground quad by (fwd / ref_fwd) to keep pixel size stable.
    bool distance_normalize{false};
    double normalize_ref_fwd_m{10.0};
    double normalize_min_scale{0.5};
    double normalize_max_scale{2.0};
    // If true, publish a dotted quad (N×N points) in pixel space inside the projected quad,
    // instead of publishing only 4 corner points. This works with cam_geac's "draw crosses" path.
    bool ground_quad_dotted{true};
    int ground_quad_dotted_n{10};
    // Stable UI mode: publish a fixed pixel-size dotted rectangle centered at projected point.
    bool use_pixel_quad{false};
    double pixel_quad_width_px{400.0};
    double pixel_quad_height_px{400.0};
    int pixel_quad_dotted_n{10};
    // Stabilize the projected center used by pixel_quad/pixel_grid.
    bool stabilize_center{true};
    // Low-pass filter factor in [0,1]. 0 -> no update, 1 -> no smoothing.
    double center_lpf_alpha{0.35};
    // If projection invalid / off-screen, keep last valid center.
    bool center_hold_on_invalid{true};
    // Clamp center inside image with this margin (px) to avoid jumping to extremes.
    double center_margin_px{40.0};
    bool use_pixel_grid{false};
    int pixel_grid_n{10};
    double pixel_grid_spacing_px{30.0};
    int img_width{1920};
    int img_height{1080};
    std::string pixel_origin{"top_left"};  // top_left or bottom_right
    // Which axis is camera "forward" in camera_frame for pinhole projection.
    // - "z": optical frame convention (X right, Y down, Z forward)
    // - "x": robot body convention (X forward, Y left, Z up)
    std::string forward_axis{"z"};
    // Some camera frames define "forward" as negative axis (e.g. -Z).
    // Set to -1.0 to flip (forward,right,down) consistently.
    double forward_sign{1.0};
    // Flip image vertical direction if your camera frame uses +Y up (instead of +Y down).
    // Applied after forward_axis mapping and forward_sign.
    double down_sign{1.0};
    bool clamp_to_image{true};

    // If true, do NOT pad/resize to 100 points at the end.
    // Needed for "landing grid" format where we want 4 corners + N×N grid points.
    bool keep_point_count{false};

    bool debug_force_fixed_uv{false};
    int debug_u{800};
    int debug_v{400};
    int debug_span_px{100};
  };

  struct Kinematics
  {
    std::array<double, 3> swing_center_xyz_m{0.0, 0.0, 0.0};
    std::array<double, 3> boom_joint_xyz_m{0.0, 0.0, 0.0};
    double boom_length_m{0.0};
    double stick_length_m{0.0};
    std::array<double, 3> bucket_open_xyz_m{0.0, 0.0, 0.0};
  };

  struct Grid
  {
    double resolution_m{0.1};
    double width_m{3.22};
    double length_m{3.22};
  };

  struct StaticTf
  {
    std::string parent;
    std::string child;
    std::array<double, 3> xyz_m;
    std::array<double, 3> rpy_deg;
  };

  struct TiltTopics
  {
    std::string boom{"/tilt_boom"};
    std::string stick{"/tilt_stick"};
    std::string bucket{"/tilt_bucket"};
    std::string body{"/tilt_body"};
  };

  struct TiltAxes
  {
    std::string boom{"pitch"};
    std::string stick{"pitch"};
    std::string bucket{"pitch"};
    std::string body{"yaw"};
  };

  struct TiltZerosDeg
  {
    double boom{0.0};
    double stick{0.0};
    double bucket{0.0};
    double body_roll{0.0};
    double body_pitch{0.0};
    double body_yaw{0.0};
  };

  struct RotaryEncoderConfig
  {
    std::string topic{"/rotary_encoder_angle"};
    double yaw_zero_deg{0.0};
  };

  struct AngleModel
  {
    bool use_relative_angles{true};
    double boom_rel_body_zero_deg{0.0};
    double stick_rel_boom_zero_deg{0.0};
    double bucket_rel_stick_zero_deg{0.0};
    double boom_pitch_sign{1.0};
    double stick_pitch_sign{1.0};
    double bucket_pitch_sign{1.0};
  };

  struct InputQos
  {
    bool tilt_reliable{false};
    bool encoder_reliable{false};
  };

  void declareAndLoadParams();
  void publishStaticTfsOnce();
  void onTimer();

  void onTiltBoom(const cannode::msg::TiltRPY & msg);
  void onTiltStick(const cannode::msg::TiltRPY & msg);
  void onTiltBucket(const cannode::msg::TiltRPY & msg);
  void onTiltBody(const cannode::msg::TiltRPY & msg);
  void onRotaryEncoder(const cannode::msg::RotaryEncoderAngle & msg);

  double getAxisDeg(const cannode::msg::TiltRPY & msg, const std::string & axis) const;

  tf2::Transform computeBaseToUpper(double swing_yaw_rad) const;
  tf2::Transform computeUpperToBoom(double boom_pitch_rad) const;
  tf2::Transform computeBoomToStick(double stick_pitch_rad) const;
  tf2::Transform computeStickToBucket(double bucket_pitch_rad) const;
  tf2::Transform computeBucketToBucketOpen() const;

  std::vector<tf2::Vector3> build100GroundPointsInUpper(const tf2::Transform & upper_T_bucket_open) const;
  bool projectCameraPointToPixel(const tf2::Vector3 & cam_p, float & u, float & v) const;

  static tf2::Quaternion quatFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg);
  static geometry_msgs::msg::TransformStamped toMsg(
    const rclcpp::Time & stamp, const std::string & parent, const std::string & child,
    const tf2::Transform & T);

  Frames frames_;
  Grid grid_;
  PointGridConfig pg_;
  Kinematics kin_;
  std::vector<StaticTf> static_tfs_;

  TiltTopics tilt_topics_;
  TiltAxes tilt_axes_;
  TiltZerosDeg tilt_zeros_deg_;
  RotaryEncoderConfig rotary_;
  AngleModel angle_model_;
  InputQos input_qos_;

  // UI-stable projected center (pixel space), used when pg_.stabilize_center is enabled.
  bool have_center_uv_{false};
  double last_center_u_{0.0};
  double last_center_v_{0.0};

  // Latest sensor data (degrees). Guarded by mtx_.
  std::mutex mtx_;
  std::optional<cannode::msg::TiltRPY> last_tilt_boom_;
  std::optional<cannode::msg::TiltRPY> last_tilt_stick_;
  std::optional<cannode::msg::TiltRPY> last_tilt_bucket_;
  std::optional<cannode::msg::TiltRPY> last_tilt_body_;
  std::optional<cannode::msg::RotaryEncoderAngle> last_encoder_;
  bool sim_time_ready_{false};

  bool publish_static_tfs_{true};
  bool publish_dynamic_tfs_{true};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_pub_;

  rclcpp::Publisher<cyber_msgs::msg::PointGrid>::SharedPtr point_grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr sub_tilt_boom_;
  rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr sub_tilt_stick_;
  rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr sub_tilt_bucket_;
  rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr sub_tilt_body_;
  rclcpp::Subscription<cannode::msg::RotaryEncoderAngle>::SharedPtr sub_encoder_;
};

}  // namespace lidar_perception

