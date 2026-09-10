#include "bucket_point_grid_publisher.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_perception
{

static constexpr double kDeg2Rad = M_PI / 180.0;

static inline std::array<std::pair<double, double>, 4> orderQuadByAngle(
  const std::array<std::pair<double, double>, 4> & q)
{
  double cx = 0.0, cy = 0.0;
  for (const auto & p : q) {
    cx += p.first;
    cy += p.second;
  }
  cx *= 0.25;
  cy *= 0.25;

  struct P
  {
    std::pair<double, double> uv;
    double ang;
  };
  std::array<P, 4> pts = {{
    {q[0], std::atan2(q[0].second - cy, q[0].first - cx)},
    {q[1], std::atan2(q[1].second - cy, q[1].first - cx)},
    {q[2], std::atan2(q[2].second - cy, q[2].first - cx)},
    {q[3], std::atan2(q[3].second - cy, q[3].first - cx)},
  }};
  std::sort(pts.begin(), pts.end(), [](const P & a, const P & b) { return a.ang < b.ang; });

  std::array<std::pair<double, double>, 4> out = {pts[0].uv, pts[1].uv, pts[2].uv, pts[3].uv};

  // Rotate so that the first point is the top-left-ish corner (smallest x+y) for stability.
  int best = 0;
  double best_score = out[0].first + out[0].second;
  for (int i = 1; i < 4; ++i) {
    const double s = out[i].first + out[i].second;
    if (s < best_score) {
      best_score = s;
      best = i;
    }
  }
  if (best != 0) {
    std::array<std::pair<double, double>, 4> rot{};
    for (int i = 0; i < 4; ++i) {
      rot[i] = out[(best + i) % 4];
    }
    out = rot;
  }
  return out;
}

static inline double cameraForwardFromPoint(
  const std::string & forward_axis,
  double forward_sign,
  const tf2::Vector3 & cam_p)
{
  // Mirror the mapping in projectCameraPointToPixel().
  double fwd = cam_p.z();
  if (forward_axis == "x") {
    fwd = cam_p.x();
  }
  const double s = (forward_sign >= 0.0) ? 1.0 : -1.0;
  fwd *= s;
  return fwd;
}

static inline bool pointInConvexQuad(
  const std::array<std::pair<double, double>, 4> & q, double x, double y)
{
  // Works for both CW and CCW quads; assumes convex.
  auto cross = [](double ax, double ay, double bx, double by, double cx, double cy) -> double {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  };
  double s0 = cross(q[0].first, q[0].second, q[1].first, q[1].second, x, y);
  double s1 = cross(q[1].first, q[1].second, q[2].first, q[2].second, x, y);
  double s2 = cross(q[2].first, q[2].second, q[3].first, q[3].second, x, y);
  double s3 = cross(q[3].first, q[3].second, q[0].first, q[0].second, x, y);
  const bool has_neg = (s0 < 0) || (s1 < 0) || (s2 < 0) || (s3 < 0);
  const bool has_pos = (s0 > 0) || (s1 > 0) || (s2 > 0) || (s3 > 0);
  return !(has_neg && has_pos);
}

BucketPointGridPublisher::BucketPointGridPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("bucket_point_grid_publisher", options)
{
  declareAndLoadParams();

  if (publish_static_tfs_) {
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  }
  if (publish_dynamic_tfs_) {
    dynamic_tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  point_grid_pub_ = create_publisher<cyber_msgs::msg::PointGrid>("/cyber_msgs/point_grid", rclcpp::QoS(10));

  // Subscriptions (sensor -> dynamic joint angles)
  auto tilt_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  tilt_qos.reliability(input_qos_.tilt_reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                                : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto encoder_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  encoder_qos.reliability(input_qos_.encoder_reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                                      : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  sub_tilt_boom_ = create_subscription<cannode::msg::TiltRPY>(
    tilt_topics_.boom, tilt_qos,
    [this](const cannode::msg::TiltRPY::SharedPtr msg) { onTiltBoom(*msg); });
  sub_tilt_stick_ = create_subscription<cannode::msg::TiltRPY>(
    tilt_topics_.stick, tilt_qos,
    [this](const cannode::msg::TiltRPY::SharedPtr msg) { onTiltStick(*msg); });
  sub_tilt_bucket_ = create_subscription<cannode::msg::TiltRPY>(
    tilt_topics_.bucket, tilt_qos,
    [this](const cannode::msg::TiltRPY::SharedPtr msg) { onTiltBucket(*msg); });
  sub_tilt_body_ = create_subscription<cannode::msg::TiltRPY>(
    tilt_topics_.body, tilt_qos,
    [this](const cannode::msg::TiltRPY::SharedPtr msg) { onTiltBody(*msg); });
  sub_encoder_ = create_subscription<cannode::msg::RotaryEncoderAngle>(
    rotary_.topic, encoder_qos,
    [this](const cannode::msg::RotaryEncoderAngle::SharedPtr msg) { onRotaryEncoder(*msg); });

  if (publish_static_tfs_) {
    publishStaticTfsOnce();
  }
  timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&BucketPointGridPublisher::onTimer, this));
}

void BucketPointGridPublisher::declareAndLoadParams()
{
  declare_parameter("publish_static_tfs", publish_static_tfs_);
  declare_parameter("publish_dynamic_tfs", publish_dynamic_tfs_);
  get_parameter("publish_static_tfs", publish_static_tfs_);
  get_parameter("publish_dynamic_tfs", publish_dynamic_tfs_);

  declare_parameter("frames.base_link", frames_.base_link);
  declare_parameter("frames.upper_link", frames_.upper_link);
  declare_parameter("frames.boom_link", frames_.boom_link);
  declare_parameter("frames.stick_link", frames_.stick_link);
  declare_parameter("frames.bucket_link", frames_.bucket_link);
  declare_parameter("frames.bucket_open", frames_.bucket_open);
  get_parameter("frames.base_link", frames_.base_link);
  get_parameter("frames.upper_link", frames_.upper_link);
  get_parameter("frames.boom_link", frames_.boom_link);
  get_parameter("frames.stick_link", frames_.stick_link);
  get_parameter("frames.bucket_link", frames_.bucket_link);
  get_parameter("frames.bucket_open", frames_.bucket_open);

  // tilt topics / axes / zeros
  declare_parameter("tilt_topics.boom", tilt_topics_.boom);
  declare_parameter("tilt_topics.stick", tilt_topics_.stick);
  declare_parameter("tilt_topics.bucket", tilt_topics_.bucket);
  declare_parameter("tilt_topics.body", tilt_topics_.body);
  get_parameter("tilt_topics.boom", tilt_topics_.boom);
  get_parameter("tilt_topics.stick", tilt_topics_.stick);
  get_parameter("tilt_topics.bucket", tilt_topics_.bucket);
  get_parameter("tilt_topics.body", tilt_topics_.body);

  declare_parameter("tilt_axes.boom", tilt_axes_.boom);
  declare_parameter("tilt_axes.stick", tilt_axes_.stick);
  declare_parameter("tilt_axes.bucket", tilt_axes_.bucket);
  declare_parameter("tilt_axes.body", tilt_axes_.body);
  get_parameter("tilt_axes.boom", tilt_axes_.boom);
  get_parameter("tilt_axes.stick", tilt_axes_.stick);
  get_parameter("tilt_axes.bucket", tilt_axes_.bucket);
  get_parameter("tilt_axes.body", tilt_axes_.body);

  declare_parameter("tilt_zeros_deg.boom", tilt_zeros_deg_.boom);
  declare_parameter("tilt_zeros_deg.stick", tilt_zeros_deg_.stick);
  declare_parameter("tilt_zeros_deg.bucket", tilt_zeros_deg_.bucket);
  declare_parameter("tilt_zeros_deg.body_roll", tilt_zeros_deg_.body_roll);
  declare_parameter("tilt_zeros_deg.body_pitch", tilt_zeros_deg_.body_pitch);
  declare_parameter("tilt_zeros_deg.body_yaw", tilt_zeros_deg_.body_yaw);
  get_parameter("tilt_zeros_deg.boom", tilt_zeros_deg_.boom);
  get_parameter("tilt_zeros_deg.stick", tilt_zeros_deg_.stick);
  get_parameter("tilt_zeros_deg.bucket", tilt_zeros_deg_.bucket);
  get_parameter("tilt_zeros_deg.body_roll", tilt_zeros_deg_.body_roll);
  get_parameter("tilt_zeros_deg.body_pitch", tilt_zeros_deg_.body_pitch);
  get_parameter("tilt_zeros_deg.body_yaw", tilt_zeros_deg_.body_yaw);

  declare_parameter("rotary_encoder.topic", rotary_.topic);
  declare_parameter("rotary_encoder.yaw_zero_deg", rotary_.yaw_zero_deg);
  get_parameter("rotary_encoder.topic", rotary_.topic);
  get_parameter("rotary_encoder.yaw_zero_deg", rotary_.yaw_zero_deg);

  // input qos (bag 回放通常是 BEST_EFFORT)
  declare_parameter("input_qos.tilt_reliable", input_qos_.tilt_reliable);
  declare_parameter("input_qos.encoder_reliable", input_qos_.encoder_reliable);
  get_parameter("input_qos.tilt_reliable", input_qos_.tilt_reliable);
  get_parameter("input_qos.encoder_reliable", input_qos_.encoder_reliable);

  declare_parameter("bucket_open_tracker.grid_resolution_m", grid_.resolution_m);
  declare_parameter("bucket_open_tracker.grid_width_m", grid_.width_m);
  declare_parameter("bucket_open_tracker.grid_length_m", grid_.length_m);
  get_parameter("bucket_open_tracker.grid_resolution_m", grid_.resolution_m);
  get_parameter("bucket_open_tracker.grid_width_m", grid_.width_m);
  get_parameter("bucket_open_tracker.grid_length_m", grid_.length_m);

  declare_parameter("bucket_open_tracker.point_grid.camera_frame", pg_.camera_frame);
  declare_parameter("bucket_open_tracker.point_grid.fx", pg_.fx);
  declare_parameter("bucket_open_tracker.point_grid.fy", pg_.fy);
  declare_parameter("bucket_open_tracker.point_grid.cx", pg_.cx);
  declare_parameter("bucket_open_tracker.point_grid.cy", pg_.cy);
  declare_parameter("bucket_open_tracker.point_grid.u_offset_px", pg_.u_offset_px);
  declare_parameter("bucket_open_tracker.point_grid.v_offset_px", pg_.v_offset_px);
  declare_parameter("bucket_open_tracker.point_grid.use_ground_quad", pg_.use_ground_quad);
  declare_parameter("bucket_open_tracker.point_grid.ground_quad_length_m", pg_.ground_quad_length_m);
  declare_parameter("bucket_open_tracker.point_grid.ground_quad_width_m", pg_.ground_quad_width_m);
  declare_parameter("bucket_open_tracker.point_grid.ground_quad_align_upper_yaw", pg_.ground_quad_align_upper_yaw);
  declare_parameter("bucket_open_tracker.point_grid.distance_normalize", pg_.distance_normalize);
  declare_parameter("bucket_open_tracker.point_grid.normalize_ref_fwd_m", pg_.normalize_ref_fwd_m);
  declare_parameter("bucket_open_tracker.point_grid.normalize_min_scale", pg_.normalize_min_scale);
  declare_parameter("bucket_open_tracker.point_grid.normalize_max_scale", pg_.normalize_max_scale);
  declare_parameter("bucket_open_tracker.point_grid.ground_quad_dotted", pg_.ground_quad_dotted);
  declare_parameter("bucket_open_tracker.point_grid.ground_quad_dotted_n", pg_.ground_quad_dotted_n);
  declare_parameter("bucket_open_tracker.point_grid.use_pixel_quad", pg_.use_pixel_quad);
  declare_parameter("bucket_open_tracker.point_grid.pixel_quad_width_px", pg_.pixel_quad_width_px);
  declare_parameter("bucket_open_tracker.point_grid.pixel_quad_height_px", pg_.pixel_quad_height_px);
  declare_parameter("bucket_open_tracker.point_grid.pixel_quad_dotted_n", pg_.pixel_quad_dotted_n);
  declare_parameter("bucket_open_tracker.point_grid.stabilize_center", pg_.stabilize_center);
  declare_parameter("bucket_open_tracker.point_grid.center_lpf_alpha", pg_.center_lpf_alpha);
  declare_parameter("bucket_open_tracker.point_grid.center_hold_on_invalid", pg_.center_hold_on_invalid);
  declare_parameter("bucket_open_tracker.point_grid.center_margin_px", pg_.center_margin_px);
  declare_parameter("bucket_open_tracker.point_grid.use_pixel_grid", pg_.use_pixel_grid);
  declare_parameter("bucket_open_tracker.point_grid.pixel_grid_n", pg_.pixel_grid_n);
  declare_parameter("bucket_open_tracker.point_grid.pixel_grid_spacing_px", pg_.pixel_grid_spacing_px);
  declare_parameter("bucket_open_tracker.point_grid.img_width", pg_.img_width);
  declare_parameter("bucket_open_tracker.point_grid.img_height", pg_.img_height);
  declare_parameter("bucket_open_tracker.point_grid.pixel_origin", pg_.pixel_origin);
  declare_parameter("bucket_open_tracker.point_grid.forward_axis", pg_.forward_axis);
  declare_parameter("bucket_open_tracker.point_grid.forward_sign", pg_.forward_sign);
  declare_parameter("bucket_open_tracker.point_grid.down_sign", pg_.down_sign);
  declare_parameter("bucket_open_tracker.point_grid.clamp_to_image", pg_.clamp_to_image);
  declare_parameter("bucket_open_tracker.point_grid.keep_point_count", pg_.keep_point_count);
  declare_parameter("bucket_open_tracker.point_grid.debug_force_fixed_uv", pg_.debug_force_fixed_uv);
  declare_parameter("bucket_open_tracker.point_grid.debug_u", pg_.debug_u);
  declare_parameter("bucket_open_tracker.point_grid.debug_v", pg_.debug_v);
  declare_parameter("bucket_open_tracker.point_grid.debug_span_px", pg_.debug_span_px);
  get_parameter("bucket_open_tracker.point_grid.camera_frame", pg_.camera_frame);
  get_parameter("bucket_open_tracker.point_grid.fx", pg_.fx);
  get_parameter("bucket_open_tracker.point_grid.fy", pg_.fy);
  get_parameter("bucket_open_tracker.point_grid.cx", pg_.cx);
  get_parameter("bucket_open_tracker.point_grid.cy", pg_.cy);
  get_parameter("bucket_open_tracker.point_grid.u_offset_px", pg_.u_offset_px);
  get_parameter("bucket_open_tracker.point_grid.v_offset_px", pg_.v_offset_px);
  get_parameter("bucket_open_tracker.point_grid.use_ground_quad", pg_.use_ground_quad);
  get_parameter("bucket_open_tracker.point_grid.ground_quad_length_m", pg_.ground_quad_length_m);
  get_parameter("bucket_open_tracker.point_grid.ground_quad_width_m", pg_.ground_quad_width_m);
  get_parameter("bucket_open_tracker.point_grid.ground_quad_align_upper_yaw", pg_.ground_quad_align_upper_yaw);
  get_parameter("bucket_open_tracker.point_grid.distance_normalize", pg_.distance_normalize);
  get_parameter("bucket_open_tracker.point_grid.normalize_ref_fwd_m", pg_.normalize_ref_fwd_m);
  get_parameter("bucket_open_tracker.point_grid.normalize_min_scale", pg_.normalize_min_scale);
  get_parameter("bucket_open_tracker.point_grid.normalize_max_scale", pg_.normalize_max_scale);
  get_parameter("bucket_open_tracker.point_grid.ground_quad_dotted", pg_.ground_quad_dotted);
  get_parameter("bucket_open_tracker.point_grid.ground_quad_dotted_n", pg_.ground_quad_dotted_n);
  get_parameter("bucket_open_tracker.point_grid.use_pixel_quad", pg_.use_pixel_quad);
  get_parameter("bucket_open_tracker.point_grid.pixel_quad_width_px", pg_.pixel_quad_width_px);
  get_parameter("bucket_open_tracker.point_grid.pixel_quad_height_px", pg_.pixel_quad_height_px);
  get_parameter("bucket_open_tracker.point_grid.pixel_quad_dotted_n", pg_.pixel_quad_dotted_n);
  get_parameter("bucket_open_tracker.point_grid.stabilize_center", pg_.stabilize_center);
  get_parameter("bucket_open_tracker.point_grid.center_lpf_alpha", pg_.center_lpf_alpha);
  get_parameter("bucket_open_tracker.point_grid.center_hold_on_invalid", pg_.center_hold_on_invalid);
  get_parameter("bucket_open_tracker.point_grid.center_margin_px", pg_.center_margin_px);
  get_parameter("bucket_open_tracker.point_grid.use_pixel_grid", pg_.use_pixel_grid);
  get_parameter("bucket_open_tracker.point_grid.pixel_grid_n", pg_.pixel_grid_n);
  get_parameter("bucket_open_tracker.point_grid.pixel_grid_spacing_px", pg_.pixel_grid_spacing_px);
  get_parameter("bucket_open_tracker.point_grid.img_width", pg_.img_width);
  get_parameter("bucket_open_tracker.point_grid.img_height", pg_.img_height);
  get_parameter("bucket_open_tracker.point_grid.pixel_origin", pg_.pixel_origin);
  get_parameter("bucket_open_tracker.point_grid.forward_axis", pg_.forward_axis);
  get_parameter("bucket_open_tracker.point_grid.forward_sign", pg_.forward_sign);
  get_parameter("bucket_open_tracker.point_grid.down_sign", pg_.down_sign);
  get_parameter("bucket_open_tracker.point_grid.clamp_to_image", pg_.clamp_to_image);
  get_parameter("bucket_open_tracker.point_grid.keep_point_count", pg_.keep_point_count);
  get_parameter("bucket_open_tracker.point_grid.debug_force_fixed_uv", pg_.debug_force_fixed_uv);
  get_parameter("bucket_open_tracker.point_grid.debug_u", pg_.debug_u);
  get_parameter("bucket_open_tracker.point_grid.debug_v", pg_.debug_v);
  get_parameter("bucket_open_tracker.point_grid.debug_span_px", pg_.debug_span_px);

  // angle model
  declare_parameter("angle_model.use_relative_angles", angle_model_.use_relative_angles);
  declare_parameter("angle_model.boom_rel_body_zero_deg", angle_model_.boom_rel_body_zero_deg);
  declare_parameter("angle_model.stick_rel_boom_zero_deg", angle_model_.stick_rel_boom_zero_deg);
  declare_parameter("angle_model.bucket_rel_stick_zero_deg", angle_model_.bucket_rel_stick_zero_deg);
  declare_parameter("angle_model.boom_pitch_sign", angle_model_.boom_pitch_sign);
  declare_parameter("angle_model.stick_pitch_sign", angle_model_.stick_pitch_sign);
  declare_parameter("angle_model.bucket_pitch_sign", angle_model_.bucket_pitch_sign);
  get_parameter("angle_model.use_relative_angles", angle_model_.use_relative_angles);
  get_parameter("angle_model.boom_rel_body_zero_deg", angle_model_.boom_rel_body_zero_deg);
  get_parameter("angle_model.stick_rel_boom_zero_deg", angle_model_.stick_rel_boom_zero_deg);
  get_parameter("angle_model.bucket_rel_stick_zero_deg", angle_model_.bucket_rel_stick_zero_deg);
  get_parameter("angle_model.boom_pitch_sign", angle_model_.boom_pitch_sign);
  get_parameter("angle_model.stick_pitch_sign", angle_model_.stick_pitch_sign);
  get_parameter("angle_model.bucket_pitch_sign", angle_model_.bucket_pitch_sign);

  declare_parameter(
    "kinematics.swing_center_xyz_m",
    std::vector<double>(kin_.swing_center_xyz_m.begin(), kin_.swing_center_xyz_m.end()));
  declare_parameter(
    "kinematics.boom_joint_xyz_m",
    std::vector<double>(kin_.boom_joint_xyz_m.begin(), kin_.boom_joint_xyz_m.end()));
  declare_parameter("kinematics.boom_length_m", kin_.boom_length_m);
  declare_parameter("kinematics.stick_length_m", kin_.stick_length_m);
  declare_parameter(
    "kinematics.bucket_open_xyz_m",
    std::vector<double>(kin_.bucket_open_xyz_m.begin(), kin_.bucket_open_xyz_m.end()));

  std::vector<double> swing_center, boom_joint, bucket_open;
  get_parameter("kinematics.swing_center_xyz_m", swing_center);
  get_parameter("kinematics.boom_joint_xyz_m", boom_joint);
  get_parameter("kinematics.boom_length_m", kin_.boom_length_m);
  get_parameter("kinematics.stick_length_m", kin_.stick_length_m);
  get_parameter("kinematics.bucket_open_xyz_m", bucket_open);
  if (swing_center.size() == 3) kin_.swing_center_xyz_m = {swing_center[0], swing_center[1], swing_center[2]};
  if (boom_joint.size() == 3) kin_.boom_joint_xyz_m = {boom_joint[0], boom_joint[1], boom_joint[2]};
  if (bucket_open.size() == 3) kin_.bucket_open_xyz_m = {bucket_open[0], bucket_open[1], bucket_open[2]};

  for (int i = 0; i < 8; ++i) {
    const std::string p_parent = "static_tfs." + std::to_string(i) + ".parent";
    const std::string p_child = "static_tfs." + std::to_string(i) + ".child";
    const std::string p_xyz = "static_tfs." + std::to_string(i) + ".xyz_m";
    const std::string p_rpy = "static_tfs." + std::to_string(i) + ".rpy_deg";

    declare_parameter(p_parent, std::string{""});
    declare_parameter(p_child, std::string{""});
    declare_parameter(p_xyz, std::vector<double>{});
    declare_parameter(p_rpy, std::vector<double>{});

    std::string parent, child;
    std::vector<double> xyz, rpy;
    get_parameter(p_parent, parent);
    get_parameter(p_child, child);
    get_parameter(p_xyz, xyz);
    get_parameter(p_rpy, rpy);
    if (parent.empty() || child.empty() || xyz.size() != 3 || rpy.size() != 3) continue;
    static_tfs_.push_back(StaticTf{parent, child, {xyz[0], xyz[1], xyz[2]}, {rpy[0], rpy[1], rpy[2]}});
  }
}

void BucketPointGridPublisher::onTiltBoom(const cannode::msg::TiltRPY & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_tilt_boom_ = msg;
}
void BucketPointGridPublisher::onTiltStick(const cannode::msg::TiltRPY & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_tilt_stick_ = msg;
}
void BucketPointGridPublisher::onTiltBucket(const cannode::msg::TiltRPY & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_tilt_bucket_ = msg;
}
void BucketPointGridPublisher::onTiltBody(const cannode::msg::TiltRPY & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_tilt_body_ = msg;
}
void BucketPointGridPublisher::onRotaryEncoder(const cannode::msg::RotaryEncoderAngle & msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_encoder_ = msg;
}

double BucketPointGridPublisher::getAxisDeg(const cannode::msg::TiltRPY & msg, const std::string & axis) const
{
  if (axis == "roll") return static_cast<double>(msg.roll);
  if (axis == "pitch") return static_cast<double>(msg.pitch);
  if (axis == "yaw") return static_cast<double>(msg.yaw);
  return static_cast<double>(msg.pitch);
}

void BucketPointGridPublisher::publishStaticTfsOnce()
{
  if (!static_tf_pub_) {
    return;
  }
  const auto stamp = now();
  std::vector<geometry_msgs::msg::TransformStamped> msgs;
  msgs.reserve(static_tfs_.size());
  for (const auto & s : static_tfs_) {
    tf2::Transform T;
    T.setOrigin(tf2::Vector3{s.xyz_m[0], s.xyz_m[1], s.xyz_m[2]});
    T.setRotation(quatFromRpyDeg(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]));
    msgs.push_back(toMsg(stamp, s.parent, s.child, T));
  }
  if (!msgs.empty()) static_tf_pub_->sendTransform(msgs);
}

tf2::Transform BucketPointGridPublisher::computeBaseToUpper(double swing_yaw_rad) const
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3{kin_.swing_center_xyz_m[0], kin_.swing_center_xyz_m[1], kin_.swing_center_xyz_m[2]});
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, swing_yaw_rad);
  T.setRotation(q);
  return T;
}

tf2::Transform BucketPointGridPublisher::computeUpperToBoom(double boom_pitch_rad) const
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3{kin_.boom_joint_xyz_m[0], kin_.boom_joint_xyz_m[1], kin_.boom_joint_xyz_m[2]});
  tf2::Quaternion q;
  q.setRPY(0.0, boom_pitch_rad, 0.0);
  T.setRotation(q);
  return T;
}

tf2::Transform BucketPointGridPublisher::computeBoomToStick(double stick_pitch_rad) const
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3{kin_.boom_length_m, 0.0, 0.0});
  tf2::Quaternion q;
  q.setRPY(0.0, stick_pitch_rad, 0.0);
  T.setRotation(q);
  return T;
}

tf2::Transform BucketPointGridPublisher::computeStickToBucket(double bucket_pitch_rad) const
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3{kin_.stick_length_m, 0.0, 0.0});
  tf2::Quaternion q;
  q.setRPY(0.0, bucket_pitch_rad, 0.0);
  T.setRotation(q);
  return T;
}

tf2::Transform BucketPointGridPublisher::computeBucketToBucketOpen() const
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3{kin_.bucket_open_xyz_m[0], kin_.bucket_open_xyz_m[1], kin_.bucket_open_xyz_m[2]});
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  T.setRotation(q);
  return T;
}

std::vector<tf2::Vector3> BucketPointGridPublisher::build100GroundPointsInUpper(
  const tf2::Transform & upper_T_bucket_open) const
{
  const tf2::Vector3 p = upper_T_bucket_open.getOrigin();
  const double ground_z_upper = -kin_.swing_center_xyz_m[2];
  const tf2::Vector3 center{p.x(), p.y(), ground_z_upper};

  constexpr int N = 10;
  const double span_x = grid_.length_m;
  const double span_y = grid_.width_m;
  const double step_x = (N > 1) ? (span_x / (N - 1)) : span_x;
  const double step_y = (N > 1) ? (span_y / (N - 1)) : span_y;

  std::vector<tf2::Vector3> pts;
  pts.reserve(N * N);
  for (int iy = 0; iy < N; ++iy) {
    for (int ix = 0; ix < N; ++ix) {
      const double dx = (ix * step_x) - (span_x * 0.5);
      const double dy = (iy * step_y) - (span_y * 0.5);
      // 不在 base_link 平面强行 clamp，避免靠边时大量点被压成同一个值导致“看起来一条线”
      pts.emplace_back(center.x() + dx, center.y() + dy, ground_z_upper);
    }
  }
  return pts;
}

bool BucketPointGridPublisher::projectCameraPointToPixel(const tf2::Vector3 & cam_p, float & u, float & v) const
{
  const double x = cam_p.x();
  const double y = cam_p.y();
  const double z = cam_p.z();

  // Two common camera frames exist in ROS ecosystems; we make it selectable.
  double fwd = z;
  double right = x;
  double down = y;
  if (pg_.forward_axis == "x") {
    // Treat +X as forward, +Y as left, +Z as up.
    // Map to pinhole (right, down, forward):
    // right = -Y, down = -Z, forward = +X
    fwd = x;
    right = -y;
    down = -z;
  }

  const double s = (pg_.forward_sign >= 0.0) ? 1.0 : -1.0;
  fwd *= s;
  right *= s;
  down *= s;

  down *= (pg_.down_sign >= 0.0) ? 1.0 : -1.0;

  if (!(fwd > 1e-6)) return false;

  double uu = pg_.fx * (right / fwd) + pg_.cx;
  double vv = pg_.fy * (down / fwd) + pg_.cy;

  if (pg_.pixel_origin == "bottom_right") {
    uu = (static_cast<double>(pg_.img_width) - 1.0) - uu;
    vv = (static_cast<double>(pg_.img_height) - 1.0) - vv;
  }

  // Post-projection translation in pixel space.
  uu += pg_.u_offset_px;
  vv += pg_.v_offset_px;

  if (pg_.clamp_to_image) {
    uu = std::min(std::max(uu, 0.0), static_cast<double>(pg_.img_width - 1));
    vv = std::min(std::max(vv, 0.0), static_cast<double>(pg_.img_height - 1));
  }

  u = static_cast<float>(uu);
  v = static_cast<float>(vv);
  return true;
}

void BucketPointGridPublisher::onTimer()
{
  const auto stamp = now();
  // When use_sim_time=true, ROS time stays at 0 until /clock arrives.
  // Publishing TF with stamp=0 and then switching to /clock often triggers RViz TF_OLD_DATA drops.
  if (get_clock()->get_clock_type() == RCL_ROS_TIME) {
    if (stamp.nanoseconds() == 0) {
      if (!sim_time_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "等待 /clock（use_sim_time=true），暂不发布 TF/点阵");
      }
      return;
    }
    sim_time_ready_ = true;
  }

  // Read latest sensors (degrees)
  std::optional<cannode::msg::TiltRPY> boom, stick, bucket, body;
  std::optional<cannode::msg::RotaryEncoderAngle> enc;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    boom = last_tilt_boom_;
    stick = last_tilt_stick_;
    bucket = last_tilt_bucket_;
    body = last_tilt_body_;
    enc = last_encoder_;
  }

  // Default to zeros if not available (but warn throttled)
  double body_pitch_deg = 0.0;
  double boom_abs_deg = 0.0;
  double stick_abs_deg = 0.0;
  double bucket_abs_deg = 0.0;
  double swing_yaw_deg = 0.0;

  if (body) {
    body_pitch_deg = getAxisDeg(*body, "pitch") - tilt_zeros_deg_.body_pitch;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "未收到 body 倾角话题：%s", tilt_topics_.body.c_str());
  }
  if (boom) {
    boom_abs_deg = getAxisDeg(*boom, tilt_axes_.boom) - tilt_zeros_deg_.boom;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "未收到 boom 倾角话题：%s", tilt_topics_.boom.c_str());
  }
  if (stick) {
    stick_abs_deg = getAxisDeg(*stick, tilt_axes_.stick) - tilt_zeros_deg_.stick;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "未收到 stick 倾角话题：%s", tilt_topics_.stick.c_str());
  }
  if (bucket) {
    bucket_abs_deg = getAxisDeg(*bucket, tilt_axes_.bucket) - tilt_zeros_deg_.bucket;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "未收到 bucket 倾角话题：%s", tilt_topics_.bucket.c_str());
  }

  if (enc) {
    swing_yaw_deg = static_cast<double>(enc->angle1) - rotary_.yaw_zero_deg;
  } else if (body) {
    // fallback: use body yaw if encoder missing
    swing_yaw_deg = getAxisDeg(*body, tilt_axes_.body) - tilt_zeros_deg_.body_yaw;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "未收到回转编码器话题：%s", rotary_.topic.c_str());
  }

  // Joint angles (degrees)
  double boom_pitch_deg = 0.0;
  double stick_pitch_deg = 0.0;
  double bucket_pitch_deg = 0.0;
  if (angle_model_.use_relative_angles) {
    boom_pitch_deg =
      angle_model_.boom_pitch_sign *
      ((boom_abs_deg - body_pitch_deg) - angle_model_.boom_rel_body_zero_deg);
    stick_pitch_deg =
      angle_model_.stick_pitch_sign *
      ((stick_abs_deg - boom_abs_deg) - angle_model_.stick_rel_boom_zero_deg);
    bucket_pitch_deg =
      angle_model_.bucket_pitch_sign *
      ((bucket_abs_deg - stick_abs_deg) - angle_model_.bucket_rel_stick_zero_deg);
  } else {
    boom_pitch_deg = angle_model_.boom_pitch_sign * boom_abs_deg;
    stick_pitch_deg = angle_model_.stick_pitch_sign * stick_abs_deg;
    bucket_pitch_deg = angle_model_.bucket_pitch_sign * bucket_abs_deg;
  }

  const double swing_yaw = swing_yaw_deg * kDeg2Rad;
  const double boom_pitch = boom_pitch_deg * kDeg2Rad;
  const double stick_pitch = stick_pitch_deg * kDeg2Rad;
  const double bucket_pitch = bucket_pitch_deg * kDeg2Rad;

  const tf2::Transform base_T_upper = computeBaseToUpper(swing_yaw);
  const tf2::Transform upper_T_boom = computeUpperToBoom(boom_pitch);
  const tf2::Transform boom_T_stick = computeBoomToStick(stick_pitch);
  const tf2::Transform stick_T_bucket = computeStickToBucket(bucket_pitch);
  const tf2::Transform bucket_T_open = computeBucketToBucketOpen();

  const tf2::Transform base_T_boom = base_T_upper * upper_T_boom;
  const tf2::Transform base_T_stick = base_T_boom * boom_T_stick;
  const tf2::Transform base_T_bucket = base_T_stick * stick_T_bucket;
  const tf2::Transform base_T_open = base_T_bucket * bucket_T_open;

  std::vector<geometry_msgs::msg::TransformStamped> dyn;
  dyn.reserve(5);
  dyn.push_back(toMsg(stamp, frames_.base_link, frames_.upper_link, base_T_upper));
  dyn.push_back(toMsg(stamp, frames_.upper_link, frames_.boom_link, upper_T_boom));
  dyn.push_back(toMsg(stamp, frames_.boom_link, frames_.stick_link, boom_T_stick));
  dyn.push_back(toMsg(stamp, frames_.stick_link, frames_.bucket_link, stick_T_bucket));
  dyn.push_back(toMsg(stamp, frames_.bucket_link, frames_.bucket_open, bucket_T_open));
  if (dynamic_tf_pub_) {
    dynamic_tf_pub_->sendTransform(dyn);
  }

  cyber_msgs::msg::PointGrid out;
  out.points.reserve(100);

  if (pg_.debug_force_fixed_uv) {
    constexpr int N = 10;
    const double step = (N > 1) ? (static_cast<double>(pg_.debug_span_px) / (N - 1)) : 0.0;
    for (int iy = 0; iy < N; ++iy) {
      for (int ix = 0; ix < N; ++ix) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(pg_.debug_u + (ix * step - pg_.debug_span_px * 0.5));
        p.y = static_cast<float>(pg_.debug_v + (iy * step - pg_.debug_span_px * 0.5));
        p.z = 0.0f;
        out.points.push_back(p);
      }
    }
    point_grid_pub_->publish(out);
    return;
  }

  tf2::Transform upper_T_lidar;
  tf2::Transform lidar_T_cam;
  bool have_upper_lidar = false;
  bool have_lidar_cam = false;
  for (const auto & s : static_tfs_) {
    if (s.parent == frames_.upper_link && s.child == "rslidar") {
      upper_T_lidar.setOrigin(tf2::Vector3{s.xyz_m[0], s.xyz_m[1], s.xyz_m[2]});
      upper_T_lidar.setRotation(quatFromRpyDeg(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]));
      have_upper_lidar = true;
    }
    if (s.parent == "rslidar" && s.child == pg_.camera_frame) {
      lidar_T_cam.setOrigin(tf2::Vector3{s.xyz_m[0], s.xyz_m[1], s.xyz_m[2]});
      lidar_T_cam.setRotation(quatFromRpyDeg(s.rpy_deg[0], s.rpy_deg[1], s.rpy_deg[2]));
      have_lidar_cam = true;
    }
  }
  if (!have_upper_lidar || !have_lidar_cam) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "缺少静态外参：upper_link->rslidar 或 rslidar->%s，无法投影。",
      pg_.camera_frame.c_str());
    return;
  }

  // Transforms:
  // base -> upper (dynamic), upper -> lidar (static), lidar -> cam (static)
  const tf2::Transform upper_T_open = base_T_upper.inverse() * base_T_open;
  const tf2::Transform upper_T_cam = upper_T_lidar * lidar_T_cam;
  const tf2::Transform base_T_cam = base_T_upper * upper_T_cam;
  const tf2::Transform cam_T_upper = upper_T_cam.inverse();
  const tf2::Transform cam_T_base = base_T_cam.inverse();

  // Ground plane:
  // - In base_link frame we treat ground as z=0.
  // - In upper_link frame we approximate ground as z = -swing_center_z.
  const double ground_z_upper = -kin_.swing_center_xyz_m[2];

  if (pg_.use_ground_quad) {
    double L = std::max(0.0, pg_.ground_quad_length_m);
    double W = std::max(0.0, pg_.ground_quad_width_m);

    // Use base_link ground projection for the landing grid (more physically meaningful).
    const tf2::Vector3 open_base = base_T_open.getOrigin();
    const tf2::Vector3 base_center_ground{open_base.x(), open_base.y(), 0.0};

    if (pg_.distance_normalize) {
      const tf2::Vector3 cam_center = cam_T_base * base_center_ground;
      const double fwd = cameraForwardFromPoint(pg_.forward_axis, pg_.forward_sign, cam_center);
      const double ref = std::max(1e-3, pg_.normalize_ref_fwd_m);
      double scale = fwd / ref;
      const double smin = std::max(0.0, pg_.normalize_min_scale);
      const double smax = std::max(smin, pg_.normalize_max_scale);
      scale = std::min(std::max(scale, smin), smax);
      L *= scale;
      W *= scale;
    }

    const double hx = 0.5 * L;
    const double hy = 0.5 * W;

    const double yaw = pg_.ground_quad_align_upper_yaw ? swing_yaw : 0.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    auto rot_xy = [&](double x, double y) -> std::pair<double, double> {
      return {c * x - s * y, s * x + c * y};
    };

    const std::array<std::pair<double, double>, 4> local_xy = {
      std::pair<double, double>{-hx, -hy},
      std::pair<double, double>{+hx, -hy},
      std::pair<double, double>{+hx, +hy},
      std::pair<double, double>{-hx, +hy},
    };

    std::array<tf2::Vector3, 4> base_corners{};
    for (size_t i = 0; i < 4; ++i) {
      auto [rx, ry] = rot_xy(local_xy[i].first, local_xy[i].second);
      base_corners[i] = tf2::Vector3{base_center_ground.x() + rx, base_center_ground.y() + ry, 0.0};
    }

    std::array<std::pair<double, double>, 4> quad_uv{};
    bool ok = true;
    for (size_t i = 0; i < 4; ++i) {
      const tf2::Vector3 cam_p = cam_T_base * base_corners[i];
      float u = 0.0f, v = 0.0f;
      if (!projectCameraPointToPixel(cam_p, u, v)) { ok = false; break; }
      quad_uv[i] = {static_cast<double>(u), static_cast<double>(v)};
    }
    if (!ok) {
      // keep empty; downstream overlay will TTL out
    } else if (!pg_.ground_quad_dotted) {
      quad_uv = orderQuadByAngle(quad_uv);
      out.points.reserve(4);
      for (size_t i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point32 pix;
        pix.x = static_cast<float>(quad_uv[i].first);
        pix.y = static_cast<float>(quad_uv[i].second);
        pix.z = 0.0f;
        out.points.push_back(pix);
      }
    } else {
      quad_uv = orderQuadByAngle(quad_uv);
      // Landing grid format:
      // publish 4 corners first, then a regular N×N grid of projected points on the ground rectangle (upper frame).
      const int N = std::max(2, pg_.ground_quad_dotted_n);
      out.points.reserve(static_cast<size_t>(4 + N * N));

      // 4 corners (for red border in cam_geac)
      for (size_t i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point32 pix;
        pix.x = static_cast<float>(quad_uv[i].first);
        pix.y = static_cast<float>(quad_uv[i].second);
        pix.z = 0.0f;
        out.points.push_back(pix);
      }

      // Ground grid in meters (base frame, rotated if enabled), then project each point.
      const double dx = (N > 1) ? (L / static_cast<double>(N - 1)) : 0.0;
      const double dy = (N > 1) ? (W / static_cast<double>(N - 1)) : 0.0;
      for (int iy = 0; iy < N; ++iy) {
        for (int ix = 0; ix < N; ++ix) {
          const double lx = -hx + dx * static_cast<double>(ix);
          const double ly = -hy + dy * static_cast<double>(iy);
          auto [rx, ry] = rot_xy(lx, ly);
          const tf2::Vector3 base_pt{base_center_ground.x() + rx, base_center_ground.y() + ry, 0.0};
          const tf2::Vector3 cam_p = cam_T_base * base_pt;
          float u = 0.0f, v = 0.0f;
          if (!projectCameraPointToPixel(cam_p, u, v)) {
            // Skip invalid points; keep ordering dense enough for drawing.
            continue;
          }
          geometry_msgs::msg::Point32 pix;
          pix.x = u;
          pix.y = v;
          pix.z = 0.0f;
          out.points.push_back(pix);
        }
      }
    }
  } else if (pg_.use_pixel_quad) {
    // Fixed pixel-size dotted rectangle around projected ground center (stable UI).
    const tf2::Vector3 p = upper_T_open.getOrigin();
    const tf2::Vector3 upper_center_ground{p.x(), p.y(), ground_z_upper};
    const tf2::Vector3 cam_center = cam_T_upper * upper_center_ground;
    float u0 = static_cast<float>(pg_.cx);
    float v0 = static_cast<float>(pg_.cy);
    const bool proj_ok = projectCameraPointToPixel(cam_center, u0, v0);

    auto clamp_center = [this](double uu, double vv) -> std::pair<double, double> {
      const double margin = std::max(0.0, pg_.center_margin_px);
      const double min_u = margin;
      const double max_u = std::max(min_u, static_cast<double>(pg_.img_width - 1) - margin);
      const double min_v = margin;
      const double max_v = std::max(min_v, static_cast<double>(pg_.img_height - 1) - margin);
      uu = std::min(std::max(uu, min_u), max_u);
      vv = std::min(std::max(vv, min_v), max_v);
      return {uu, vv};
    };

    if (pg_.stabilize_center) {
      const bool on_screen = (u0 >= 0.0f && v0 >= 0.0f &&
                              u0 <= static_cast<float>(pg_.img_width - 1) &&
                              v0 <= static_cast<float>(pg_.img_height - 1));
      const bool valid = proj_ok && on_screen;
      if (!valid && pg_.center_hold_on_invalid && have_center_uv_) {
        u0 = static_cast<float>(last_center_u_);
        v0 = static_cast<float>(last_center_v_);
      } else {
        auto [cu, cv] = clamp_center(static_cast<double>(u0), static_cast<double>(v0));
        const double alpha = std::min(std::max(pg_.center_lpf_alpha, 0.0), 1.0);
        if (!have_center_uv_) {
          last_center_u_ = cu;
          last_center_v_ = cv;
          have_center_uv_ = true;
        } else {
          last_center_u_ = (1.0 - alpha) * last_center_u_ + alpha * cu;
          last_center_v_ = (1.0 - alpha) * last_center_v_ + alpha * cv;
        }
        u0 = static_cast<float>(last_center_u_);
        v0 = static_cast<float>(last_center_v_);
      }
    }

    const double w = std::max(1.0, pg_.pixel_quad_width_px);
    const double h = std::max(1.0, pg_.pixel_quad_height_px);
    const double min_u = static_cast<double>(u0) - 0.5 * w;
    const double max_u = static_cast<double>(u0) + 0.5 * w;
    const double min_v = static_cast<double>(v0) - 0.5 * h;
    const double max_v = static_cast<double>(v0) + 0.5 * h;

    const int N = std::max(2, pg_.pixel_quad_dotted_n);
    const double du = (max_u - min_u) / static_cast<double>(N - 1);
    const double dv = (max_v - min_v) / static_cast<double>(N - 1);

    out.points.reserve(100);
    for (int iy = 0; iy < N; ++iy) {
      for (int ix = 0; ix < N; ++ix) {
        if (out.points.size() >= 100) break;
        geometry_msgs::msg::Point32 pix;
        pix.x = static_cast<float>(min_u + du * static_cast<double>(ix));
        pix.y = static_cast<float>(min_v + dv * static_cast<double>(iy));
        pix.z = 0.0f;
        out.points.push_back(pix);
      }
      if (out.points.size() >= 100) break;
    }
  } else if (pg_.use_pixel_grid) {
    // Fixed pixel-spacing grid around projected ground center.
    const tf2::Vector3 p = upper_T_open.getOrigin();
    const tf2::Vector3 upper_center_ground{p.x(), p.y(), ground_z_upper};
    const tf2::Vector3 cam_center = cam_T_upper * upper_center_ground;
    float u0 = static_cast<float>(pg_.cx);
    float v0 = static_cast<float>(pg_.cy);
    const bool proj_ok = projectCameraPointToPixel(cam_center, u0, v0);
    if (pg_.stabilize_center) {
      const bool on_screen = (u0 >= 0.0f && v0 >= 0.0f &&
                              u0 <= static_cast<float>(pg_.img_width - 1) &&
                              v0 <= static_cast<float>(pg_.img_height - 1));
      const bool valid = proj_ok && on_screen;
      if (!valid && pg_.center_hold_on_invalid && have_center_uv_) {
        u0 = static_cast<float>(last_center_u_);
        v0 = static_cast<float>(last_center_v_);
      } else {
        const double margin = std::max(0.0, pg_.center_margin_px);
        const double min_u = margin;
        const double max_u = std::max(min_u, static_cast<double>(pg_.img_width - 1) - margin);
        const double min_v = margin;
        const double max_v = std::max(min_v, static_cast<double>(pg_.img_height - 1) - margin);
        double cu = std::min(std::max(static_cast<double>(u0), min_u), max_u);
        double cv = std::min(std::max(static_cast<double>(v0), min_v), max_v);
        const double alpha = std::min(std::max(pg_.center_lpf_alpha, 0.0), 1.0);
        if (!have_center_uv_) {
          last_center_u_ = cu;
          last_center_v_ = cv;
          have_center_uv_ = true;
        } else {
          last_center_u_ = (1.0 - alpha) * last_center_u_ + alpha * cu;
          last_center_v_ = (1.0 - alpha) * last_center_v_ + alpha * cv;
        }
        u0 = static_cast<float>(last_center_u_);
        v0 = static_cast<float>(last_center_v_);
      }
    }

    const int N = std::max(1, pg_.pixel_grid_n);
    const double step = std::max(0.0, pg_.pixel_grid_spacing_px);
    const double half = 0.5 * static_cast<double>(N - 1);
    out.points.reserve(static_cast<size_t>(N) * static_cast<size_t>(N));
    for (int iy = 0; iy < N; ++iy) {
      for (int ix = 0; ix < N; ++ix) {
        geometry_msgs::msg::Point32 pix;
        pix.x = static_cast<float>(static_cast<double>(u0) + (static_cast<double>(ix) - half) * step);
        pix.y = static_cast<float>(static_cast<double>(v0) + (static_cast<double>(iy) - half) * step);
        pix.z = 0.0f;
        out.points.push_back(pix);
      }
    }
  } else {
    const auto upper_ground_pts = build100GroundPointsInUpper(upper_T_open);
    for (const auto & upper_p : upper_ground_pts) {
      const tf2::Vector3 cam_p = cam_T_upper * upper_p;
    float u = 0.0f, v = 0.0f;
    if (!projectCameraPointToPixel(cam_p, u, v)) continue;
    geometry_msgs::msg::Point32 pix;
    pix.x = u;
    pix.y = v;
    pix.z = 0.0f;
    out.points.push_back(pix);
    }
  }

  if (!pg_.keep_point_count) {
  while (out.points.size() < 100) {
    geometry_msgs::msg::Point32 p;
    p.x = 0.0f;
    p.y = 0.0f;
    p.z = 0.0f;
    out.points.push_back(p);
  }
  if (out.points.size() > 100) out.points.resize(100);
  }

  point_grid_pub_->publish(out);
}

tf2::Quaternion BucketPointGridPublisher::quatFromRpyDeg(double roll_deg, double pitch_deg, double yaw_deg)
{
  tf2::Quaternion q;
  q.setRPY(roll_deg * kDeg2Rad, pitch_deg * kDeg2Rad, yaw_deg * kDeg2Rad);
  q.normalize();
  return q;
}

geometry_msgs::msg::TransformStamped BucketPointGridPublisher::toMsg(
  const rclcpp::Time & stamp, const std::string & parent, const std::string & child,
  const tf2::Transform & T)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = T.getOrigin().x();
  msg.transform.translation.y = T.getOrigin().y();
  msg.transform.translation.z = T.getOrigin().z();
  msg.transform.rotation.x = T.getRotation().x();
  msg.transform.rotation.y = T.getRotation().y();
  msg.transform.rotation.z = T.getRotation().z();
  msg.transform.rotation.w = T.getRotation().w();
  return msg;
}

}  // namespace lidar_perception

