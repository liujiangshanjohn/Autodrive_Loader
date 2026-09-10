/**
 * 订阅 /rslidar_points，在雷达坐标系下按单个 AABB（roi: x/y/z min/max）过滤点云，
 * 沿指定轴压缩为 2D 后按 grid_step_m 网格降采样；输出格点数不设上限，仅由 ROI 与步长决定。
 *
 * cyber_msgs/PointGrid：projection_mode=2d 时 points[i].x、points[i].y 为投影平面上的两维（米），
 *   points[i].z 恒为 0。例如侧视 X–Z 需 compress_axis=y，则 .x=雷达 x，.y=雷达 z。
 * 可选 sensor_msgs/PointCloud2（output_pointcloud2_topic 非空）：同一批格点写入标准 xyz，
 *   侧视 X–Z 时为 (x, 0, z)，便于 RViz 用 PointCloud2 显示竖直面轮廓。
 * 发布后坐标可配置：output_scale、output_translate_*，满足 out = scale * in + translate；
 *   swap_output_xy 为 true 时再交换输出 x、y（等价于投影平面内旋转 90° 的常用修正）。
 *
 * projection_mode=3d：每格保留沿 compress_axis 取 max/min 的代表点完整 (x,y,z)。
 * publish_rate_hz>0：仅按该频率发布（默认 20 Hz，对齐 /cyber_msgs/point_grid）；订阅侧始终缓存最新点云。
 * 性能：unordered_map 网格、input_point_stride 抽稀、ROI Marker 降频、双回调组 + 双线程 executor，
 *   降低整帧遍历 + 红黑树 map 带来的 CPU 尖峰（作业时 RViz 点云卡顿常见诱因之一）。
 */
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <cyber_msgs/msg/point_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace lidar_perception
{

struct AxisBox {
  double min_x{-std::numeric_limits<double>::infinity()};
  double max_x{std::numeric_limits<double>::infinity()};
  double min_y{-std::numeric_limits<double>::infinity()};
  double max_y{std::numeric_limits<double>::infinity()};
  double min_z{-std::numeric_limits<double>::infinity()};
  double max_z{std::numeric_limits<double>::infinity()};
  bool enabled{false};

  bool contains(double x, double y, double z) const
  {
    if (!enabled) {
      return false;
    }
    return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
  }
};

static bool load_box(
  rclcpp::Node * node, const std::string & prefix, AxisBox * out)
{
  double mn_x = 0.0, mx_x = 0.0, mn_y = 0.0, mx_y = 0.0, mn_z = 0.0, mx_z = 0.0;
  node->declare_parameter(prefix + ".x_min", mn_x);
  node->declare_parameter(prefix + ".x_max", mx_x);
  node->declare_parameter(prefix + ".y_min", mn_y);
  node->declare_parameter(prefix + ".y_max", mx_y);
  node->declare_parameter(prefix + ".z_min", mn_z);
  node->declare_parameter(prefix + ".z_max", mx_z);
  node->get_parameter(prefix + ".x_min", mn_x);
  node->get_parameter(prefix + ".x_max", mx_x);
  node->get_parameter(prefix + ".y_min", mn_y);
  node->get_parameter(prefix + ".y_max", mx_y);
  node->get_parameter(prefix + ".z_min", mn_z);
  node->get_parameter(prefix + ".z_max", mx_z);
  out->min_x = mn_x;
  out->max_x = mx_x;
  out->min_y = mn_y;
  out->max_y = mx_y;
  out->min_z = mn_z;
  out->max_z = mx_z;
  out->enabled = (mn_x < mx_x) && (mn_y < mx_y) && (mn_z < mx_z);
  return true;
}

/** 网格 (gi,gj) 打包为 64bit，供 unordered_map 使用（比 std::map 快一个数量级） */
static inline uint64_t pack_grid_ij(int32_t gi, int32_t gj)
{
  return (static_cast<uint64_t>(static_cast<uint32_t>(gi)) << 32) |
    static_cast<uint32_t>(gj);
}

class LidarArmBucket2dGrid : public rclcpp::Node
{
public:
  explicit LidarArmBucket2dGrid(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lidar_arm_bucket_2d_grid", options)
  {
    declare_parameter<std::string>("input_topic", "/rslidar_points");
    declare_parameter<std::string>("output_topic", "/cyber_msgs/lidar_2d_points");
    declare_parameter<std::string>("compress_axis", "z");
    declare_parameter<std::string>("projection_mode", "2d");
    declare_parameter<std::string>("projection_3d_metric", "max");
    declare_parameter<double>("grid_step_m", 0.2);
    declare_parameter<bool>("publish_roi_marker", false);
    declare_parameter<std::string>("roi_marker_topic", "lidar_arm_bucket/roi");
    declare_parameter<std::string>("output_pointcloud2_topic", "");
    declare_parameter<double>("output_scale", 1.0);
    declare_parameter<double>("output_translate_x", 0.0);
    declare_parameter<double>("output_translate_y", 0.0);
    declare_parameter<double>("output_translate_z", 0.0);
    declare_parameter<bool>("swap_output_xy", false);
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<int>("input_point_stride", 1);
    declare_parameter<int>("roi_marker_decimation", 5);

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("compress_axis", compress_axis_);
    get_parameter("projection_mode", projection_mode_);
    get_parameter("projection_3d_metric", projection_3d_metric_);
    get_parameter("grid_step_m", grid_step_m_);
    get_parameter("publish_roi_marker", publish_roi_marker_);
    get_parameter("roi_marker_topic", roi_marker_topic_);
    get_parameter("output_pointcloud2_topic", output_pointcloud2_topic_);
    get_parameter("output_scale", output_scale_);
    get_parameter("output_translate_x", output_tx_);
    get_parameter("output_translate_y", output_ty_);
    get_parameter("output_translate_z", output_tz_);
    get_parameter("swap_output_xy", swap_output_xy_);
    get_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("input_point_stride", input_point_stride_);
    get_parameter("roi_marker_decimation", roi_marker_decimation_);

    input_point_stride_ = std::max(1, input_point_stride_);
    roi_marker_decimation_ = std::max(1, roi_marker_decimation_);

    if (!(output_scale_ > 0.0) || !std::isfinite(output_scale_)) {
      RCLCPP_WARN(
        get_logger(),
        "output_scale=%.6g 非法，改用 1.0",
        output_scale_);
      output_scale_ = 1.0;
    }
    if (!std::isfinite(output_tx_)) {
      output_tx_ = 0.0;
    }
    if (!std::isfinite(output_ty_)) {
      output_ty_ = 0.0;
    }
    if (!std::isfinite(output_tz_)) {
      output_tz_ = 0.0;
    }

    if (grid_step_m_ <= 0.0) {
      grid_step_m_ = 0.2;
    }

    {
      std::string a = compress_axis_;
      for (auto & c : a) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      if (a != "x" && a != "y" && a != "z") {
        RCLCPP_WARN(
          get_logger(),
          "compress_axis=%s 非法，改用 z（丢弃 Z，保留 X-Y）",
          compress_axis_.c_str());
        compress_axis_ = "z";
      } else {
        compress_axis_ = a;
      }
    }

    {
      std::string m = projection_mode_;
      for (auto & c : m) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      if (m != "2d" && m != "3d") {
        RCLCPP_WARN(
          get_logger(),
          "projection_mode=%s 非法，改用 2d",
          projection_mode_.c_str());
        projection_mode_ = "2d";
      } else {
        projection_mode_ = m;
      }
    }
    {
      std::string met = projection_3d_metric_;
      for (auto & c : met) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      if (met != "max" && met != "min") {
        RCLCPP_WARN(
          get_logger(),
          "projection_3d_metric=%s 非法，3d 模式下改用 max",
          projection_3d_metric_.c_str());
        projection_3d_metric_ = "max";
      } else {
        projection_3d_metric_ = met;
      }
    }

    load_box(this, "roi", &roi_);

    if (!roi_.enabled) {
      RCLCPP_WARN(
        get_logger(),
        "roi 未启用（需 roi.x_min<x_max 且 y、z 同理）。仍将订阅但不对点云做 ROI 过滤。");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(2));
    qos.best_effort();

    cb_group_timer_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_sub_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    pub_ = create_publisher<cyber_msgs::msg::PointGrid>(output_topic_, qos);

    if (!output_pointcloud2_topic_.empty()) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_pointcloud2_topic_, qos);
    }

    if (publish_roi_marker_) {
      auto mq = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable();
      marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(roi_marker_topic_, mq);
    }

    const bool timed_publish =
      std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0;
    if (timed_publish) {
      int period_ms = static_cast<int>(
        std::llround(1000.0 / publish_rate_hz_));
      period_ms = std::max(1, period_ms);
      publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]() {
          sensor_msgs::msg::PointCloud2::SharedPtr cloud;
          {
            std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
            cloud = latest_cloud_;
          }
          if (cloud && cloud->width > 0) {
            processCloud(cloud);
          }
        },
        cb_group_timer_);
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_group_sub_;
      sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          if (!msg || msg->width == 0) {
            return;
          }
          std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
          latest_cloud_ = std::move(msg);
        },
        sub_opts);
      RCLCPP_INFO(
        get_logger(),
        "publish_rate=%.3f Hz (%d ms)，与 bucket_point_grid_publisher /cyber_msgs/point_grid 定时一致时可设 20 Hz",
        publish_rate_hz_, period_ms);
    } else {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = cb_group_sub_;
      sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          processCloud(std::move(msg));
        },
        sub_opts);
      RCLCPP_INFO(get_logger(), "publish_rate_hz<=0：输出跟随每一帧输入点云");
    }

    RCLCPP_INFO(
      get_logger(),
      "input=%s output=%s compress_axis=%s projection_mode=%s 3d_metric=%s step=%.3f m "
      "roi_marker=%s out_scale=%.6g translate=(%.6g,%.6g,%.6g) swap_xy=%s",
      input_topic_.c_str(), output_topic_.c_str(), compress_axis_.c_str(),
      projection_mode_.c_str(), projection_3d_metric_.c_str(),
      grid_step_m_,
      publish_roi_marker_ ? roi_marker_topic_.c_str() : "off",
      output_scale_, output_tx_, output_ty_, output_tz_,
      swap_output_xy_ ? "on" : "off");
    RCLCPP_INFO(
      get_logger(),
      "perf: input_point_stride=%d roi_marker_decimation=%d（降低 CPU 与 DDS 抖动）",
      input_point_stride_, roi_marker_decimation_);
    if (!output_pointcloud2_topic_.empty()) {
      RCLCPP_INFO(
        get_logger(), "PointCloud2(xyz 投影平面) -> %s",
        output_pointcloud2_topic_.c_str());
    }
  }

private:
  struct GridCell2d {
    float cu{0.0f};
    float cv{0.0f};
  };

  /** 去掉 compress_axis 对应维度，剩余两维写入 u,v */
  static void project2d(
    const std::string & axis, double x, double y, double z,
    float * u, float * v)
  {
    if (axis == "z") {
      *u = static_cast<float>(x);
      *v = static_cast<float>(y);
    } else if (axis == "y") {
      *u = static_cast<float>(x);
      *v = static_cast<float>(z);
    } else {
      *u = static_cast<float>(y);
      *v = static_cast<float>(z);
    }
  }

  /** compress_axis 对应的坐标（被「投影」掉的维度，用于 3d 模式下选 max/min 代表点） */
  static double compressCoord(const std::string & axis, double x, double y, double z)
  {
    if (axis == "z") {
      return z;
    }
    if (axis == "y") {
      return y;
    }
    return x;
  }

  /** 对发布点：先 out = scale * in + translate，再按需交换输出 x/y（PointGrid 与 PointCloud2 共用） */
  void applyOutputTransform(float * x, float * y, float * z) const
  {
    *x = static_cast<float>(*x * output_scale_ + output_tx_);
    *y = static_cast<float>(*y * output_scale_ + output_ty_);
    *z = static_cast<float>(*z * output_scale_ + output_tz_);
    if (swap_output_xy_) {
      std::swap(*x, *y);
    }
  }

  static void planeGridToLidarXYZ(
    const std::string & axis, float u, float v,
    float * lx, float * ly, float * lz)
  {
    if (axis == "z") {
      *lx = u;
      *ly = v;
      *lz = 0.0f;
    } else if (axis == "y") {
      *lx = u;
      *ly = 0.0f;
      *lz = v;
    } else {
      *lx = 0.0f;
      *ly = u;
      *lz = v;
    }
  }

  void publishPlanePointCloud2(
    const std_msgs::msg::Header & header,
    const std::unordered_map<uint64_t, GridCell2d> & cells)
  {
    if (!cloud_pub_ || cells.empty()) {
      return;
    }
    const size_t n_pts = cells.size();
    sensor_msgs::msg::PointCloud2 out;
    out.header = header;
    out.height = 1;
    out.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n_pts);

    sensor_msgs::PointCloud2Iterator<float> ix(out, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(out, "z");
    size_t n = 0;
    for (const auto & kv : cells) {
      float lx = 0.0f;
      float ly = 0.0f;
      float lz = 0.0f;
      planeGridToLidarXYZ(
        compress_axis_, kv.second.cu, kv.second.cv, &lx, &ly, &lz);
      applyOutputTransform(&lx, &ly, &lz);
      *ix = lx;
      *iy = ly;
      *iz = lz;
      ++ix;
      ++iy;
      ++iz;
      ++n;
      if (n >= n_pts) {
        break;
      }
    }
    out.width = static_cast<uint32_t>(n);
    modifier.resize(n);
    cloud_pub_->publish(out);
  }

  /** 在雷达系下发布 ROI 轴对齐包围盒，供 RViz Marker 叠加点云调参 */
  void publishRoiMarker(const std_msgs::msg::Header & header)
  {
    if (!marker_pub_) {
      return;
    }
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "lidar_arm_bucket_roi";
    m.id = 0;
    if (!roi_.enabled) {
      m.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(m);
      return;
    }
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = (roi_.min_x + roi_.max_x) * 0.5;
    m.pose.position.y = (roi_.min_y + roi_.max_y) * 0.5;
    m.pose.position.z = (roi_.min_z + roi_.max_z) * 0.5;
    m.scale.x = std::max(1e-6, roi_.max_x - roi_.min_x);
    m.scale.y = std::max(1e-6, roi_.max_y - roi_.min_y);
    m.scale.z = std::max(1e-6, roi_.max_z - roi_.min_z);
    m.color.r = 0.15f;
    m.color.g = 0.85f;
    m.color.b = 0.25f;
    m.color.a = 0.35f;
    marker_pub_->publish(m);
  }

  void processCloud(sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (!cloud || cloud->width == 0) {
      return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

    struct Cell3d {
      double best_w{0.0};
      float rx{0.0f};
      float ry{0.0f};
      float rz{0.0f};
    };
    std::unordered_map<uint64_t, GridCell2d> cells2d;
    std::unordered_map<uint64_t, Cell3d> cells3d;
    const size_t n_total = cloud->width * cloud->height;
    const size_t reserve_hint = std::min(
      n_total / static_cast<size_t>(std::max(1, input_point_stride_)) + 128u,
      static_cast<size_t>(1u << 20));
    cells2d.reserve(reserve_hint);
    cells3d.reserve(reserve_hint);

    const float inv_step = static_cast<float>(1.0 / grid_step_m_);
    const bool use3d = (projection_mode_ == "3d");
    const bool take_max = (projection_3d_metric_ == "max");
    size_t idx = 0;
    while (idx < n_total) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      const double z = static_cast<double>(*iter_z);
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        if (!roi_.enabled || roi_.contains(x, y, z)) {
          float u = 0.0f;
          float v = 0.0f;
          project2d(compress_axis_, x, y, z, &u, &v);

          const int32_t gi = static_cast<int32_t>(std::floor(u * inv_step));
          const int32_t gj = static_cast<int32_t>(std::floor(v * inv_step));
          const uint64_t key = pack_grid_ij(gi, gj);

          if (use3d) {
            const double w = compressCoord(compress_axis_, x, y, z);
            const float fx = static_cast<float>(x);
            const float fy = static_cast<float>(y);
            const float fz = static_cast<float>(z);
            auto ins = cells3d.try_emplace(key, Cell3d{w, fx, fy, fz});
            if (!ins.second) {
              Cell3d & c = ins.first->second;
              const bool better =
                take_max ? (w > c.best_w) : (w < c.best_w);
              if (better) {
                c.best_w = w;
                c.rx = fx;
                c.ry = fy;
                c.rz = fz;
              }
            }
          } else {
            const float cu =
              (static_cast<float>(gi) + 0.5f) * static_cast<float>(grid_step_m_);
            const float cv =
              (static_cast<float>(gj) + 0.5f) * static_cast<float>(grid_step_m_);
            cells2d.try_emplace(key, GridCell2d{cu, cv});
          }
        }
      }

      const size_t step =
        std::min(static_cast<size_t>(input_point_stride_), n_total - idx);
      for (size_t k = 1; k < step; ++k) {
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
      idx += step;
    }

    cyber_msgs::msg::PointGrid out;
    if (use3d) {
      out.points.reserve(cells3d.size());
      for (const auto & kv : cells3d) {
        geometry_msgs::msg::Point32 p;
        p.x = kv.second.rx;
        p.y = kv.second.ry;
        p.z = kv.second.rz;
        applyOutputTransform(&p.x, &p.y, &p.z);
        out.points.push_back(p);
      }
    } else {
      out.points.reserve(cells2d.size());
      for (const auto & kv : cells2d) {
        geometry_msgs::msg::Point32 p;
        p.x = kv.second.cu;
        p.y = kv.second.cv;
        p.z = 0.0f;
        applyOutputTransform(&p.x, &p.y, &p.z);
        out.points.push_back(p);
      }
    }

    pub_->publish(out);

    if (!use3d && cloud_pub_) {
      publishPlanePointCloud2(cloud->header, cells2d);
    }

    if (publish_roi_marker_) {
      ++roi_marker_tick_;
      if (roi_marker_tick_ % roi_marker_decimation_ == 0) {
        publishRoiMarker(cloud->header);
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string compress_axis_{"z"};
  std::string projection_mode_{"2d"};
  std::string projection_3d_metric_{"max"};
  double grid_step_m_{0.2};
  bool publish_roi_marker_{false};
  std::string roi_marker_topic_;
  std::string output_pointcloud2_topic_;
  double output_scale_{1.0};
  double output_tx_{0.0};
  double output_ty_{0.0};
  double output_tz_{0.0};
  bool swap_output_xy_{false};
  double publish_rate_hz_{20.0};
  int input_point_stride_{1};
  int roi_marker_decimation_{1};
  uint32_t roi_marker_tick_{0};

  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_sub_;

  std::mutex latest_cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  AxisBox roi_;

  rclcpp::Publisher<cyber_msgs::msg::PointGrid>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace lidar_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<lidar_perception::LidarArmBucket2dGrid>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 2u);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
