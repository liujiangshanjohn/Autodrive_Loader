/*********************************************************************************************************************
 ROS2：Destination*Ros 自建 Node 仅 create_publisher，主进程从不 spin。
 部分环境下 discovery / executor 处理不充分，表现为 echo / 下游订阅异常。
 在 init 中注册 Node，NodeManager::init 结束后启动后台 executor。
*********************************************************************************************************************/
#pragma once

#ifdef ROS2_FOUND

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robosense
{
namespace lidar
{

struct Ros2PublisherSpinState {
  std::mutex mtx;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes;
  std::thread spin_thread;
  bool started{false};
};

inline Ros2PublisherSpinState & ros2PublisherSpinState()
{
  static Ros2PublisherSpinState st;
  return st;
}

inline void register_ros2_node_for_spin(std::shared_ptr<rclcpp::Node> node)
{
  if (!node) {
    return;
  }
  auto & st = ros2PublisherSpinState();
  std::lock_guard<std::mutex> lk(st.mtx);
  if (st.started) {
    return;
  }
  st.nodes.push_back(std::move(node));
}

inline void start_ros2_executor_spin_after_init()
{
  auto & st = ros2PublisherSpinState();
  std::vector<std::shared_ptr<rclcpp::Node>> nodes_copy;
  {
    std::lock_guard<std::mutex> lk(st.mtx);
    if (st.started) {
      return;
    }
    if (st.nodes.empty()) {
      st.started = true;
      return;
    }
    st.started = true;
    nodes_copy = st.nodes;
  }

  st.spin_thread = std::thread([nodes_copy]() {
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2u);
    for (const auto & n : nodes_copy) {
      exec.add_node(n);
    }
    exec.spin();
  });
  st.spin_thread.detach();
}

}  // namespace lidar
}  // namespace robosense

#endif  // ROS2_FOUND
