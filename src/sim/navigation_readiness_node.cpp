#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <open_mower_next/msg/map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

bool isFiniteOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & pose = msg.pose.pose;
  const auto & twist = msg.twist.twist;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && std::isfinite(twist.linear.x) &&
         std::isfinite(twist.linear.y) && std::isfinite(twist.angular.z);
}

std::string joinMissing(const std::set<std::string> & missing)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & topic : missing) {
    if (!first) {
      stream << ", ";
    }
    first = false;
    stream << topic;
  }
  return stream.str();
}

class NavigationReadinessNode final : public rclcpp::Node
{
public:
  NavigationReadinessNode()
  : Node("navigation_readiness_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    timeout_seconds_ = declare_parameter<double>("timeout_seconds", 90.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    required_topics_ = {
      "/diff_drive_base_controller/odom", "/gps/fix", "/gps/odom", "/imu/data_raw",
      "/fusion/odom", "/map_grid", "/mowing_map"};

    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

    diff_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_base_controller/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr) { markReady("/diff_drive_base_controller/odom"); });
    gps_fix_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr) { markReady("/gps/fix"); });
    gps_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/gps/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (isFiniteOdom(*msg)) {
          markReady("/gps/odom");
        }
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr) { markReady("/imu/data_raw"); });
    fusion_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fusion/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (isFiniteOdom(*msg)) {
          markReady("/fusion/odom");
        }
      });
    map_grid_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map_grid", map_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) { markReady("/map_grid"); });
    mowing_map_subscription_ = create_subscription<open_mower_next::msg::Map>(
      "/mowing_map", map_qos,
      [this](const open_mower_next::msg::Map::SharedPtr) { markReady("/mowing_map"); });

    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { checkReady(); });
    RCLCPP_INFO(get_logger(), "Waiting for navigation prerequisites before starting Nav2");
  }

  int exitCode() const { return exit_code_; }

private:
  void markReady(const std::string & name) { ready_topics_.insert(name); }

  bool hasMapToBaseLinkTf()
  {
    try {
      return tf_buffer_.canTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.01));
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  std::set<std::string> missingTopics() const
  {
    std::set<std::string> missing;
    for (const auto & topic : required_topics_) {
      if (!ready_topics_.count(topic)) {
        missing.insert(topic);
      }
    }
    return missing;
  }

  void checkReady()
  {
    const auto missing = missingTopics();
    const bool tf_ready = hasMapToBaseLinkTf();
    if (missing.empty() && tf_ready) {
      exit_code_ = 0;
      RCLCPP_INFO(get_logger(), "Navigation prerequisites ready; starting Nav2");
      rclcpp::shutdown();
      return;
    }

    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= timeout_seconds_) {
      exit_code_ = 1;
      RCLCPP_ERROR(
        get_logger(), "Timed out waiting for navigation prerequisites. Missing topics: [%s], tf %s -> %s: %s",
        joinMissing(missing).c_str(), map_frame_.c_str(), base_frame_.c_str(), tf_ready ? "ready" : "missing");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Navigation prerequisites pending. Missing topics: [%s], tf %s -> %s: %s",
      joinMissing(missing).c_str(), map_frame_.c_str(), base_frame_.c_str(), tf_ready ? "ready" : "missing");
  }

  std::chrono::steady_clock::time_point start_time_ = std::chrono::steady_clock::now();
  double timeout_seconds_ = 90.0;
  std::string map_frame_;
  std::string base_frame_;
  int exit_code_ = 1;

  std::set<std::string> required_topics_;
  std::set<std::string> ready_topics_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr diff_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fusion_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_grid_subscription_;
  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr mowing_map_subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationReadinessNode>();
  rclcpp::spin(node);
  const int exit_code = node->exitCode();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
