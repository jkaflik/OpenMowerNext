#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <omros2_firmware_msgs/msg/power_status.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace open_mower_next::sim
{

class SimNode final : public rclcpp::Node
{
public:
  explicit SimNode(const rclcpp::NodeOptions & options);

  ~SimNode() override = default;

private:
  geometry_msgs::msg::PoseStamped docking_station_contact_pose_;
  std::string docking_station_frame_;
  std::string charging_port_frame_;
  double docking_detection_tolerance_x_;
  double docking_detection_tolerance_y_;

  sensor_msgs::msg::BatteryState battery_state_msg_;
  omros2_firmware_msgs::msg::PowerStatus power_status_msg_;

  std::string gps_odom_frame_;
  std::string gps_child_frame_;
  double gps_speed_stddev_;
  bool has_gps_fix_ = false;
  builtin_interfaces::msg::Time last_gps_fix_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<omros2_firmware_msgs::msg::PowerStatus>::SharedPtr power_status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_odom_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_speed_vector_subscription_;

  rclcpp::TimerBase::SharedPtr charger_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  double battery_state_max_voltage_;
  double battery_state_min_voltage_;
  double battery_state_voltage_drop_per_second_;
  double battery_state_voltage_charge_per_second_;
  bool charger_present_ = false;
  float charge_voltage_ = 0.0f;
  rclcpp::Time last_battery_voltage_update_;

  // TF2 buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void chargerPresentSimulationCallback();
  void batteryStateSimulationCallback();
  void publishPowerStatus(const rclcpp::Time & stamp);
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void gpsSpeedVectorCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

  bool isInDockingStation();
};
};  // namespace open_mower_next::sim
