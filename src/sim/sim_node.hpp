#pragma once

#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

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

  std_msgs::msg::Bool charger_present_msg_;
  sensor_msgs::msg::BatteryState battery_state_msg_;
  std_msgs::msg::Float32 charge_voltage_msg_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_present_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr charge_voltage_publisher_;

  rclcpp::TimerBase::SharedPtr charger_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  double battery_state_max_voltage_;
  double battery_state_min_voltage_;
  double battery_state_voltage_drop_per_second_;
  double battery_state_voltage_charge_per_second_;
  rclcpp::Time last_battery_voltage_update_;

  // TF2 buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void chargerPresentSimulationCallback();
  void batteryStateSimulationCallback();

  bool isInDockingStation();
};
};  // namespace open_mower_next::sim
