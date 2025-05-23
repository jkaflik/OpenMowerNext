#pragma once
#include <rclcpp/node.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>
#include "open_mower_next/msg/map.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>

namespace open_mower_next::sim
{

class SimNode final : public rclcpp::Node
{
public:
  explicit SimNode(const rclcpp::NodeOptions& options);

  ~SimNode() override = default;

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr docking_station_pose_subscriber_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> docking_station_pose_;
  // Docking station pose offset parameters
  double docking_station_offset_x_;
  double docking_station_offset_y_;

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

  void mapCallback(msg::Map::SharedPtr msg);
  void dockingStationPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
  void chargerPresentSimulationCallback();
  void batteryStateSimulationCallback();

  bool isInDockingStation();
};
};  // namespace open_mower_next::sim
