#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <opennav_docking_core/charging_dock.hpp>
#include <omros2_firmware_msgs/msg/power_status.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace open_mower_next::docking_helper
{

class ChargerPresenceChargingDock : public opennav_docking_core::ChargingDock
{
public:
  ChargerPresenceChargingDock();

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                 std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::PoseStamped getStagingPose(const geometry_msgs::msg::Pose& pose,
                                                 const std::string& frame) override;

  bool getRefinedPose(geometry_msgs::msg::PoseStamped& pose, std::string id) override;
  bool isDocked() override;
  bool isCharging() override;
  bool disableCharging() override;
  bool hasStoppedCharging() override;

private:
  rclcpp::Subscription<omros2_firmware_msgs::msg::PowerStatus>::SharedPtr power_status_sub_;
  bool is_charging_ = false;  // Initialize to prevent undefined behavior
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;

  double staging_x_offset_ = 0.5;    // Default offset in meters
  double staging_yaw_offset_ = 0.0;  // Default yaw offset in radians
};

}  // namespace open_mower_next::docking_helper
