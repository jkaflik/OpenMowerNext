#include "docking_helper/charger_presence_charging_dock.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::docking_helper
{

ChargerPresenceChargingDock::ChargerPresenceChargingDock() : opennav_docking_core::ChargingDock()
{
}

void ChargerPresenceChargingDock::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                            const std::string& name, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::runtime_error("Unable to lock weak_ptr to LifecycleNode in configure()");
  }

  node_ = node;
  tf_buffer_ = tf_buffer;

  node_->declare_parameter(name + ".staging_x_offset", 1.0);
  node_->declare_parameter(name + ".staging_yaw_offset", 0.0);

  staging_x_offset_ = node_->get_parameter(name + ".staging_x_offset").as_double();
  staging_yaw_offset_ = node_->get_parameter(name + ".staging_yaw_offset").as_double();

  power_status_sub_ = node_->create_subscription<omros2_firmware_msgs::msg::PowerStatus>(
      "/power/status", 10,
      [this](const omros2_firmware_msgs::msg::PowerStatus::SharedPtr msg) { is_charging_ = msg->charger_present; });

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);

  RCLCPP_INFO(node_->get_logger(),
              "ChargerPresenceChargingDock configured with staging_x_offset: %f, staging_yaw_offset: %f",
              staging_x_offset_, staging_yaw_offset_);
}

void ChargerPresenceChargingDock::cleanup()
{
  // Clean up subscriptions
  power_status_sub_.reset();
}

void ChargerPresenceChargingDock::activate()
{
  // Nothing to activate
}

void ChargerPresenceChargingDock::deactivate()
{
  // Nothing to deactivate
}

geometry_msgs::msg::PoseStamped ChargerPresenceChargingDock::getStagingPose(const geometry_msgs::msg::Pose& pose,
                                                                            const std::string& frame)
{
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();

  staging_pose.pose = pose;

  tf2::Transform pose_tf;
  tf2::fromMsg(pose, pose_tf);

  tf2::Transform offset_tf;
  tf2::Quaternion q;
  q.setRPY(0, 0, staging_yaw_offset_);
  offset_tf.setRotation(q);
  offset_tf.setOrigin(tf2::Vector3(staging_x_offset_, 0.0, 0.0));

  // Apply the offset
  tf2::Transform staging_tf = pose_tf * offset_tf;

  // Convert back to pose
  tf2::toMsg(staging_tf, staging_pose.pose);

  staging_pose_pub_->publish(staging_pose);

  return staging_pose;
}

bool ChargerPresenceChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped& pose, std::string id)
{
  (void)id;
  // just publish to a topic
  dock_pose_pub_->publish(pose);
  return true;
}

bool ChargerPresenceChargingDock::isDocked()
{
  return is_charging_;
}

bool ChargerPresenceChargingDock::isCharging()
{
  return is_charging_;
}

bool ChargerPresenceChargingDock::disableCharging()
{
  // disableCharging is checked on undocking and
  // has to return true to indicate that the robot
  // is allowed to undock. We don't want to control
  // the charging state here.
  return true;
}

bool ChargerPresenceChargingDock::hasStoppedCharging()
{
  return !is_charging_;
}

}  // namespace open_mower_next::docking_helper

PLUGINLIB_EXPORT_CLASS(open_mower_next::docking_helper::ChargerPresenceChargingDock, opennav_docking_core::ChargingDock)
