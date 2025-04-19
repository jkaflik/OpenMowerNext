#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/msg/docking_station.hpp"
#include "open_mower_next/srv/find_nearest_docking_station.hpp"
#include "open_mower_next/action/dock_robot_nearest.hpp"
#include "open_mower_next/action/dock_robot_to.hpp"

namespace open_mower_next::docking_helper
{
class DockingHelperNode : public rclcpp::Node
{
public:
  explicit DockingHelperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DockingHelperNode();

private:
  // Action server types
  using DockRobotNearestAction = open_mower_next::action::DockRobotNearest;
  using DockRobotNearestGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotNearestAction>;

  using DockRobotToAction = open_mower_next::action::DockRobotTo;
  using DockRobotToGoalHandle = rclcpp_action::ServerGoalHandle<DockRobotToAction>;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);
  std::shared_ptr<rclcpp::Subscription<open_mower_next::msg::Map>> map_sub_;
  std::vector<open_mower_next::msg::DockingStation> docking_stations_;

  std::shared_ptr<open_mower_next::msg::DockingStation> findNearestDockingStation();
  void findNearestDockingStationService(
      const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
      std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response);

  std::shared_ptr<geometry_msgs::msg::PoseStamped>
  dockingStationPose(const std::shared_ptr<open_mower_next::msg::DockingStation>& station);

  rclcpp::Service<open_mower_next::srv::FindNearestDockingStation>::SharedPtr find_nearest_docking_station_service_;

  rclcpp_action::Client<nav2_msgs::action::DockRobot>::SharedPtr dock_client_;

  rclcpp_action::Server<DockRobotNearestAction>::SharedPtr dock_robot_nearest_server_;
  rclcpp_action::Server<DockRobotToAction>::SharedPtr dock_robot_to_server_;

  rclcpp_action::GoalResponse handleDockRobotNearestGoal(const rclcpp_action::GoalUUID& uuid,
                                                         std::shared_ptr<const DockRobotNearestAction::Goal> goal);
  rclcpp_action::CancelResponse
  handleDockRobotNearestCancel(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);
  void handleDockRobotNearestAccepted(const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handleDockRobotToGoal(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const DockRobotToAction::Goal> goal);
  rclcpp_action::CancelResponse handleDockRobotToCancel(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);
  void handleDockRobotToAccepted(const std::shared_ptr<DockRobotToGoalHandle> goal_handle);

  std::shared_ptr<open_mower_next::msg::DockingStation> findDockingStationById(const std::string& id);
};

}  // namespace open_mower_next::docking_helper
