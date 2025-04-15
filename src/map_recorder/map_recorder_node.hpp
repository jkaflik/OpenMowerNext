#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "open_mower_next/msg/docking_station.hpp"
#include "open_mower_next/msg/area.hpp"
#include "open_mower_next/srv/save_docking_station.hpp"
#include "open_mower_next/srv/save_area.hpp"
#include "open_mower_next/action/record_docking_station.hpp"
#include "open_mower_next/action/record_area_boundary.hpp"

namespace open_mower_next::map_recorder
{
class MapRecorderNode : public rclcpp::Node
{
public:
  explicit MapRecorderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MapRecorderNode();

private:
  // Action server types
  using RecordDockingStationAction = open_mower_next::action::RecordDockingStation;
  using RecordDockingStationGoalHandle = rclcpp_action::ServerGoalHandle<RecordDockingStationAction>;

  using RecordAreaBoundaryAction = open_mower_next::action::RecordAreaBoundary;
  using RecordAreaBoundaryGoalHandle = rclcpp_action::ServerGoalHandle<RecordAreaBoundaryAction>;

  // Action servers
  rclcpp_action::Server<RecordDockingStationAction>::SharedPtr record_docking_station_server_;
  rclcpp_action::Server<RecordAreaBoundaryAction>::SharedPtr record_area_boundary_server_;

  // Service servers
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_recording_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr add_boundary_point_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_area_recording_service_;  // New service

  // Clients for saving data
  rclcpp::Client<open_mower_next::srv::SaveDockingStation>::SharedPtr save_docking_station_client_;
  rclcpp::Client<open_mower_next::srv::SaveArea>::SharedPtr save_area_client_;

  // Subscriber for charging status
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charging_status_sub_;

  // TF listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  bool is_recording_area_;
  bool is_charging_detected_;
  bool auto_recording_mode_;
  double distance_threshold_;
  std::string current_area_name_;
  uint8_t current_area_type_;  // Changed from string to uint8_t
  geometry_msgs::msg::PoseStamped last_recorded_position_;
  std::vector<geometry_msgs::msg::Point> current_boundary_points_;

  // Current goal handles
  std::shared_ptr<RecordDockingStationGoalHandle> docking_goal_handle_;
  std::shared_ptr<RecordAreaBoundaryGoalHandle> area_boundary_goal_handle_;

  // Action server callbacks for RecordDockingStation
  rclcpp_action::GoalResponse handleDockingGoal(const rclcpp_action::GoalUUID& uuid,
                                                std::shared_ptr<const RecordDockingStationAction::Goal> goal);
  rclcpp_action::CancelResponse handleDockingCancel(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle);
  void handleDockingAccepted(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle);

  // Action server callbacks for RecordAreaBoundary
  rclcpp_action::GoalResponse handleAreaBoundaryGoal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const RecordAreaBoundaryAction::Goal> goal);
  rclcpp_action::CancelResponse
  handleAreaBoundaryCancel(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle);
  void handleAreaBoundaryAccepted(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle);

  // Service callbacks
  void handleSetRecordingMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handleAddBoundaryPoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleFinishAreaRecording(  // New handler
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Topic callbacks
  void chargingStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Helper methods
  bool validateAreaBoundary();
  geometry_msgs::msg::PoseStamped getRobotPose();
  double distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                              const geometry_msgs::msg::PoseStamped& pose2);
  bool checkPositionCovariance();
  std::string generateUniqueId();
};

}  // namespace open_mower_next::map_recorder
