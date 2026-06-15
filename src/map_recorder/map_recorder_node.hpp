#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <omros2_firmware_msgs/msg/power_status.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "open_mower_next/msg/docking_station.hpp"
#include "open_mower_next/msg/area.hpp"
#include "open_mower_next/srv/save_docking_station.hpp"
#include "open_mower_next/srv/save_area.hpp"
#include "open_mower_next/action/record_docking_station.hpp"
#include "open_mower_next/action/record_area_boundary.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"  // Added DriveOnHeading action

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

  // DriveOnHeading action client
  using DriveOnHeadingAction = nav2_msgs::action::DriveOnHeading;
  using DriveOnHeadingGoalHandle = rclcpp_action::ClientGoalHandle<DriveOnHeadingAction>;
  rclcpp_action::Client<DriveOnHeadingAction>::SharedPtr drive_on_heading_client_;

  // Docking parameters
  double docking_approach_distance_;
  double docking_approach_speed_;

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

  rclcpp::Subscription<omros2_firmware_msgs::msg::PowerStatus>::SharedPtr charging_status_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr boundary_polygon_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  bool is_recording_area_;
  bool is_charging_detected_;
  bool auto_recording_mode_;
  double distance_threshold_;
  std::string current_area_name_;
  uint8_t current_area_type_;
  geometry_msgs::msg::PoseStamped last_recorded_position_;
  std::vector<geometry_msgs::msg::Point> current_boundary_points_;

  std::shared_ptr<RecordDockingStationGoalHandle> docking_goal_handle_;
  std::shared_ptr<RecordAreaBoundaryGoalHandle> area_boundary_goal_handle_;

  rclcpp_action::GoalResponse handleDockingGoal(const rclcpp_action::GoalUUID& uuid,
                                                std::shared_ptr<const RecordDockingStationAction::Goal> goal);
  rclcpp_action::CancelResponse handleDockingCancel(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle);
  void handleDockingAccepted(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle);

  rclcpp_action::GoalResponse handleAreaBoundaryGoal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const RecordAreaBoundaryAction::Goal> goal);
  rclcpp_action::CancelResponse
  handleAreaBoundaryCancel(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle);
  void handleAreaBoundaryAccepted(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle);

  void handleSetRecordingMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void handleAddBoundaryPoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleFinishAreaRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void chargingStatusCallback(const omros2_firmware_msgs::msg::PowerStatus::SharedPtr msg);

  bool validateAreaBoundary();

  geometry_msgs::msg::PoseStamped getMowerPose()
  {
    return getTransformPose("mower");
  }

  /// @brief returns the docking station pose when the robot is charging
  /// @details we assume the docking station is in front of the robot's charging port
  /// when the robot is charging
  /// Robot's charging port is rotated 180 degrees from the docking station
  /// @return the docking station pose when the robot is charging
  /// @note the pose is in the map frame
  geometry_msgs::msg::PoseStamped getDockingStationPoseWhenCharging()
  {
    auto pose = getTransformPose("charging_port");

    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    tf2::Quaternion rotation;
    rotation.setRPY(0, 0, M_PI);
    q = q * rotation;

    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }

  geometry_msgs::msg::PoseStamped getTransformPose(std::string frame)
  {
    geometry_msgs::msg::PoseStamped pose;
    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero);

      pose.header = transform_stamped.header;
      pose.pose.position.x = transform_stamped.transform.translation.x;
      pose.pose.position.y = transform_stamped.transform.translation.y;
      pose.pose.position.z = transform_stamped.transform.translation.z;
      pose.pose.orientation = transform_stamped.transform.rotation;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(get_logger(), "Could not transform from %s to map: %s", frame.c_str(), ex.what());
      throw;
    }

    return pose;
  }

  double distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                              const geometry_msgs::msg::PoseStamped& pose2);
  bool checkPositionCovariance();
  std::string generateUniqueId();
};

}  // namespace open_mower_next::map_recorder
