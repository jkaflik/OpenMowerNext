#include "map_recorder/map_recorder_node.hpp"
#include "map_recorder/utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace open_mower_next::map_recorder
{
MapRecorderNode::MapRecorderNode(const rclcpp::NodeOptions& options)
  : Node("map_recorder", options)
  , is_recording_area_(false)
  , is_charging_detected_(false)
  , auto_recording_mode_(true)
  , distance_threshold_(0.05)
{
  this->declare_parameter("docking_approach_distance", 1.0);
  this->declare_parameter("docking_approach_speed", 0.1);

  docking_approach_distance_ = this->get_parameter("docking_approach_distance").as_double();
  docking_approach_speed_ = this->get_parameter("docking_approach_speed").as_double();
  distance_threshold_ = this->declare_parameter("distance_threshold", 0.05);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  record_docking_station_server_ = rclcpp_action::create_server<RecordDockingStationAction>(
      this, "record_docking_station", std::bind(&MapRecorderNode::handleDockingGoal, this, _1, _2),
      std::bind(&MapRecorderNode::handleDockingCancel, this, _1),
      std::bind(&MapRecorderNode::handleDockingAccepted, this, _1));

  record_area_boundary_server_ = rclcpp_action::create_server<RecordAreaBoundaryAction>(
      this, "record_area_boundary", std::bind(&MapRecorderNode::handleAreaBoundaryGoal, this, _1, _2),
      std::bind(&MapRecorderNode::handleAreaBoundaryCancel, this, _1),
      std::bind(&MapRecorderNode::handleAreaBoundaryAccepted, this, _1));

  drive_on_heading_client_ = rclcpp_action::create_client<DriveOnHeadingAction>(this, "drive_on_heading");

  save_docking_station_client_ = create_client<open_mower_next::srv::SaveDockingStation>("save_docking_station");
  save_area_client_ = create_client<open_mower_next::srv::SaveArea>("save_area");

  charging_status_sub_ = create_subscription<omros2_firmware_msgs::msg::PowerStatus>(
      "/power/status", 10, std::bind(&MapRecorderNode::chargingStatusCallback, this, _1));

  boundary_polygon_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>("/map_recorder/record_boundaries_polygon", 10);

  RCLCPP_INFO(get_logger(), "Map Recorder node initialized");
}

MapRecorderNode::~MapRecorderNode()
{
}

rclcpp_action::GoalResponse MapRecorderNode::handleDockingGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RecordDockingStationAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request for docking station recording: %s", goal->name.c_str());

  if (goal->name.empty())
  {
    RCLCPP_ERROR(get_logger(), "Docking station name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if ((docking_goal_handle_ && docking_goal_handle_->is_active()) || is_recording_area_)
  {
    RCLCPP_ERROR(get_logger(), "Another recording is already in progress");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MapRecorderNode::handleDockingCancel(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking station recording");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MapRecorderNode::handleDockingAccepted(const std::shared_ptr<RecordDockingStationGoalHandle> goal_handle)
{
  is_charging_detected_ = false;
  docking_goal_handle_ = goal_handle;

  std::thread{ [this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RecordDockingStationAction::Feedback>();
    auto result = std::make_shared<RecordDockingStationAction::Result>();
    std::shared_ptr<DriveOnHeadingGoalHandle> drive_goal_handle;

    RCLCPP_INFO(get_logger(), "Recording docking station: %s", goal->name.c_str());

    feedback->status = RecordDockingStationAction::Feedback::STATUS_DRIVING;
    feedback->message = "Driving towards docking station...";
    goal_handle->publish_feedback(feedback);

    if (!drive_on_heading_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(get_logger(), "DriveOnHeading action server not available after waiting");
      result->code = RecordDockingStationAction::Result::CODE_SERVICE_UNAVAILABLE;
      result->message = "DriveOnHeading action server not available";
      goal_handle->abort(result);
      return;
    }

    auto drive_goal = DriveOnHeadingAction::Goal();
    drive_goal.target.x = docking_approach_distance_;
    drive_goal.speed = docking_approach_speed_;

    // Calculate time allowance based on distance and speed
    // Add a safety margin by multiplying by 1.5
    int time_allowance_sec = static_cast<int>(std::ceil((docking_approach_distance_ / docking_approach_speed_) * 1.5));
    drive_goal.time_allowance.sec = time_allowance_sec;
    drive_goal.time_allowance.nanosec = 0;

    RCLCPP_INFO(get_logger(), "Sending DriveOnHeading with target.x: %.2f, speed: %.2f, time_allowance: %d seconds",
                docking_approach_distance_, docking_approach_speed_, time_allowance_sec);

    auto drive_goal_handle_future = drive_on_heading_client_->async_send_goal(drive_goal);

    if (drive_goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send DriveOnHeading goal");
      result->code = RecordDockingStationAction::Result::CODE_DRIVE_FAILURE;
      result->message = "Failed to send DriveOnHeading goal";
      goal_handle->abort(result);
      return;
    }

    drive_goal_handle = drive_goal_handle_future.get();
    if (!drive_goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by DriveOnHeading server");
      result->code = RecordDockingStationAction::Result::CODE_DRIVE_FAILURE;
      result->message = "Goal was rejected by DriveOnHeading server";
      goal_handle->abort(result);
      return;
    }

    feedback->status = RecordDockingStationAction::Feedback::STATUS_WAITING_FOR_CHARGING;
    feedback->message = "Waiting for charging to be detected...";
    goal_handle->publish_feedback(feedback);

    auto start_time = this->now();
    while (rclcpp::ok() && !is_charging_detected_ && (this->now() - start_time) < rclcpp::Duration(60s))
    {
      if (goal_handle->is_canceling())
      {
        if (drive_goal_handle)
        {
          RCLCPP_INFO(get_logger(), "Canceling DriveOnHeading goal due to docking goal cancellation");
          drive_on_heading_client_->async_cancel_goal(drive_goal_handle);
        }

        result->code = RecordDockingStationAction::Result::CODE_CANCELED;
        result->message = "Docking station recording was canceled";
        goal_handle->canceled(result);
        return;
      }

      std::this_thread::sleep_for(100ms);
    }

    if (drive_goal_handle)
    {
      RCLCPP_INFO(get_logger(), "Canceling DriveOnHeading goal");
      drive_on_heading_client_->async_cancel_goal(drive_goal_handle);
    }

    if (!is_charging_detected_)
    {
      result->code = RecordDockingStationAction::Result::CODE_NO_CHARGING_DETECTED;
      result->message = "Timeout waiting for charging to be detected";
      goal_handle->abort(result);
      return;
    }

    feedback->status = RecordDockingStationAction::Feedback::STATUS_RECORDING;
    feedback->message = "Charging detected! Verifying position accuracy...";
    goal_handle->publish_feedback(feedback);

    if (!checkPositionCovariance())
    {
      result->code = RecordDockingStationAction::Result::CODE_POOR_LOCALIZATION;
      result->message = "Position covariance exceeds threshold (2cm)";
      goal_handle->abort(result);
      return;
    }

    feedback->status = RecordDockingStationAction::Feedback::STATUS_SAVING;
    feedback->message = "Position verified. Recording docking station...";
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::PoseStamped docking_station_pose;
    try
    {
      docking_station_pose = getDockingStationPoseWhenCharging();
    }
    catch (const std::exception& e)
    {
      result->code = RecordDockingStationAction::Result::CODE_UNKNOWN_ERROR;
      result->message = "Failed to get robot pose: " + std::string(e.what());
      goal_handle->abort(result);
      return;
    }

    open_mower_next::msg::DockingStation docking_station;
    docking_station.id = generateUniqueId();
    docking_station.name = goal->name;
    docking_station.pose.header = docking_station_pose.header;
    docking_station.pose.pose = docking_station_pose.pose;

    auto request = std::make_shared<open_mower_next::srv::SaveDockingStation::Request>();
    request->docking_station = docking_station;

    if (!save_docking_station_client_->wait_for_service(10s))
    {
      result->code = RecordDockingStationAction::Result::CODE_SERVICE_UNAVAILABLE;
      result->message = "Save docking station service not available";
      goal_handle->abort(result);
      return;
    }

    auto future = save_docking_station_client_->async_send_request(request);

    if (future.wait_for(5s) != std::future_status::ready)
    {
      result->code = RecordDockingStationAction::Result::CODE_SERVICE_FAILED;
      result->message = "Save docking station service timed out";
      goal_handle->abort(result);
      return;
    }

    auto service_response = future.get();
    if (service_response)
    {
      if (service_response->code == open_mower_next::srv::SaveDockingStation::Response::CODE_SUCCESS)
      {
        result->code = RecordDockingStationAction::Result::CODE_SUCCESS;
        result->message = service_response->message;
        result->docking_station = docking_station;

        RCLCPP_INFO(get_logger(), "Docking station recording completed successfully");
        goal_handle->succeed(result);
      }
      else
      {
        result->code = RecordDockingStationAction::Result::CODE_SERVICE_FAILED;
        result->message = service_response->message;
        RCLCPP_ERROR(get_logger(), "Failed to save docking station: %s", service_response->message.c_str());
        goal_handle->abort(result);
      }
    }
    else
    {
      result->code = RecordDockingStationAction::Result::CODE_SERVICE_FAILED;
      result->message = "Failed to save docking station: service call failed";
      goal_handle->abort(result);
    }
    return;
  } }.detach();
}

rclcpp_action::GoalResponse MapRecorderNode::handleAreaBoundaryGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RecordAreaBoundaryAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request for area boundary recording: %s", goal->name.c_str());

  if (goal->name.empty())
  {
    RCLCPP_ERROR(get_logger(), "Area name cannot be empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->type > 2)
  {
    RCLCPP_ERROR(get_logger(), "Invalid area type (must be 0, 1, or 2)");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (docking_goal_handle_ || is_recording_area_)
  {
    RCLCPP_ERROR(get_logger(), "Another recording is already in progress");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MapRecorderNode::handleAreaBoundaryCancel(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel area boundary recording");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MapRecorderNode::handleAreaBoundaryAccepted(const std::shared_ptr<RecordAreaBoundaryGoalHandle> goal_handle)
{
  current_boundary_points_.clear();
  is_recording_area_ = true;
  area_boundary_goal_handle_ = goal_handle;
  auto goal = goal_handle->get_goal();

  current_area_name_ = goal->name;
  current_area_type_ = goal->type;
  auto_recording_mode_ = goal->auto_recording;

  if (goal->distance_threshold > 0)
  {
    distance_threshold_ = goal->distance_threshold;
  }

  set_recording_mode_service_ = create_service<std_srvs::srv::SetBool>(
      "set_recording_mode", std::bind(&MapRecorderNode::handleSetRecordingMode, this, _1, _2));

  add_boundary_point_service_ = create_service<std_srvs::srv::Trigger>(
      "add_boundary_point", std::bind(&MapRecorderNode::handleAddBoundaryPoint, this, _1, _2));

  finish_area_recording_service_ = create_service<std_srvs::srv::Trigger>(
      "finish_area_recording", std::bind(&MapRecorderNode::handleFinishAreaRecording, this, _1, _2));

  RCLCPP_INFO(get_logger(), "Area recording services are now available");

  std::thread{ [this, goal_handle]() {
    auto feedback = std::make_shared<RecordAreaBoundaryAction::Feedback>();
    auto result = std::make_shared<RecordAreaBoundaryAction::Result>();

    RCLCPP_INFO(get_logger(), "Recording area boundary: %s (type: %u)", current_area_name_.c_str(), current_area_type_);

    try
    {
      last_recorded_position_ = getMowerPose();

      current_boundary_points_.push_back(last_recorded_position_.pose.position);

      geometry_msgs::msg::PolygonStamped area_poly;
      area_poly.header.frame_id = "map";
      area_poly.header.stamp = this->now();

      geometry_msgs::msg::Point32 pt;
      pt.x = last_recorded_position_.pose.position.x;
      pt.y = last_recorded_position_.pose.position.y;
      pt.z = last_recorded_position_.pose.position.z;
      area_poly.polygon.points.push_back(pt);

      feedback->status = RecordAreaBoundaryAction::Feedback::STATUS_RECORDING;
      feedback->message = "Recording started. Added first boundary point.";
      feedback->point_count = 1;
      feedback->area = area_poly;
      goal_handle->publish_feedback(feedback);

      while (rclcpp::ok() && is_recording_area_)
      {
        if (goal_handle->is_canceling())
        {
          result->code = RecordAreaBoundaryAction::Result::CODE_CANCELED;
          result->message = "Area boundary recording was canceled";
          goal_handle->canceled(result);
          is_recording_area_ = false;

          set_recording_mode_service_.reset();
          add_boundary_point_service_.reset();
          finish_area_recording_service_.reset();
          RCLCPP_INFO(get_logger(), "Area recording services have been removed");

          return;
        }

        if (auto_recording_mode_)
        {
          geometry_msgs::msg::PoseStamped current_pose = getMowerPose();
          double dist = utils::poseDistance(current_pose, last_recorded_position_);

          if (dist > distance_threshold_)
          {
            current_boundary_points_.push_back(current_pose.pose.position);
            last_recorded_position_ = current_pose;

            geometry_msgs::msg::PolygonStamped updated_poly;
            updated_poly.header.frame_id = "map";
            updated_poly.header.stamp = this->now();

            for (const auto& point : current_boundary_points_)
            {
              geometry_msgs::msg::Point32 p;
              p.x = point.x;
              p.y = point.y;
              p.z = point.z;
              updated_poly.polygon.points.push_back(p);
            }

            feedback->status = RecordAreaBoundaryAction::Feedback::STATUS_RECORDING;
            feedback->message = "Added point automatically";
            feedback->point_count = current_boundary_points_.size();
            feedback->area = updated_poly;
            goal_handle->publish_feedback(feedback);

            boundary_polygon_pub_->publish(updated_poly);

            RCLCPP_INFO(get_logger(), "Added boundary point #%zu automatically", current_boundary_points_.size());
          }
        }

        std::this_thread::sleep_for(100ms);
      }

      set_recording_mode_service_.reset();
      add_boundary_point_service_.reset();
      finish_area_recording_service_.reset();
      RCLCPP_INFO(get_logger(), "Area recording services have been removed");

      if (current_boundary_points_.size() < 3)
      {
        result->code = RecordAreaBoundaryAction::Result::CODE_TOO_FEW_POINTS;
        result->message = "Not enough points for a valid area boundary (minimum 3)";
        goal_handle->abort(result);
        return;
      }

      feedback->status = RecordAreaBoundaryAction::Feedback::STATUS_PROCESSING;
      feedback->message = "Processing recorded points...";
      goal_handle->publish_feedback(feedback);

      if (!utils::isValidPolygon(current_boundary_points_))
      {
        result->code = RecordAreaBoundaryAction::Result::CODE_INVALID_POLYGON;
        result->message = "Invalid polygon formation";
        goal_handle->abort(result);
        return;
      }

      feedback->status = RecordAreaBoundaryAction::Feedback::STATUS_SAVING;
      feedback->message = "Saving area...";
      goal_handle->publish_feedback(feedback);

      open_mower_next::msg::Area area;
      area.id = utils::generateUniqueId();
      area.name = current_area_name_;
      area.type = current_area_type_;

      geometry_msgs::msg::PolygonStamped polygon;
      polygon.header.frame_id = "map";
      polygon.header.stamp = this->now();

      for (const auto& point : current_boundary_points_)
      {
        geometry_msgs::msg::Point32 pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        polygon.polygon.points.push_back(pt);
      }
      area.area = polygon;

      auto request = std::make_shared<open_mower_next::srv::SaveArea::Request>();
      request->area = area;

      if (!save_area_client_->wait_for_service(10s))
      {
        result->code = RecordAreaBoundaryAction::Result::CODE_SERVICE_UNAVAILABLE;
        result->message = "Save area service not available";
        goal_handle->abort(result);
        return;
      }

      auto future = save_area_client_->async_send_request(request);

      if (future.wait_for(5s) != std::future_status::ready)
      {
        result->code = RecordAreaBoundaryAction::Result::CODE_SERVICE_FAILED;
        result->message = "Save area service timed out";
        goal_handle->abort(result);
        return;
      }

      auto service_response = future.get();
      if (service_response)
      {
        if (service_response->code == open_mower_next::srv::SaveArea::Response::CODE_SUCCESS)
        {
          result->code = RecordAreaBoundaryAction::Result::CODE_SUCCESS;
          result->message = service_response->message;
          result->area = area;

          RCLCPP_INFO(get_logger(), "Area boundary recording completed successfully");
          goal_handle->succeed(result);
        }
        else
        {
          result->code = RecordAreaBoundaryAction::Result::CODE_SERVICE_FAILED;
          result->message = service_response->message;
          RCLCPP_ERROR(get_logger(), "Failed to save area: %s", service_response->message.c_str());
          goal_handle->abort(result);
        }
      }
      else
      {
        result->code = RecordAreaBoundaryAction::Result::CODE_SERVICE_FAILED;
        result->message = "Failed to save area, service call failed";
        goal_handle->abort(result);
      }
    }
    catch (const std::exception& e)
    {
      result->code = RecordAreaBoundaryAction::Result::CODE_UNKNOWN_ERROR;
      result->message = std::string("Error during recording: ") + e.what();
      goal_handle->abort(result);
      is_recording_area_ = false;

      set_recording_mode_service_.reset();
      add_boundary_point_service_.reset();
      finish_area_recording_service_.reset();
      RCLCPP_INFO(get_logger(), "Area recording services have been removed");
    }
  } }.detach();
}

void MapRecorderNode::handleSetRecordingMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  auto_recording_mode_ = request->data;

  response->success = true;
  response->message =
      auto_recording_mode_ ? "Switched to automatic recording mode" : "Switched to manual recording mode";

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MapRecorderNode::handleAddBoundaryPoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  if (auto_recording_mode_)
  {
    response->success = false;
    response->message = "Cannot manually add points in automatic mode";
    return;
  }

  try
  {
    geometry_msgs::msg::PoseStamped current_pose = getMowerPose();
    current_boundary_points_.push_back(current_pose.pose.position);
    last_recorded_position_ = current_pose;

    response->success = true;
    response->message = "Boundary point added successfully";

    RCLCPP_INFO(get_logger(), "Added boundary point #%zu manually", current_boundary_points_.size());

    if (area_boundary_goal_handle_)
    {
      auto feedback = std::make_shared<RecordAreaBoundaryAction::Feedback>();

      geometry_msgs::msg::PolygonStamped updated_poly;
      updated_poly.header.frame_id = "map";
      updated_poly.header.stamp = this->now();

      for (const auto& point : current_boundary_points_)
      {
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        updated_poly.polygon.points.push_back(p);
      }

      feedback->area = updated_poly;
      feedback->message = "Added point manually";
      feedback->point_count = current_boundary_points_.size();
      area_boundary_goal_handle_->publish_feedback(feedback);

      boundary_polygon_pub_->publish(updated_poly);
    }
  }
  catch (const std::exception& e)
  {
    response->success = false;
    response->message = std::string("Failed to add boundary point: ") + e.what();
  }
}

void MapRecorderNode::handleFinishAreaRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!is_recording_area_)
  {
    response->success = false;
    response->message = "No active area boundary recording";
    return;
  }

  if (!area_boundary_goal_handle_)
  {
    response->success = false;
    response->message = "No area boundary goal handle available";
    return;
  }

  is_recording_area_ = false;
  response->success = true;
  response->message = "Area recording finished. Processing and saving...";

  RCLCPP_INFO(get_logger(), "Finalizing area boundary recording with %zu points", current_boundary_points_.size());

  return;
}

void MapRecorderNode::chargingStatusCallback(const omros2_firmware_msgs::msg::PowerStatus::SharedPtr msg)
{
  is_charging_detected_ = msg->charger_present;
}

bool MapRecorderNode::checkPositionCovariance()
{
  // TODO: check robot's pose covariance
  return true;
}

std::string MapRecorderNode::generateUniqueId()
{
  return utils::generateUniqueId();
}

double MapRecorderNode::distanceBetweenPoses(const geometry_msgs::msg::PoseStamped& pose1,
                                             const geometry_msgs::msg::PoseStamped& pose2)
{
  return utils::poseDistance(pose1, pose2);
}

bool MapRecorderNode::validateAreaBoundary()
{
  return utils::isValidPolygon(current_boundary_points_);
}
}  // namespace open_mower_next::map_recorder
