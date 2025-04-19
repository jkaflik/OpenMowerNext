#include "docking_helper/docking_helper_node.hpp"
#include "docking_helper_node.hpp"
#include <functional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::placeholders;

open_mower_next::docking_helper::DockingHelperNode::DockingHelperNode(const rclcpp::NodeOptions& options)
  : Node("docking_helper", options)
{
  map_sub_ = create_subscription<open_mower_next::msg::Map>(
      "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
      std::bind(&DockingHelperNode::mapCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  find_nearest_docking_station_service_ = create_service<open_mower_next::srv::FindNearestDockingStation>(
      "/find_nearest_docking_station", std::bind(&DockingHelperNode::findNearestDockingStationService, this,
                                                 std::placeholders::_1, std::placeholders::_2));

  dock_client_ = rclcpp_action::create_client<nav2_msgs::action::DockRobot>(this, "/dock_robot");

  dock_robot_nearest_server_ = rclcpp_action::create_server<DockRobotNearestAction>(
      this, "dock_robot_nearest", std::bind(&DockingHelperNode::handleDockRobotNearestGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotNearestCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotNearestAccepted, this, _1));

  dock_robot_to_server_ = rclcpp_action::create_server<DockRobotToAction>(
      this, "dock_robot_to", std::bind(&DockingHelperNode::handleDockRobotToGoal, this, _1, _2),
      std::bind(&DockingHelperNode::handleDockRobotToCancel, this, _1),
      std::bind(&DockingHelperNode::handleDockRobotToAccepted, this, _1));

  while (!dock_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(get_logger(), "Waiting for dock action server...");
  }
  RCLCPP_INFO(get_logger(), "Dock action server is available");
}

open_mower_next::docking_helper::DockingHelperNode::~DockingHelperNode()
{
  // No specific cleanup required
}

void open_mower_next::docking_helper::DockingHelperNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received map with %zu docking stations", msg->docking_stations.size());

  docking_stations_.clear();

  for (const auto& docking_station : msg->docking_stations)
  {
    RCLCPP_DEBUG(get_logger(), "Docking station: %s", docking_station.name.c_str());

    docking_stations_.push_back(docking_station);
  }
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStation()
{
  if (docking_stations_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No docking stations available");
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header.frame_id = "map";
  pose_in.header.stamp = this->now();
  pose_in.pose.position.x = 0.0;
  pose_in.pose.position.y = 0.0;
  pose_in.pose.position.z = 0.0;
  pose_in.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped robot_pose;

  try
  {
    // Transform from map frame to base_link frame
    robot_pose = tf_buffer_->transform(pose_in, "base_link", tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not transform pose: %s", ex.what());
    return nullptr;
  }

  double min_distance = std::numeric_limits<double>::max();
  open_mower_next::msg::DockingStation nearest_station;

  for (const auto& station : docking_stations_)
  {
    double dx = station.pose.pose.position.x - robot_pose.pose.position.x;
    double dy = station.pose.pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_station = station;
    }
  }

  RCLCPP_INFO(get_logger(), "Found nearest docking station at distance: %f meters", min_distance);
  return std::make_shared<open_mower_next::msg::DockingStation>(nearest_station);
}

void open_mower_next::docking_helper::DockingHelperNode::findNearestDockingStationService(
    const std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Request> request,
    std::shared_ptr<open_mower_next::srv::FindNearestDockingStation::Response> response)
{
  try
  {
    auto nearest_station = findNearestDockingStation();

    if (!nearest_station)
    {
      response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_NOT_FOUND;
      RCLCPP_ERROR(get_logger(), "Failed to find nearest docking station");
      return;
    }

    response->docking_station = *nearest_station;
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception occured while finding nearest docking station: %s", e.what());
    response->code = open_mower_next::srv::FindNearestDockingStation::Response::CODE_UNKNOWN_ERROR;
    return;
  }
}

std::shared_ptr<geometry_msgs::msg::PoseStamped> open_mower_next::docking_helper::DockingHelperNode::dockingStationPose(
    const std::shared_ptr<open_mower_next::msg::DockingStation>& station)
{
  if (!station)
  {
    RCLCPP_ERROR(get_logger(), "Cannot transform null docking station");
    return nullptr;
  }

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped->header = station->pose.header;
  pose_stamped->pose = station->pose.pose;

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::fromMsg(pose_stamped->pose.orientation, q_orig);

  q_rot.setRPY(0.0, 0.0, M_PI);  // Rotate 180 degrees around Z
  q_new = q_orig * q_rot;        // Apply rotation
  q_new.normalize();

  pose_stamped->pose.orientation = tf2::toMsg(q_new);

  return pose_stamped;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotNearestAction::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(get_logger(), "Received request to dock to nearest docking station");

  if (docking_stations_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No docking stations available");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestCancel(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to nearest station");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotNearestAccepted(
    const std::shared_ptr<DockRobotNearestGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto feedback = std::make_shared<DockRobotNearestAction::Feedback>();
    auto result = std::make_shared<DockRobotNearestAction::Result>();

    feedback->status = DockRobotNearestAction::Feedback::STATUS_NONE;
    feedback->num_retries = 0;
    feedback->docking_time.sec = 0;
    feedback->docking_time.nanosec = 0;

    auto nearest_station = findNearestDockingStation();
    if (!nearest_station)
    {
      result->code = DockRobotNearestAction::Result::CODE_DOCK_NOT_IN_DB;
      result->message = "No docking stations available";
      result->num_retries = 0;
      goal_handle->abort(result);
      return;
    }

    feedback->chosen_docking_station = *nearest_station;
    result->chosen_docking_station = *nearest_station;

    feedback->status = DockRobotNearestAction::Feedback::STATUS_NAV_TO_STAGING_POSE;
    feedback->message = "Starting docking to: " + nearest_station->name;
    goal_handle->publish_feedback(feedback);

    auto start_time = this->now();

    std::shared_ptr<uint16_t> current_status =
        std::make_shared<uint16_t>(DockRobotNearestAction::Feedback::STATUS_NONE);
    std::shared_ptr<uint16_t> current_retries = std::make_shared<uint16_t>(0);
    std::atomic<bool> docking_active(true);

    auto nav2_goal = nav2_msgs::action::DockRobot::Goal();
    nav2_goal.use_dock_id = false;
    nav2_goal.navigate_to_staging_pose = true;
    nav2_goal.dock_pose.header = nearest_station->pose.header;
    nav2_goal.dock_pose.pose = dockingStationPose(nearest_station)->pose;

    if (!dock_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      result->code = DockRobotNearestAction::Result::CODE_UNKNOWN;
      result->message = "Dock robot action server not available";
      result->num_retries = 0;
      goal_handle->abort(result);
      return;
    }

    // Send the goal
    RCLCPP_INFO(get_logger(), "Sending docking goal to nearest station");
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::DockRobot>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this, current_status,
         current_retries](typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::DockRobot>::SharedPtr,
                          const std::shared_ptr<const nav2_msgs::action::DockRobot::Feedback> feedback) {
          RCLCPP_INFO(get_logger(), "Docking state: %d, retries: %d", feedback->state, feedback->num_retries);

          *current_status = feedback->state;
          *current_retries = feedback->num_retries;
        };

    send_goal_options.goal_response_callback = [this](const auto& goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(get_logger(), "Docking goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Docking goal accepted by server");
      }
    };

    send_goal_options.result_callback = [this, goal_handle, result, &docking_active](const auto& nav_result) {
      auto status = nav_result.result;
      bool success = status->success;
      uint16_t error_code = status->error_code;
      uint16_t num_retries = status->num_retries;

      docking_active = false;

      result->num_retries = num_retries;

      if (success)
      {
        RCLCPP_INFO(get_logger(), "Docking action succeeded");
        result->code = DockRobotNearestAction::Result::CODE_SUCCESS;
        result->message = "Docking completed successfully";
        goal_handle->succeed(result);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Docking action failed with error code: %d, message: %s", error_code,
                     status->error_msg.c_str());

        result->code = error_code;
        result->message = status->error_msg.empty() ? "Docking failed" : status->error_msg;
        goal_handle->abort(result);
      }
    };

    dock_client_->async_send_goal(nav2_goal, send_goal_options);

    std::thread feedback_thread([&]() {
      std::string status_messages[] = {
        "No activity",         "Navigating to staging pose", "Initial perception of dock",
        "Controlling to dock", "Waiting for charge",         "Retrying docking"
      };

      uint16_t last_status = 99;  // Invalid value to ensure first update is sent
      uint16_t last_retries = 0;

      while (docking_active && rclcpp::ok())
      {
        auto current_time = this->now();
        auto elapsed = current_time - start_time;
        feedback->docking_time.sec = elapsed.seconds();
        feedback->docking_time.nanosec = elapsed.nanoseconds() % 1000000000;

        uint16_t status = *current_status;
        uint16_t retries = *current_retries;

        if (status != last_status || retries != last_retries)
        {
          feedback->status = status;
          feedback->num_retries = retries;

          if (status < sizeof(status_messages) / sizeof(status_messages[0]))
          {
            feedback->message = status_messages[status];
            if (retries > 0)
            {
              feedback->message += " (retry " + std::to_string(retries) + ")";
            }
          }
          else
          {
            feedback->message = "Unknown status: " + std::to_string(status);
          }

          last_status = status;
          last_retries = retries;

          goal_handle->publish_feedback(feedback);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });

    if (feedback_thread.joinable())
    {
      feedback_thread.join();
    }
  } }.detach();
}

std::shared_ptr<open_mower_next::msg::DockingStation>
open_mower_next::docking_helper::DockingHelperNode::findDockingStationById(const std::string& id)
{
  for (const auto& station : docking_stations_)
  {
    if (station.id == id)
    {
      return std::make_shared<open_mower_next::msg::DockingStation>(station);
    }
  }
  return nullptr;
}

rclcpp_action::GoalResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DockRobotToAction::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received request to dock to station ID: %s", goal->dock_id.c_str());

  if (docking_stations_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No docking stations available");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto docking_station = findDockingStationById(goal->dock_id);
  if (!docking_station)
  {
    RCLCPP_ERROR(get_logger(), "Docking station with ID %s not found", goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToCancel(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel docking to station ID: %s",
              goal_handle->get_goal()->dock_id.c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void open_mower_next::docking_helper::DockingHelperNode::handleDockRobotToAccepted(
    const std::shared_ptr<DockRobotToGoalHandle> goal_handle)
{
  std::thread{ [this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DockRobotToAction::Feedback>();
    auto result = std::make_shared<DockRobotToAction::Result>();

    feedback->status = DockRobotToAction::Feedback::STATUS_NONE;
    feedback->num_retries = 0;
    feedback->docking_time.sec = 0;
    feedback->docking_time.nanosec = 0;

    auto docking_station = findDockingStationById(goal->dock_id);
    if (!docking_station)
    {
      result->code = DockRobotToAction::Result::CODE_DOCK_NOT_IN_DB;
      result->message = "Docking station with ID " + goal->dock_id + " not found";
      result->num_retries = 0;
      goal_handle->abort(result);
      return;
    }

    feedback->chosen_docking_station = *docking_station;
    result->chosen_docking_station = *docking_station;

    feedback->status = DockRobotToAction::Feedback::STATUS_NAV_TO_STAGING_POSE;
    feedback->message = "Starting docking to: " + docking_station->name;
    goal_handle->publish_feedback(feedback);

    auto start_time = this->now();

    std::shared_ptr<uint16_t> current_status = std::make_shared<uint16_t>(DockRobotToAction::Feedback::STATUS_NONE);
    std::shared_ptr<uint16_t> current_retries = std::make_shared<uint16_t>(0);
    std::atomic<bool> docking_active(true);

    auto nav2_goal = nav2_msgs::action::DockRobot::Goal();
    nav2_goal.use_dock_id = false;
    nav2_goal.navigate_to_staging_pose = true;
    nav2_goal.dock_pose.header = docking_station->pose.header;
    nav2_goal.dock_pose.pose = dockingStationPose(docking_station)->pose;

    if (!dock_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      result->code = DockRobotToAction::Result::CODE_UNKNOWN;
      result->message = "Dock robot action server not available";
      result->num_retries = 0;
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(get_logger(), "Sending docking goal to station ID: %s", goal->dock_id.c_str());
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::DockRobot>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this, current_status,
         current_retries](typename rclcpp_action::ClientGoalHandle<nav2_msgs::action::DockRobot>::SharedPtr,
                          const std::shared_ptr<const nav2_msgs::action::DockRobot::Feedback> feedback) {
          RCLCPP_INFO(get_logger(), "Docking state: %d, retries: %d", feedback->state, feedback->num_retries);

          *current_status = feedback->state;
          *current_retries = feedback->num_retries;
        };

    send_goal_options.goal_response_callback = [this](const auto& goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(get_logger(), "Docking goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Docking goal accepted by server");
      }
    };

    send_goal_options.result_callback = [this, goal_handle, result, &docking_active](const auto& nav_result) {
      auto status = nav_result.result;
      bool success = status->success;
      uint16_t error_code = status->error_code;
      uint16_t num_retries = status->num_retries;

      docking_active = false;

      result->num_retries = num_retries;

      if (success)
      {
        RCLCPP_INFO(get_logger(), "Docking action succeeded");
        result->code = DockRobotToAction::Result::CODE_SUCCESS;
        result->message = "Docking completed successfully";
        goal_handle->succeed(result);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Docking action failed with error code: %d, message: %s", error_code,
                     status->error_msg.c_str());

        result->code = error_code;
        result->message = status->error_msg.empty() ? "Docking failed" : status->error_msg;
        goal_handle->abort(result);
      }
    };

    dock_client_->async_send_goal(nav2_goal, send_goal_options);

    std::thread feedback_thread([&]() {
      std::string status_messages[] = {
        "No activity",         "Navigating to staging pose", "Initial perception of dock",
        "Controlling to dock", "Waiting for charge",         "Retrying docking"
      };

      uint16_t last_status = 99;  // Invalid value to ensure first update is sent
      uint16_t last_retries = 0;

      while (docking_active && rclcpp::ok())
      {
        auto current_time = this->now();
        auto elapsed = current_time - start_time;
        feedback->docking_time.sec = elapsed.seconds();
        feedback->docking_time.nanosec = elapsed.nanoseconds() % 1000000000;

        uint16_t status = *current_status;
        uint16_t retries = *current_retries;

        if (status != last_status || retries != last_retries)
        {
          feedback->status = status;
          feedback->num_retries = retries;

          if (status < sizeof(status_messages) / sizeof(status_messages[0]))
          {
            feedback->message = status_messages[status];
            if (retries > 0)
            {
              feedback->message += " (retry " + std::to_string(retries) + ")";
            }
          }
          else
          {
            feedback->message = "Unknown status: " + std::to_string(status);
          }

          last_status = status;
          last_retries = retries;

          goal_handle->publish_feedback(feedback);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });

    if (feedback_thread.joinable())
    {
      feedback_thread.join();
    }
  } }.detach();
}
