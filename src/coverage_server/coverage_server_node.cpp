#include "coverage_server_node.hpp"

#include "utils.h"

#include <tf2/LinearMath/Quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::coverage_server
{
CoverageServerNode::CoverageServerNode(const rclcpp::NodeOptions & options)
: Node("coverage_server", options)
{
  RCLCPP_INFO(get_logger(), "Starting Coverage Server node");

  robot_width_ = declare_parameter("robot_width", 0.325);
  operation_width_ = declare_parameter("operation_width", 0.065);
  min_turning_radius_ = declare_parameter("min_turning_radius", 0.01);

  RCLCPP_INFO(
    get_logger(), "Configured with robot_width=%.3f, mowing_tool_width=%.3f", robot_width_,
    operation_width_);

  area_coverage_service_ = create_service<open_mower_next::srv::AreaCoverage>(
    "area_coverage", std::bind(
                       &CoverageServerNode::handleAreaCoverageRequest, this, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3));

  // Setup map subscription
  map_subscription_ = create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&CoverageServerNode::mapCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "coverage/path", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "coverage/visualization", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

  RCLCPP_INFO(get_logger(), "Coverage Server initialized and ready");
}

CoverageServerNode::~CoverageServerNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down Coverage Server node");
}

void CoverageServerNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  current_map_ = *msg;
  RCLCPP_INFO(get_logger(), "Received updated map with %zu areas", msg->areas.size());
}

void CoverageServerNode::handleAreaCoverageRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
  std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response)
{
  if (request->area_id.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area ID cannot be empty";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received area coverage request for area ID: %s", request->area_id.c_str());

  const auto area = findAreaById(request->area_id);

  if (area == nullptr) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area not found";
    return;
  }

  if (area.get()->type != open_mower_next::msg::Area::TYPE_OPERATION) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area is not of type OPERATION";
    return;
  }

  const auto & area_polygon = area.get()->area;

  if (area_polygon.polygon.points.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area has an invalid polygon";
    return;
  }

  // Find exclusions if requested
  std::vector<geometry_msgs::msg::PolygonStamped> exclusion_polygons;
  if (request->with_exclusions) {
    findExclusionsInPolygon(area_polygon, exclusion_polygons);
  }

  f2c::types::Robot robot(robot_width_, operation_width_, min_turning_radius_);

  const auto swaths = generateSwaths(
    robot, area_polygon, exclusion_polygons, request->headland_loops, request->swath_angle);

  nav_msgs::msg::Path path = utils::toMsg(swaths, area_polygon.header.frame_id);

  response->message = "Coverage path generated successfully";
  response->code = open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS;
  response->path = path;
  response->polygon = area_polygon;
  response->area_id = request->area_id;

  const auto markers = createVisualizationMarkers(swaths, area_polygon.header.frame_id);
  visualization_pub_->publish(markers);

  RCLCPP_INFO(
    get_logger(), "Generated path with a %zu poses for area ID: %s", path.poses.size(),
    request->area_id.c_str());
}

msg::Area::SharedPtr CoverageServerNode::findAreaById(const std::string & area_id)
{
  geometry_msgs::msg::PolygonStamped result;
  result.header.frame_id = "map";
  result.header.stamp = now();

  for (const auto & area : current_map_.areas) {
    if (area.id == area_id) {
      RCLCPP_INFO(
        get_logger(), "Found area with ID: %s, name: %s", area_id.c_str(), area.name.c_str());

      return std::make_shared<msg::Area>(area);
    }
  }

  return nullptr;
}

std::vector<std::string> CoverageServerNode::findAreasInPolygon(
  const geometry_msgs::msg::PolygonStamped & polygon)
{
  std::vector<std::string> area_ids;

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_OPERATION) {
      // TODO: Implement proper polygon intersection test
      area_ids.push_back(area.id);
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu operation areas within the polygon", area_ids.size());
  return area_ids;
}

void CoverageServerNode::findExclusionsInPolygon(
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons)
{
  exclusion_polygons.clear();

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_EXCLUSION) {
      // TODO: Implement proper polygon intersection test.
      // There are three scenarios:
      // 1. The exclusion area is completely inside the field polygon. Just add it.
      // 2. The exclusion area is completely outside the field polygon. Ignore it.
      // 3. The exclusion area intersects with the field polygon. Cut the intersection from the
      // field polygon.

      geometry_msgs::msg::PolygonStamped exclusion;
      exclusion.header = field_polygon.header;
      exclusion.polygon = area.area.polygon;

      exclusion_polygons.push_back(exclusion);
      RCLCPP_INFO(
        get_logger(), "Adding exclusion area with ID: %s, name: %s", area.id.c_str(),
        area.name.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu exclusion areas", exclusion_polygons.size());
}

f2c::types::Swaths CoverageServerNode::generateSwaths(
  const f2c::types::Robot & robot, const geometry_msgs::msg::PolygonStamped & field_polygon,
  const std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons,
  const uint16_t headland_loops = 0, const uint16_t swath_angle = 0)
{
  f2c::hg::ConstHL hg;

  RCLCPP_INFO(get_logger(), "Generating swaths...");

  f2c::types::Cells cells{utils::toCell(field_polygon, exclusion_polygons)};

  auto mainland = cells;

  if (headland_loops > 0) {
    auto headland_width = robot.getCovWidth() * headland_loops;

    RCLCPP_INFO(
      get_logger(), "Generating headland with width=%.2f (%d loops of %.2f)", headland_width,
      headland_loops, robot.getCovWidth());

    mainland = hg.generateHeadlands(cells, headland_width);
  }

  f2c::sg::BruteForce swath_generator;
  f2c::obj::SwathLength swath_objective;

  const auto best_angle =
    swath_generator.computeBestAngle(swath_objective, robot.getCovWidth(), mainland.getGeometry(0));
  const auto requested_angle = best_angle + (swath_angle * M_PI / 180.0);

  f2c::types::Swaths swaths =
    swath_generator.generateSwaths(requested_angle, robot.getCovWidth(), mainland.getGeometry(0));

  RCLCPP_INFO(get_logger(), "Generated %zu swaths", swaths.size());

  const f2c::rp::BoustrophedonOrder sorter;

  f2c::types::Swaths sorted_swaths = sorter.genSortedSwaths(swaths);

  if (headland_loops > 0) {
    auto cells_vector = hg.generateHeadlandSwaths(cells, operation_width_, headland_loops, false);

    for (auto & cell : cells_vector) {
      for (auto & ring : cell.getGeometry(0)) {
        sorted_swaths.append(
          f2c::types::LineString(ring), robot.getCovWidth(), f2c::types::SwathType::HEADLAND);
      }
    }

    RCLCPP_INFO(get_logger(), "Generated %zu headland swaths", cells_vector.size());
  }

  return sorted_swaths;
}

visualization_msgs::msg::MarkerArray CoverageServerNode::createVisualizationMarkers(
  const f2c::types::Swaths & swaths, const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray markers;

  // Add a deletion marker to clear all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = frame_id;
  delete_marker.header.stamp = now();
  delete_marker.ns = "all";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  for (int i = 0; i < swaths.size(); ++i) {
    if (i > 0) {
      visualization_msgs::msg::Marker connection_marker;
      connection_marker.ns = "connections";
      connection_marker.color.r = 0.0f;
      connection_marker.color.g = 0.0f;
      connection_marker.color.b = 1.0f;
      connection_marker.color.a = 1.0f;
      connection_marker.header.frame_id = frame_id;
      connection_marker.header.stamp = now();
      connection_marker.id = i;
      connection_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      connection_marker.action = visualization_msgs::msg::Marker::ADD;
      connection_marker.scale.x = 0.01;
      connection_marker.points.push_back(utils::toMsg(swaths[i-1].endPoint()));
      connection_marker.points.push_back(utils::toMsg(swaths[i].startPoint()));

      markers.markers.push_back(connection_marker);
    }

    auto swath = swaths[i];

    visualization_msgs::msg::Marker marker;

    if (swath.getType() == f2c::types::SwathType::HEADLAND) {
      marker.ns = "headland";
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
    } else {
      marker.ns = "mainland";
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
    }

    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = swath.getWidth();

    const auto start_point = swath.startPoint();
    const auto end_point = swath.endPoint();

    for (int j = 0; j < swath.numPoints(); ++j) {
      const auto point = swath.getPoint(j);
      marker.points.push_back(utils::toMsg(point));
    }

    markers.markers.push_back(marker);
  }

  return markers;
}

}  // namespace open_mower_next::coverage_server