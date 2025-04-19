#include "map_server/geo_json_map.hpp"
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace open_mower_next::map_server
{
GeoJSONMap::GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node) : path_(path), node_(map_server_node)
{
  if (path_.empty())
  {
    throw std::invalid_argument("GeoJSON file path cannot be empty");
  }

  std::vector<double> datum_vals;
  datum_vals = node_->declare_parameter("datum", datum_vals);
  if (datum_vals.size() == 2)
  {
    datum_lat_ = datum_vals[0];
    datum_lon_ = datum_vals[1];
  }
  else
  {
    throw std::invalid_argument("Datum parameter must contain exactly two values (lat, lon)");
  }

  initializeGeographicLibTransformer();
  publishDatum();

  auto topic_name = node_->declare_parameter("foxglove_geojson_topic_name", "map/foxglove_geojson");
  foxglove_geo_json_publisher_ = node_->create_publisher<foxglove_msgs::msg::GeoJSON>(
      topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void GeoJSONMap::initializeGeographicLibTransformer()
{
  // Initialize the GeographicLib local cartesian projection
  local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(datum_lat_, datum_lon_, 0.0);
  datum_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Initialized GeographicLib with datum: lat=%f, lon=%f", datum_lat_, datum_lon_);
}

json GeoJSONMap::pointToCoordinates(const geometry_msgs::msg::Point& point) const
{
  auto geo_point = mapToLL(point);
  json p = json::array();
  p.push_back(geo_point.longitude);
  p.push_back(geo_point.latitude);
  return p;
}

json GeoJSONMap::pointToCoordinates(const geometry_msgs::msg::Point32& point) const
{
  auto geo_point = mapToLL(point.x, point.y, 0.0);
  json p = json::array();
  p.push_back(geo_point.longitude);
  p.push_back(geo_point.latitude);
  return p;
}

json GeoJSONMap::mapAreaToGeoJSONFeature(const msg::Area& area) const
{
  auto colorByType = [](const std::string& type) -> std::string {
    if (type == "navigation")
    {
      return "#0000ff";
    }
    if (type == "operation")
    {
      return "#00ff00";
    }
    if (type == "exclusion")
    {
      return "#ff0000";
    }
    return "#000000";
  };

  json feature;
  feature["type"] = "Feature";
  feature["properties"]["id"] = area.id;
  feature["properties"]["name"] = area.name;
  feature["properties"]["type"] = area.type == msg::Area::TYPE_NAVIGATION ?
                                      "navigation" :
                                      (area.type == msg::Area::TYPE_OPERATION ? "operation" : "exclusion");
  feature["properties"]["fill"] = colorByType(feature["properties"]["type"].get<std::string>());
  feature["properties"]["style"]["color"] = feature["properties"]["fill"];

  feature["geometry"]["type"] = "Polygon";
  feature["geometry"]["coordinates"] = json::array();

  json coordinates = json::array();
  for (const auto& point : area.area.polygon.points)
  {
    json p = pointToCoordinates(point);
    coordinates.push_back(p);
  }
  feature["geometry"]["coordinates"].push_back(coordinates);

  return feature;
}

geometry_msgs::msg::Point GeoJSONMap::movePointTowardsOrientation(const geometry_msgs::msg::Point& point,
                                                                  const geometry_msgs::msg::Quaternion& quaternion,
                                                                  double x) const
{
  tf2::Quaternion tf2Quaternion;
  tf2::fromMsg(quaternion, tf2Quaternion);
  tf2::Matrix3x3 mat(tf2Quaternion);

  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  geometry_msgs::msg::Point outputPoint;

  outputPoint.x = point.x + x * std::cos(yaw);
  outputPoint.y = point.y + x * std::sin(yaw);
  outputPoint.z = point.z;

  return outputPoint;
}

json GeoJSONMap::dockingStationToGeoJSONFeature(const msg::DockingStation& docking_station)
{
  json feature;
  feature["type"] = "Feature";
  feature["properties"]["id"] = docking_station.id;
  feature["properties"]["name"] = docking_station.name;
  feature["properties"]["type"] = "docking_station";
  feature["geometry"]["type"] = "LineString";

  auto coordinates = json::array();

  // calculate orientation point
  auto orientationPoint =
      movePointTowardsOrientation(docking_station.pose.pose.position, docking_station.pose.pose.orientation, 0.5);

  coordinates.push_back(pointToCoordinates(docking_station.pose.pose.position));
  coordinates.push_back(pointToCoordinates(orientationPoint));

  feature["geometry"]["coordinates"] = coordinates;

  return feature;
}

void GeoJSONMap::eventuallyPublishFoxgloveGeoJSON(json data) const
{
  if (foxglove_geo_json_publisher_ == nullptr)
  {
    RCLCPP_INFO(node_->get_logger(), "Foxglove GeoJSON publisher not initialized, skipping");
    return;
  }

  foxglove_msgs::msg::GeoJSON msg;
  msg.geojson = data.dump();

  foxglove_geo_json_publisher_->publish(msg);
}

msg::Map GeoJSONMap::load()
{
  std::ifstream f(path_, std::ios::in);
  if (!f.is_open())
  {
    throw std::runtime_error("Could not open GeoJSON file: " + path_);
  }

  json data = json::parse(f);

  if (data["type"] != "FeatureCollection")
  {
    throw std::runtime_error("GeoJSON file is not a FeatureCollection type");
  }

  msg::Map map;
  map.header.stamp = node_->now();  // or use the modification time of the file?
  map.header.frame_id = node_->get_parameter("world_frame").as_string();

  for (const auto& feature : data["features"])
  {
    if (feature["type"] != "Feature")
    {
      RCLCPP_WARN(node_->get_logger(), "Non-feature object found in GeoJSON file");
      continue;
    }

    if (feature["geometry"]["type"] == "Polygon")
    {
      parsePolygonFeature(map, feature);
      continue;
    }

    if (feature["geometry"]["type"] == "LineString")
    {
      parseLineStringFeature(map, feature);
      continue;
    }

    RCLCPP_WARN(node_->get_logger(), "Unsupported geometry type: %s",
                feature["geometry"]["type"].get<std::string>().c_str());
  }

  eventuallyPublishFoxgloveGeoJSON(data);

  return map;
}

void GeoJSONMap::save(msg::Map map)
{
  RCLCPP_DEBUG(node_->get_logger(), "Building GeoJSON data");

  json data;
  data["type"] = "FeatureCollection";
  data["features"] = json::array();

  for (const auto& area : map.areas)
  {
    auto feature = mapAreaToGeoJSONFeature(area);
    data["features"].push_back(feature);
  }

  for (const auto& docking_station : map.docking_stations)
  {
    auto feature = dockingStationToGeoJSONFeature(docking_station);
    data["features"].push_back(feature);
  }

  RCLCPP_INFO(node_->get_logger(), "Saving map to %s", path_.c_str());
  std::ofstream out_file(path_);
  if (!out_file.is_open())
  {
    throw std::runtime_error("Failed to open file for writing: " + path_);
  }
  out_file << std::setw(4) << data << std::endl;
  out_file.close();
  RCLCPP_INFO(node_->get_logger(), "Map saved successfully");

  eventuallyPublishFoxgloveGeoJSON(data);
}

void GeoJSONMap::parsePolygonFeature(msg::Map& map, const json& feature)
{
  msg::Area area;
  area.id = feature["properties"].value("id", "");
  area.name = feature["properties"].value("name", "");

  area.type =
      feature["properties"]["type"] == "navigation" ?
          msg::Area::TYPE_NAVIGATION :
          (feature["properties"]["type"] == "operation" ? msg::Area::TYPE_OPERATION : msg::Area::TYPE_EXCLUSION);

  for (const auto& ll : feature["geometry"]["coordinates"][0])
  {
    auto p = parsePoint(ll);
    area.area.polygon.points.push_back(p);
  }

  map.areas.push_back(area);
}

geometry_msgs::msg::Point32 GeoJSONMap::parsePoint(json::const_reference value) const
{
  geometry_msgs::msg::Point32 p;

  auto map_point = llToMap(value[1], value[0], 0.0);
  p.x = map_point.x;
  p.y = map_point.y;

  return p;
}

geometry_msgs::msg::PoseStamped GeoJSONMap::calculateTwoPointsPose(geometry_msgs::msg::Point32 origin,
                                                                   geometry_msgs::msg::Point32 point32)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = node_->get_parameter("world_frame").as_string();
  pose.header.stamp = node_->now();
  pose.pose.position.x = origin.x;
  pose.pose.position.y = origin.y;

  const double yaw = atan2(point32.y - origin.y, point32.x - origin.x);
  pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));

  return pose;
}

void GeoJSONMap::parseLineStringFeature(msg::Map& map, const json& feature)
{
  if (feature["properties"]["type"] != "docking_station")
  {
    RCLCPP_WARN(node_->get_logger(), "Unsupported line string type: %s",
                feature["properties"]["type"].get<std::string>().c_str());
    return;
  }

  msg::DockingStation docking_station;
  docking_station.id = feature["properties"]["id"];
  docking_station.name = feature["properties"]["name"];

  // docking station line string must have two coordinates
  // the first one is the origin of the docking station (usually a middle of the charging connectors)
  // the second point is used to determine the orientation of the docking station (towards the robot's connectors)

  if (feature["geometry"]["coordinates"].size() < 2)
  {
    RCLCPP_WARN(node_->get_logger(), "Docking station line string must have at least two coordinates");
    return;
  }

  if (feature["geometry"]["coordinates"][0].size() > 2)
  {
    RCLCPP_WARN(node_->get_logger(),
                "Docking station expected to have two coordinates, but has %d. Reading only the first two.",
                static_cast<int>(feature["geometry"]["coordinates"][0].size()));
  }

  const geometry_msgs::msg::Point32 origin = parsePoint(feature["geometry"]["coordinates"][0]);
  const geometry_msgs::msg::Point32 orientation = parsePoint(feature["geometry"]["coordinates"][1]);

  docking_station.pose = calculateTwoPointsPose(origin, orientation);

  map.docking_stations.push_back(docking_station);
}

void GeoJSONMap::publishDatum()
{
  if (!datum_initialized_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Datum not initialized, cannot publish");
    return;
  }

  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = datum_lat_;
  geo_point.longitude = datum_lon_;

  auto datum_geopoint_publisher = node_->create_publisher<geographic_msgs::msg::GeoPoint>(
      "map/datum", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  datum_geopoint_publisher->publish(geo_point);
  datum_geopoint_publisher.reset();

  RCLCPP_INFO(node_->get_logger(), "Datum published");
  RCLCPP_INFO(node_->get_logger(), "Datum: %f, %f, %f", geo_point.latitude, geo_point.longitude, geo_point.altitude);
}

geographic_msgs::msg::GeoPoint GeoJSONMap::mapToLL(const geometry_msgs::msg::Point& map_point) const
{
  return mapToLL(map_point.x, map_point.y, map_point.z);
}

geographic_msgs::msg::GeoPoint GeoJSONMap::mapToLL(double x, double y, double z) const
{
  if (!datum_initialized_ || !local_cartesian_)
  {
    RCLCPP_ERROR(node_->get_logger(), "GeographicLib transform not initialized");
    return geographic_msgs::msg::GeoPoint();
  }

  double lat, lon, h;
  local_cartesian_->Reverse(x, y, z, lat, lon, h);

  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = lat;
  geo_point.longitude = lon;
  geo_point.altitude = h;

  return geo_point;
}

geometry_msgs::msg::Point GeoJSONMap::llToMap(const geographic_msgs::msg::GeoPoint& ll_point) const
{
  return llToMap(ll_point.latitude, ll_point.longitude, ll_point.altitude);
}

geometry_msgs::msg::Point GeoJSONMap::llToMap(double lat, double lon, double alt) const
{
  if (!datum_initialized_ || !local_cartesian_)
  {
    RCLCPP_ERROR(node_->get_logger(), "GeographicLib transform not initialized");
    return geometry_msgs::msg::Point();
  }

  double x, y, z;
  local_cartesian_->Forward(lat, lon, alt, x, y, z);

  geometry_msgs::msg::Point map_point;
  map_point.x = x;
  map_point.y = y;
  map_point.z = z;

  return map_point;
}
}  // namespace open_mower_next::map_server
