#pragma once

#include <nlohmann/json.hpp>
#include <foxglove_msgs/msg/geo_json.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "map_server_node.hpp"

using json = nlohmann::json;

namespace open_mower_next::map_server
{
class GeoJSONMap : public MapIO
{
public:
  explicit GeoJSONMap(std::string path, MapServerNode::SharedPtr map_server_node);
  void save(msg::Map map);
  msg::Map load();

private:
  void parsePolygonFeature(msg::Map& map, const json& feature);
  geometry_msgs::msg::Point32 parsePoint(json::const_reference value) const;
  geometry_msgs::msg::PoseStamped calculateTwoPointsPose(geometry_msgs::msg::Point32 origin,
                                                         geometry_msgs::msg::Point32 point32);
  void parseLineStringFeature(msg::Map& map, const json& feature);
  json pointToCoordinates(const geometry_msgs::msg::Point& point) const;
  json pointToCoordinates(const geometry_msgs::msg::Point32& point) const;
  json mapAreaToGeoJSONFeature(const msg::Area& area) const;
  geometry_msgs::msg::Point movePointTowardsOrientation(const geometry_msgs::msg::Point& point,
                                                        const geometry_msgs::msg::Quaternion& quaternion,
                                                        double x) const;
  json dockingStationToGeoJSONFeature(const msg::DockingStation& docking_station);

  MapServerNode::SharedPtr node_;

  void initializeGeographicLibTransformer();
  void publishDatum();

  void eventuallyPublishFoxgloveGeoJSON(json data) const;
  rclcpp::Publisher<foxglove_msgs::msg::GeoJSON>::SharedPtr foxglove_geo_json_publisher_;

  std::string path_;

  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  geographic_msgs::msg::GeoPoint mapToLL(const geometry_msgs::msg::Point& map_point) const;
  geographic_msgs::msg::GeoPoint mapToLL(double x, double y, double z = 0.0) const;
  geometry_msgs::msg::Point llToMap(const geographic_msgs::msg::GeoPoint& ll_point) const;
  geometry_msgs::msg::Point llToMap(double lat, double lon, double alt = 0.0) const;

  double datum_lat_ = 0.0;
  double datum_lon_ = 0.0;
  bool datum_initialized_ = false;
};

}  // namespace open_mower_next::map_server
