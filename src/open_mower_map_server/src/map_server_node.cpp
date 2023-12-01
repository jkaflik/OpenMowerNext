#include "open_mower_map_server/map_server_node.hpp"
#include "open_mower_map_server/polygon_iterator.hpp"
#include "open_mower_map_server/polygon_utils.hpp"
#include "open_mower_map_server/geo_json_map.hpp"

namespace open_mower_map_server
{
    MapServerNode::MapServerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("map_server_node", options)
    {
        configureMap();
        configureGaussianBlur();

        declare_parameter("world_frame", "map");

        auto occupancy_grid_topic_name_ = declare_parameter("grid.topic_name", "map_grid");
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic_name_,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        auto map_topic_name_ = declare_parameter("topic_name", "map");
        map_publisher_ = this->create_publisher<msg::Map>(map_topic_name_,
                                                          rclcpp::QoS(
                                                              rclcpp::KeepLast(1)).transient_local().reliable());

        publishMap();
    }

    void MapServerNode::publishMap()
    {
        current_map_ = map_io_->load();

        // log areas
        RCLCPP_INFO(get_logger(), "Loaded %lu areas", current_map_.areas.size());
        RCLCPP_INFO(get_logger(), "%-10s %-20s %-10s %10s", "ID", "Name", "Type", "Area");

        for (const auto& area : current_map_.areas)
        {
            std::string type = area.type == msg::Area::TYPE_NAVIGATION
                                   ? "navigation"
                                   : (
                                       area.type == msg::Area::TYPE_LAWN ? "lawn" : "exclusion");

            auto calculated = polygonArea(area.area.polygon);

            RCLCPP_INFO(get_logger(), "%-10s %-20s %-10s %10.2fsqmt", area.id.c_str(), area.name.c_str(), type.c_str(),
                        calculated);
        }

        // log docks
        RCLCPP_INFO(get_logger(), "Loaded %lu docks", current_map_.docking_stations.size());
        RCLCPP_INFO(get_logger(), "%-10s %-20s", "ID", "Name");

        for (const auto& dock : current_map_.docking_stations)
        {
            RCLCPP_INFO(get_logger(), "%-10s %-20s", dock.id.c_str(), dock.name.c_str());
        }

        RCLCPP_INFO(get_logger(), "Publishing map");
        map_publisher_->publish(current_map_);
        RCLCPP_INFO(get_logger(), "Publishing occupancy grid");
        occupancy_grid_publisher_->publish(mapToOccupancyGrid(current_map_));
    }

    void MapServerNode::configureMap()
    {
        map_type_ = declare_parameter("type", "geojson");

        if (map_type_ == "geojson")
        {
            map_file_ = declare_parameter("path", "~/.openmower/map.geojson");
            RCLCPP_INFO(get_logger(), "Using GeoJSON map: %s", map_file_.c_str());
            map_io_ = new GeoJSONMap(map_file_, std::shared_ptr<MapServerNode>(this, [](MapServerNode*)
            {
            }));
        }
        else
        {
            throw std::runtime_error("Unsupported map type: " + map_type_);
        }
    }

    void MapServerNode::configureGaussianBlur()
    {
        use_gaussian_blur_ = declare_parameter("grid.use_gaussian_blur", false);
        if (use_gaussian_blur_)
        {
            RCLCPP_INFO(get_logger(), "Using Gaussian blur");

            std::vector<double> kernel = declare_parameter("map.gaussian_blur.kernel",
                                                           std::vector<double>{1, 2, 1, 2, 4, 2, 1, 2, 1});
            std::vector<float> kernel_float(kernel.begin(), kernel.end());

            gaussian_filter_ = new SomeGaussianFilter(kernel_float);
        }
    }

    std::vector<msg::Area::SharedPtr> MapServerNode::areasWithExclusionsLast(std::vector<msg::Area::SharedPtr> areas)
    {
        std::sort(areas.begin(), areas.end(), [](const msg::Area::SharedPtr& a, const msg::Area::SharedPtr& b)
        {
            if (a->type == b->type)
            {
                return a->type == msg::Area::TYPE_EXCLUSION;
            }

            return a->type < b->type;
        });

        return areas;
    }

    nav_msgs::msg::OccupancyGrid MapServerNode::mapToOccupancyGrid(open_mower_map_server::msg::Map map)
    {
        float minX, minY, maxX, maxY;

        for (const auto& area : map.areas)
        {
            for (const auto& point : area.area.polygon.points)
            {
                minX = std::min(minX, point.x);
                minY = std::min(minY, point.y);
                maxX = std::max(maxX, point.x);
                maxY = std::max(maxY, point.y);
            }
        }

        maxX += 1;
        maxY += 1;
        minX -= 1;
        minY -= 1;

        nav_msgs::msg::OccupancyGrid occupancy_grid;
        occupancy_grid.header = map.header;

        // The origin of the map [m, m, rad]. This is the real-world pose of the bottom left corner of cell (0,0) in the map.
        occupancy_grid.info.origin.position.x = minX;
        occupancy_grid.info.origin.position.y = minY;

        // cell size is in meters
        occupancy_grid.info.resolution = 0.1;

        // occupancy grid width/height is in cells, not meters
        occupancy_grid.info.width = (maxX - minX) / occupancy_grid.info.resolution;
        occupancy_grid.info.height = (maxY - minY) / occupancy_grid.info.resolution;

        occupancy_grid.info.map_load_time = this->now();

        occupancy_grid.data = std::vector<int8_t>(occupancy_grid.info.width * occupancy_grid.info.height, -1);

        for (const auto& area : map.areas)
        {
            uint8_t value = area.type == msg::Area::TYPE_EXCLUSION ? 1 : 0;

            fillGridWithPolygon(occupancy_grid, area.area.polygon, value);
        }

        RCLCPP_INFO(get_logger(), "Occupancy grid size: %dm x %dm (%.2fm resolution)", occupancy_grid.info.width,
                    occupancy_grid.info.height, occupancy_grid.info.resolution);

        if (gaussian_filter_)
        {
            RCLCPP_INFO(get_logger(), "Applying Gaussian filter");

            gaussian_filter_->apply(occupancy_grid.data, occupancy_grid.info.width, occupancy_grid.info.height);
        }

        return occupancy_grid;
    }

    void MapServerNode::fillGridWithPolygon(nav_msgs::msg::OccupancyGrid& occupancy_grid,
                                            const geometry_msgs::msg::Polygon& polygon, uint8_t value)
    {
        auto iterator = PolygonGridIterator(polygon, occupancy_grid.info.resolution);
        while (iterator.next())
        {
            auto point = *iterator;
            auto x = point.x - occupancy_grid.info.origin.position.x;
            auto y = point.y - occupancy_grid.info.origin.position.y;
            auto index = (int) (x / occupancy_grid.info.resolution) + (int) (y / occupancy_grid.info.resolution) * occupancy_grid.info.width;

            if (index >= occupancy_grid.data.size())
            {
                RCLCPP_ERROR(this->get_logger(), "Index out of bounds: %d Grid size: %lu", index,
                            occupancy_grid.data.size());

                continue;
            }

            if (occupancy_grid.data[index] > -1)
            {
                continue; // do not overwrite existing values
            }

            occupancy_grid.data[index] = value;
        }
    }
}
