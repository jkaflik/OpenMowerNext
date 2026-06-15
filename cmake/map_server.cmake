###
# map_server_node
###
add_executable(map_server_node
        src/map_server/node_main.cpp
        src/map_server/map_server_node.hpp
        src/map_server/map_server_node.cpp
        src/map_server/geo_json_map.cpp
        src/map_server/geo_json_map.hpp
        src/map_server/polygon_iterator.hpp
        src/map_server/some_gaussian_filter.hpp
        src/map_server/polygon_utils.hpp)
target_compile_features(map_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(map_server_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_server_node "${cpp_typesupport_target}")


ament_target_dependencies(map_server_node
        std_msgs
        geographic_msgs
        geometry_msgs
        nav_msgs
        foxglove_msgs
        visualization_msgs
        rclcpp
        tf2
        tf2_geometry_msgs
        unique_identifier_msgs
)

target_link_libraries(map_server_node
        nlohmann_json::nlohmann_json
        ${GeographicLib_LIBRARIES}
)

INSTALL(TARGETS map_server_node
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(map_server_node ${PROJECT_NAME})
