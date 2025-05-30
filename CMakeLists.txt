cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(opennav_docking REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "src/srv/RemoveArea.srv"
        "src/srv/RemoveDockingStation.srv"
        "src/srv/SaveArea.srv"
        "src/srv/SaveDockingStation.srv"
        "src/srv/FindNearestDockingStation.srv"
        "src/msg/Area.msg"
        "src/msg/Map.msg"
        "src/msg/DockingStation.msg"
        "src/action/DockRobotTo.action"
        "src/action/DockRobotNearest.action"
        "src/action/RecordDockingStation.action"
        "src/action/RecordAreaBoundary.action"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_geometry_msgs
        unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

include(cmake/map_server.cmake)
include(cmake/sim.cmake)
include(cmake/docking_helper.cmake)
include(cmake/map_recorder.cmake)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

INSTALL(
        DIRECTORY config launch description worlds maps
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
