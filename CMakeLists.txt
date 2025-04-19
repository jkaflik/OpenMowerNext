cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
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
        geometry_msgs
        nav_msgs
        foxglove_msgs
        visualization_msgs
        rclcpp
        robot_localization
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

##
# map_server_node
##

###
# sim_node
###
add_executable(sim_node
        src/sim/node_main.cpp
        src/sim/sim_node.hpp
        src/sim/sim_node.cpp
        src/sim/sim_node.cpp
        src/sim/sim_node.hpp
        src/sim/node_main.cpp)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(sim_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_node "${cpp_typesupport_target}")

ament_target_dependencies(sim_node
        rclcpp
        tf2
        tf2_geometry_msgs
)

INSTALL(TARGETS sim_node
        DESTINATION lib/${PROJECT_NAME})

##
# sim_node
##
##

###
# docking_helper
###
add_executable(docking_helper
        src/docking_helper/main.cpp
        src/docking_helper/docking_helper_node.cpp
        src/docking_helper/docking_helper_node.hpp)
target_compile_features(docking_helper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(docking_helper PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(docking_helper "${cpp_typesupport_target}")

ament_target_dependencies(docking_helper
        rclcpp
        rclcpp_action
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav2_msgs
        geometry_msgs
)

INSTALL(TARGETS docking_helper
        DESTINATION lib/${PROJECT_NAME})

##
# docking_helper
##

##
# map_recorder
##
add_executable(map_recorder
  src/map_recorder/main.cpp
  src/map_recorder/map_recorder_node.cpp
)
target_compile_features(map_recorder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(map_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_recorder "${cpp_typesupport_target}")
ament_target_dependencies(map_recorder
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
  std_msgs
  std_srvs
)

INSTALL(TARGETS map_recorder
        DESTINATION lib/${PROJECT_NAME})

##
# map_recorder
##

# Create a dependency link for our executables on the message generation
add_dependencies(map_server_node ${PROJECT_NAME})
add_dependencies(sim_node ${PROJECT_NAME})
add_dependencies(docking_helper ${PROJECT_NAME})
add_dependencies(map_recorder ${PROJECT_NAME})

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
