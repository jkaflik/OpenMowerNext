cmake_minimum_required(VERSION 3.8)
project(open_mower_next)

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
find_package(Fields2Cover REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

option(OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS "Enable explicit integration tests" OFF)

rosidl_generate_interfaces(${PROJECT_NAME}
        "src/srv/RemoveArea.srv"
        "src/srv/RemoveDockingStation.srv"
        "src/srv/SaveArea.srv"
        "src/srv/SaveDockingStation.srv"
        "src/srv/FindNearestDockingStation.srv"
        "src/srv/PolygonCoverage.srv"
        "src/srv/AreaCoverage.srv"
        "src/msg/Area.msg"
        "src/msg/Map.msg"
        "src/msg/DockingStation.msg"
        "src/msg/PolygonWithHoles.msg"
        "src/msg/CoverageGeometry.msg"
        "src/action/DockRobotTo.action"
        "src/action/DockRobotNearest.action"
        "src/action/RecordDockingStation.action"
        "src/action/RecordAreaBoundary.action"
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
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
include(cmake/coverage_server.cmake)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    # These repository-wide linters pick up vendored deps under src/lib and docs/node_modules.
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    if (OPEN_MOWER_NEXT_ENABLE_INTEGRATION_TESTS)
        find_package(ament_cmake_pytest REQUIRED)
        ament_add_pytest_test(webots_smoke test/integration/test_webots_smoke.py TIMEOUT 180)
        set_tests_properties(webots_smoke PROPERTIES LABELS integration RESOURCE_LOCK webots)
        ament_add_pytest_test(navigation_docking test/integration/test_navigation_docking.py TIMEOUT 420)
        set_tests_properties(navigation_docking PROPERTIES LABELS integration RESOURCE_LOCK webots)
    endif ()
endif ()

INSTALL(
        DIRECTORY config launch description worlds maps protos resource
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
