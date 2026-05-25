###
# sim_node
###
add_executable(sim_node
        src/sim/node_main.cpp
        src/sim/sim_node.cpp)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(sim_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_node "${cpp_typesupport_target}")

ament_target_dependencies(sim_node
        geometry_msgs
        nav_msgs
        rclcpp
        sensor_msgs
        std_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
)

INSTALL(TARGETS sim_node
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(sim_node ${PROJECT_NAME})

###
# navigation_readiness_node
###
add_executable(navigation_readiness_node
        src/sim/navigation_readiness_node.cpp)
target_compile_features(navigation_readiness_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(navigation_readiness_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigation_readiness_node "${cpp_typesupport_target}")

ament_target_dependencies(navigation_readiness_node
        nav_msgs
        rclcpp
        sensor_msgs
        tf2
        tf2_ros
)

INSTALL(TARGETS navigation_readiness_node
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(navigation_readiness_node ${PROJECT_NAME})
