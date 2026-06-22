###
# map_recorder
###
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
  omros2_firmware_msgs
  std_msgs
  std_srvs
)

INSTALL(TARGETS map_recorder
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(map_recorder ${PROJECT_NAME})
