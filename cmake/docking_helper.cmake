###
# docking_helper
###
add_executable(docking_helper
        src/docking_helper/main.cpp
        src/docking_helper/docking_helper_node.cpp)
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
  omros2_firmware_msgs
  std_msgs
)

INSTALL(TARGETS docking_helper
        DESTINATION lib/${PROJECT_NAME})

add_library(charger_presence_charging_dock SHARED
        src/docking_helper/charger_presence_charging_dock.cpp)
target_compile_features(charger_presence_charging_dock PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(charger_presence_charging_dock PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(charger_presence_charging_dock
  pluginlib
  opennav_docking_core
  omros2_firmware_msgs
  std_msgs
  rclcpp_lifecycle
  tf2_ros
)

INSTALL(TARGETS charger_presence_charging_dock
        DESTINATION lib/${PROJECT_NAME})

INSTALL(FILES src/docking_helper/charger_presence_charging_dock.hpp
        DESTINATION include/${PROJECT_NAME}/docking_helper)

set_target_properties(charger_presence_charging_dock PROPERTIES LINKER_LANGUAGE CXX)
pluginlib_export_plugin_description_file(opennav_docking_core src/docking_helper/plugins.xml)

add_dependencies(docking_helper ${PROJECT_NAME})
