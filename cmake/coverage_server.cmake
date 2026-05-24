###
# coverage_server
###
add_executable(coverage_server
  src/coverage_server/main.cpp
  src/coverage_server/coverage_server_node.cpp
)
target_compile_features(coverage_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(coverage_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(coverage_server
  "${cpp_typesupport_target}"
    Fields2Cover::Fields2Cover)
ament_target_dependencies(coverage_server
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  visualization_msgs
)

INSTALL(TARGETS coverage_server
        DESTINATION lib/${PROJECT_NAME})

add_dependencies(coverage_server ${PROJECT_NAME})

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(coverage_server_utils_test test/coverage_server_utils_test.cpp)
  if (TARGET coverage_server_utils_test)
    target_include_directories(coverage_server_utils_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    )
    target_link_libraries(coverage_server_utils_test
      "${cpp_typesupport_target}"
      Fields2Cover::Fields2Cover
    )
    add_dependencies(coverage_server_utils_test ${PROJECT_NAME})
    ament_target_dependencies(coverage_server_utils_test
      geometry_msgs
      nav_msgs
      tf2_geometry_msgs
    )
  endif ()
endif ()
