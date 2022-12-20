cmake_minimum_required(VERSION 3.8)
project(test_explore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(explore_frontier_test SHARED
  src/explore_frontier_test.cpp
)

target_include_directories(explore_frontier_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(explore_frontier_test
  "rclcpp"
  "rclcpp_action"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "nav2_msgs"
  "nav2_util"
)

rclcpp_components_register_node(explore_frontier_test PLUGIN "test_explore::ExploreFrontierTest" EXECUTABLE explore_frontier)

install( TARGETS explore_frontier_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
