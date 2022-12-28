cmake_minimum_required(VERSION 3.8)
project(explore_frontier)

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

set(library_name explore_frontier_core)

add_library(${library_name} SHARED
  src/explore_frontier.cpp
)

target_include_directories(${library_name} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "nav2_msgs"
  "nav2_util"
)

rclcpp_components_register_nodes(${library_name} "explore_frontier::ExploreFrontier")

install( TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
