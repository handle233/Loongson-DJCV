cmake_minimum_required(VERSION 3.8)
project(pid_path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# Add include directories
include_directories(include)

# Create the controller plugin library
add_library(pid_path_tracker_plugin SHARED
  src/pid_path_tracker.cpp
)
ament_target_dependencies(pid_path_tracker_plugin PUBLIC
  "rclcpp"
  "rclcpp_lifecycle"
  "nav2_core"
  "pluginlib"
  "tf2_ros"
  "nav_msgs"
  "geometry_msgs"
  "control_toolbox"
  "nav2_util"
  "nav2_costmap_2d"
)

# Export the plugin description file
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# Install targets
install(TARGETS
  pid_path_tracker_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()