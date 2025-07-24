cmake_minimum_required(VERSION 3.5)
project(my_local_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_local_control SHARED src/my_local_control.cpp)
target_include_directories(my_local_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(my_local_control
    rclcpp nav2_core nav2_util nav2_costmap_2d
    geometry_msgs nav_msgs tf2 tf2_geometry_msgs pluginlib)

pluginlib_export_plugin_description_file(nav2_core controller_plugin.xml)

install(TARGETS my_local_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

ament_package()
