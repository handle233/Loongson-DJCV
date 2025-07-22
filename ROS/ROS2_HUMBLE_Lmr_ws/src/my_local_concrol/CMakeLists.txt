cmake_minimum_required(VERSION 3.5)
project(my_local_concrol)

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

add_library(my_local_concrol SHARED src/my_local_concrol.cpp)
ament_target_dependencies(my_local_concrol
    rclcpp nav2_core nav2_util nav2_costmap_2d
    geometry_msgs nav_msgs tf2 tf2_geometry_msgs pluginlib)

pluginlib_export_plugin_description_file(nav2_core controller_plugin.xml)

install(TARGETS my_local_concrol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

ament_package()
