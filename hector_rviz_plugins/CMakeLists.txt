cmake_minimum_required(VERSION 3.5)
project(hector_rviz_plugins VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hector_rviz_plugins_msgs REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_AUTOMOC ON)

###########
## Build ##
###########

set(HEADERS
  include/hector_rviz_plugins/hector_view_controller.hpp
  include/hector_rviz_plugins/multi_robot_state_display.hpp
  include/hector_rviz_plugins/pointcloud_filter_display.hpp
)

set(SOURCES
  src/hector_view_controller.cpp
  src/multi_robot_state_display.cpp
  src/pointcloud_filter_display.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} hector_rviz_plugins_msgs moveit_ros_visualization rviz2 rviz_default_plugins rviz_ogre_vendor std_msgs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hector_rviz_plugins_msgs
  moveit_ros_visualization
  rviz2
  rviz_default_plugins
  rviz_ogre_vendor
  std_msgs
)

ament_package()
