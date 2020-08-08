cmake_minimum_required(VERSION 3.0.2)
project(hector_rviz_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS hector_rviz_plugins_msgs moveit_ros_visualization rviz)
find_package(Qt5 REQUIRED COMPONENTS Core Gui)
set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_rviz_plugins
  CATKIN_DEPENDS hector_rviz_plugins_msgs moveit_ros_visualization rviz
  DEPENDS Qt5Core Qt5Gui
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)

set(HEADERS
  include/hector_rviz_plugins/multi_robot_state_display.h
)

set(SOURCES
  src/multi_robot_state_display.cpp
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
