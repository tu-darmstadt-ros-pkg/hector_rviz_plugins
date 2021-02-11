cmake_minimum_required(VERSION 3.0.2)
project(hector_rviz_plugins VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS hector_rviz_plugins_msgs moveit_ros_visualization rviz std_msgs)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_rviz_plugins
  CATKIN_DEPENDS hector_rviz_plugins_msgs moveit_ros_visualization rviz
  DEPENDS Qt5Core Qt5Gui Qt5Widgets
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(HEADERS
  include/hector_rviz_plugins/hector_view_controller.h
  include/hector_rviz_plugins/multi_robot_state_display.h
)

set(SOURCES
  src/hector_view_controller.cpp
  src/multi_robot_state_display.cpp
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
