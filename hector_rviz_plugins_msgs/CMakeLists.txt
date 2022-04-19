cmake_minimum_required(VERSION 3.0.2)
project(hector_rviz_plugins_msgs VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation moveit_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    GraphNode.msg
    Graph.msg
    MultiRobotStateEntry.msg
    DisplayMultiRobotState.msg
    ViewMode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveEye.srv
  MoveEyeAndFocus.srv
  SetViewMode.srv
  TrackFrame.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs moveit_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs moveit_msgs
)
