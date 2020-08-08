cmake_minimum_required(VERSION 3.0.2)
project(hector_rviz_plugins_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation moveit_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    MultiRobotStateEntry.msg
    DisplayMultiRobotState.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
