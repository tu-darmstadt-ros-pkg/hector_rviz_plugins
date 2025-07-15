cmake_minimum_required(VERSION 3.5)
project(hector_rviz_plugins_msgs VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphNode.msg"
  "msg/Graph.msg"
  "msg/MultiRobotStateEntry.msg"
  "msg/DisplayMultiRobotState.msg"
  "msg/ViewMode.msg"
  "msg/RelativeViewControllerCmd.msg"
  "srv/MoveEye.srv"
  "srv/MoveEyeAndFocus.srv"
  "srv/SetViewMode.srv"
  "srv/TrackFrame.srv"
  DEPENDENCIES geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
