# hector_rviz_plugins
Collection of RViz plugins.

## HectorViewController
A view controller with support for animation, movement using the arrow keys, 2D and 3D mode as well as ROS interfaces for control by external nodes.
It can also track a specified frame allowing the camera to follow the robot.
The tracked frame is followed with a P-controller with variable P-Gain to dampen the camera movements.

## MultiRobotStateDisplay
An RViz display to display multiple robot states with possibly differing positions and orientations.

![Two robot states with different poses](multi_robot_state_display.png)

## PointCloudFilter
An RViz display that allows to filter a point cloud spatially, i.e. removes points that lie above a specified theshold in a given direction (x,y,z) or max radial distance of a coordinate system.

![Filtering points with too big z-values](point_cloud_filter_display.png)
