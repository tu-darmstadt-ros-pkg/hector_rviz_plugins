# A name identifying this robot, used to optimize updates
string id
# Per-robot robot description (topic name) used to load this robot's URDF.
# Empty string means the display's default robot description is used.
string robot_description
# The frame can be set to override the frame in the main message
geometry_msgs/PoseStamped pose
moveit_msgs/DisplayRobotState robot_state
# Robot-wide alpha in [0, 1]. Defaults to fully opaque.
float32 alpha 1.0
# Whether the visual geometry is shown.
bool show_visual true
# Whether the collision geometry is shown.
bool show_collision false
