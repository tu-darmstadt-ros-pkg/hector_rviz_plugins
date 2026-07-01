/*
 * Copyright (C) 2020  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_HPP
#define HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_HPP

#include <hector_rviz_plugins_msgs/msg/display_multi_robot_state.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <unordered_map>

namespace rviz_common::properties
{
class StringProperty;
} // namespace rviz_common::properties

namespace hector_rviz_plugins
{
class PrivateRobotStateDisplayHelper;

class MultiRobotStateDisplay
    : public rviz_common::RosTopicDisplay<hector_rviz_plugins_msgs::msg::DisplayMultiRobotState>
{
  Q_OBJECT
public:
  MultiRobotStateDisplay();

  ~MultiRobotStateDisplay() override;

  void update( float wall_dt, float ros_dt ) override;

  void reset() override;

protected slots:
  void onSubdisplayEnableChanged();

  // Re-subscribes every entry that does not specify its own robot_description to the new default.
  void onDefaultRobotDescriptionChanged();

protected:
  void
  processMessage( hector_rviz_plugins_msgs::msg::DisplayMultiRobotState::ConstSharedPtr msg ) override;

  struct RobotEntry {
    std::unique_ptr<PrivateRobotStateDisplayHelper> display;
    geometry_msgs::msg::PoseStamped pose;
    moveit_msgs::msg::DisplayRobotState::ConstSharedPtr state;
    // Raw robot_description from the message (empty = follows the display default).
    std::string robot_description;
  };

  std::unordered_map<std::string, RobotEntry> robots_;
  hector_rviz_plugins_msgs::msg::DisplayMultiRobotState::ConstSharedPtr last_message_;
  rviz_common::properties::StringProperty *default_robot_description_property_ = nullptr;
  bool needs_state_update_ = false;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_HPP
