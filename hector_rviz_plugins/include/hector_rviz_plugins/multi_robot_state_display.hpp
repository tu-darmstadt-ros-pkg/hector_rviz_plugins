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
#include <rclcpp/subscription.hpp>
#include <rviz_common/display.hpp>
#include <unordered_map>

namespace rviz_common::properties
{
class RosTopicProperty;
} // namespace rviz_common::properties

namespace hector_rviz_plugins
{
class PrivateRobotStateDisplayHelper;

class MultiRobotStateDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  MultiRobotStateDisplay();

  ~MultiRobotStateDisplay() override;

  void update( float wall_dt, float ros_dt ) override;

protected slots:
  void onTopicChanged();

  void onSubdisplayEnableChanged();

public:
  void onEnableChanged() override;

protected:
  void onNewMultiRobotState(
      const hector_rviz_plugins_msgs::msg::DisplayMultiRobotState::ConstSharedPtr &msg );

  std::unordered_map<std::string, std::unique_ptr<PrivateRobotStateDisplayHelper>> displays_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> poses_;
  hector_rviz_plugins_msgs::msg::DisplayMultiRobotState::ConstSharedPtr last_message_;
  rclcpp::Subscription<hector_rviz_plugins_msgs::msg::DisplayMultiRobotState>::SharedPtr sub_;
  rviz_common::properties::RosTopicProperty *topic_property_;
  bool needs_state_update_ = false;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_STATE_DISPLAY_HPP
