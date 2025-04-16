/*
 * Copyright (C) 2025  Stefan Fabian
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

#ifndef HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_MODEL_HPP
#define HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_MODEL_HPP

#include <rviz_common/display.hpp>

namespace rviz_common::properties
{
class FloatProperty;
}

namespace rviz_default_plugins::displays
{
class RobotModelDisplay;
}

namespace hector_rviz_plugins
{

class MultiRobotModelDisplay : public rviz_common::Display
{
public:
  MultiRobotModelDisplay();

  ~MultiRobotModelDisplay() override;

  void onInitialize() override;
  void update( float wall_dt, float ros_dt ) override;

private:
  using RobotModelDisplayMap =
      std::map<std::string, std::unique_ptr<rviz_default_plugins::displays::RobotModelDisplay>,
               std::less<>>;
  RobotModelDisplayMap robot_model_displays_;
  float scan_age_ns_ = 999 * 1E9f;

  rviz_common::properties::BoolProperty *enable_scan_property_;
  rviz_common::properties::FloatProperty *scan_interval_property_;
};
} // namespace hector_rviz_plugins

#endif // HECTOR_RVIZ_PLUGINS_MULTI_ROBOT_MODEL_HPP
